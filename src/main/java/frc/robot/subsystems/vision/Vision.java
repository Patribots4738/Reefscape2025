// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Robot.GameMode;
import frc.robot.RobotContainer;
import frc.robot.util.Constants.CameraConstants;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.OIConstants;
import frc.robot.util.Constants.OIConstants.DriverMode;
import frc.robot.util.auto.Alignment.AlignmentMode;
import frc.robot.util.calc.PoseCalculations;

public class Vision extends SubsystemBase {

    private final VisionIOInputsAutoLogged[] inputs;
    private final VisionIO[] cameras;
    private boolean useMT1Override = false;

    // private final LoggedTunableNumber xyStdsDisabled = new LoggedTunableNumber("Vision/xyStdsDisabled", 0.001);
    // private final LoggedTunableNumber radStdsDisabled = new LoggedTunableNumber("Vision/RadStdsDisabled", 0.002);
    // private final LoggedTunableNumber xyStdsMultiTagTelopX = new LoggedTunableNumber("Vision/xyStdsMultiTagTelopX", 0.002);
    // private final LoggedTunableNumber xyStdsMultiTagTelopY = new LoggedTunableNumber("Vision/xyStdsMultiTagTelopY", 0.003);
    // private final LoggedTunableNumber xyStds2TagTelopX = new LoggedTunableNumber("Vision/xyStds2TagTelopX", 0.005);
    // private final LoggedTunableNumber xyStds2TagTelopY = new LoggedTunableNumber("Vision/xyStds2TagTelopY", 0.008);
    // private final LoggedTunableNumber xyStds2TagAutoX = new LoggedTunableNumber("Vision/xyStds2TagAutoX", 0.014);
    // private final LoggedTunableNumber xyStds2TagAutoY = new LoggedTunableNumber("Vision/xyStds2TagAutoY", 0.016);
    // private final LoggedTunableNumber radStdsMultiTag = new LoggedTunableNumber("Vision/RadStdsMultiTag", Units.degreesToRadians(2));
    // private final LoggedTunableNumber xyStds1TagLargeX = new LoggedTunableNumber("Vision/xyStds1TagLargeX", 0.015);
    // private final LoggedTunableNumber xyStds1TagLargeY = new LoggedTunableNumber("Vision/xyStds1TagLargeY", 0.033);
    // private final LoggedTunableNumber radStds1TagLarge = new LoggedTunableNumber("Vision/RaStds1TagLarge", Units.degreesToRadians(7));
    // private final LoggedTunableNumber minSingleTagArea = new LoggedTunableNumber("Vision/minSingleTagArea", 0.14);

    private final SwerveDrivePoseEstimator poseEstimator;
    private final Supplier<AlignmentMode> alignmentSupplier;

    @AutoLogOutput (key="Subsystems/Vision/RotationUpdated")
    private boolean rotationUpdated = false;

    public Vision(SwerveDrivePoseEstimator poseEstimator, Supplier<AlignmentMode> alignmentSupplier, VisionIO... io) {
        this.alignmentSupplier = alignmentSupplier;
        int cameraCount = io.length;
        cameras = new VisionIO[cameraCount];
        inputs = new VisionIOInputsAutoLogged[cameraCount];
        for (int i = 0; i < cameraCount; i++) {
            cameras[i] = io[i];
            inputs[i] = new VisionIOInputsAutoLogged();
        }
        this.poseEstimator = poseEstimator;
    }

    @Override
    public void periodic() {
        boolean shouldUseMT2 = !shouldUseMT1();
        double currentYawDegrees = poseEstimator.getEstimatedPosition().getRotation().getDegrees();
        for (int i = 0; i < cameras.length; i++) {
            VisionIO camera = cameras[i];

            if (Robot.gameMode == GameMode.DISABLED && !DriverStation.isFMSAttached()) {
                // If robot is disabled, only process some frames
                // This should minimize overheating issues with the LL4
                camera.setThrottle(CameraConstants.DISABLED_THROTTLE);
            } else {
                // If robot is enabled, process every frame
                camera.setThrottle(CameraConstants.ENABLED_THROTTLE);
            }

            camera.setUseMegaTag2(shouldUseMT2);
            camera.setRobotOrientation(currentYawDegrees);
            camera.updateInputs(inputs[i]);
            Logger.processInputs("SubsystemInputs/Vision/Camera" + i, inputs[i]);

            // Logger.recordOutput("Subsystems/Vision/Camera" + i + "MT1Pose", LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-four").pose);
            // Logger.recordOutput("Subsystems/Vision/Camera" + i + "MT2Pose", LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-four").pose);

            if (alignmentSupplier.get() == AlignmentMode.REEF) {
                int reefTag = closestReefTag();
                Logger.recordOutput("Subsystems/Vision/ClosestReefTag", reefTag);
                camera.setUsedTags(new int[] { reefTag });
            } else {
                camera.setUsedTags(FieldConstants.REEF_TAGS);
            }
        }

        updatePoseEstimator();
    }

    private void updatePoseEstimator() {
        int tagCount = 0;
        double tagArea = 0;
        double tagDistance = 0;

        List<Integer> camerasToUpdate = new ArrayList<>();
        for (int i = 0; i < cameras.length; i++) {
            boolean cameraHasTarget = cameraHasTarget(i);
            Logger.recordOutput("Subsystems/Vision/Camera" + i + "HasTarget", cameraHasTarget);
            if (cameraHasTarget) {
                boolean shouldUpdate = false;
                // Find "true" ta, td and tag count based on tags we want to filter
                // This should remove any comp surprises with tags we aren't expecting
                for (int j = 0; j < inputs[i].tagIds.length; j++) {
                    if (FieldConstants.VALID_TAGS_LIST.contains(inputs[i].tagIds[j])) {
                        shouldUpdate = true;
                        tagCount++;
                        tagArea += inputs[i].tagAreas[j];
                        tagDistance += inputs[i].tagDistances[j];
                    }
                }
                // If we found a valid tag in the visible fiducials, use this camera to update pe
                if (shouldUpdate) {
                    camerasToUpdate.add(i);
                }
            }
        }
        // Take mean
        if (tagCount > 0) {
            tagArea /= tagCount;
            tagDistance /= tagCount;
        }

        Logger.recordOutput("Subsystems/Vision/TagCount", tagCount);
        Logger.recordOutput("Subsystems/Vision/TagArea", tagArea);
        Logger.recordOutput("Subsystems/Vision/TagDistance", tagDistance);

        double xyStds = 0.0;
        double radStds = 0.0;

        if ((Robot.gameMode == GameMode.DISABLED || 
            Robot.gameMode == GameMode.AUTONOMOUS
                && Robot.currentTimestamp - RobotContainer.gameModeStart < 1.75)
                && camerasToUpdate.size() > 0) {
            xyStds = 0.001;
            radStds = 0.002;
        } else if (camerasToUpdate.size() > 0) {
            if (tagCount > 1 || tagArea > 1.0) {
                // Multiple targets or one huge target (like reef tag)
                // Trust the vision even MORE
                xyStds = Math.hypot(0.002, 0.003);
                radStds = Units.degreesToRadians(2);
            }
            // 1 target with large area
            else if (tagArea > 0.14) {
                xyStds = Math.hypot(0.015, 0.033);
                radStds = Units.degreesToRadians(7);
            }
            // Conditions don't match to add a vision measurement
            else {
                return;
            }
        }

        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(xyStds, xyStds, radStds));
        for (int i : camerasToUpdate) {
            poseEstimator.addVisionMeasurement(inputs[i].robotPose, inputs[i].timestampSeconds);
        }
        if (!camerasToUpdate.isEmpty()) {
            rotationUpdated = true;
        }

        Logger.recordOutput("Subsystems/Vision/XYStdDev", xyStds);
        Logger.recordOutput("Subsystems/Vision/ThetaStdDev", radStds);
    }

    private boolean cameraHasTarget(int cameraIndex) {
        if (!inputs[cameraIndex].robotPoseValid
            || Double.isNaN(inputs[cameraIndex].robotPose.getX()) 
            || Double.isNaN(inputs[cameraIndex].robotPose.getY()) 
            || Double.isNaN(inputs[cameraIndex].robotPose.getRotation().getRadians())
            || inputs[cameraIndex].robotPose.equals(Pose2d.kZero)
            // || (shouldUseMT1() && inputs[cameraIndex].tagIds.length == 1 && inputs[cameraIndex].averageTA < 0.17)
        )
        {
            return false;
        }
        return true;
    }

    private int closestReefTag() {
        return PoseCalculations.getClosestReefSide(poseEstimator.getEstimatedPosition()).getTagId();
    }

    public Command toggleMT1Command() {
        return runOnce(() -> this.useMT1Override = !this.useMT1Override);
    }

    @AutoLogOutput (key = "Subsystems/Vision/MT1")
    private boolean shouldUseMT1() {
        return (!(Robot.exitAuto)) && OIConstants.DRIVER_MODE != DriverMode.CALIBRATION && (Robot.gameMode == GameMode.DISABLED || !rotationUpdated) || useMT1Override;
    }

}