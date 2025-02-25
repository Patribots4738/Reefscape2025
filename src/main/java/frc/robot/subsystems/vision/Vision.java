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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Robot.GameMode;
import frc.robot.RobotContainer;
import frc.robot.util.Constants.CameraConstants;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.auto.Alignment.AlignmentMode;
import frc.robot.util.calc.PoseCalculations;

public class Vision extends SubsystemBase {

    private final VisionIOInputsAutoLogged[] inputs;
    private final VisionIO[] cameras;

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
                camera.setUsedTags(FieldConstants.VALID_TAGS);
            }
        }

        updatePoseEstimator();
    }

    private void updatePoseEstimator() {
        int tagCount = 0;
        double tagArea = 0;

        List<Integer> camerasToUpdate = new ArrayList<>();
        for (int i = 0; i < cameras.length; i++) {
            boolean updateCamera = cameraHasTarget(i);
            Logger.recordOutput("Subsystems/Vision/UpdateCamera" + i, updateCamera);
            if (updateCamera) {
                camerasToUpdate.add(i);
                tagCount += inputs[i].tagIds.length;
                tagArea = (tagArea * (camerasToUpdate.size() - 1) + inputs[i].averageTA) / camerasToUpdate.size();
            }
        }

        Logger.recordOutput("Subsystems/Vision/TagCount", tagCount);
        Logger.recordOutput("Subsystems/Vision/TagArea", tagArea);

        double xyStds = 0.0;
        double radStds = 0.0;

        if ((Robot.gameMode == GameMode.DISABLED || 
            Robot.gameMode == GameMode.AUTONOMOUS
                && Robot.currentTimestamp - RobotContainer.gameModeStart < 1.75)
                && camerasToUpdate.size() > 0) {
            xyStds = 0.001;
            radStds = 0.002;
        } else if (camerasToUpdate.size() > 0) {
            // Multiple targets detected
            if (tagCount > 1) {
                if (Robot.gameMode == GameMode.TELEOP) {
                    // Trust the vision even MORE
                    if (tagCount > 2) {
                        xyStds = Math.hypot(0.002, 0.003);
                    } else {
                        // We can only see two tags, (still trustable)
                        xyStds = Math.hypot(0.005, 0.008);
                    }
                } else {
                    xyStds = Math.hypot(0.014, 0.016);
                }
                radStds = Units.degreesToRadians(2);
            }
            // 1 target with large area and close to estimated roxose
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
            || !inputs[cameraIndex].robotPose.equals(Pose2d.kZero)
            || (shouldUseMT1() && inputs[cameraIndex].tagIds.length == 1 && inputs[cameraIndex].averageTA < 0.14))
        {
            return false;
        }
        return true;
    }

    private int closestReefTag() {
        return PoseCalculations.getClosestReefSide(poseEstimator.getEstimatedPosition()).getTagId();
    }

    @AutoLogOutput (key = "Subsystems/Vision/MT1")
    private boolean shouldUseMT1() {
        return Robot.gameMode == GameMode.DISABLED || !rotationUpdated;
    }

}