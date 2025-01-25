// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Robot.GameMode;
import frc.robot.RobotContainer;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.custom.LoggedTunableNumber;

public class Vision extends SubsystemBase {

    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
    private final VisionIO io;

    private final LoggedTunableNumber xyStdsDisabled = new LoggedTunableNumber("Vision/xyStdsDisabled", 0.001);
    private final LoggedTunableNumber radStdsDisabled = new LoggedTunableNumber("Vision/RadStdsDisabled", 0.002);
    private final LoggedTunableNumber xyStdsMultiTagTelopX = new LoggedTunableNumber("Vision/xyStdsMultiTagTelopX", 0.002);
    private final LoggedTunableNumber xyStdsMultiTagTelopY = new LoggedTunableNumber("Vision/xyStdsMultiTagTelopY", 0.003);
    private final LoggedTunableNumber xyStds2TagTelopX = new LoggedTunableNumber("Vision/xyStds2TagTelopX", 0.005);
    private final LoggedTunableNumber xyStds2TagTelopY = new LoggedTunableNumber("Vision/xyStds2TagTelopY", 0.008);
    private final LoggedTunableNumber xyStds2TagAutoX = new LoggedTunableNumber("Vision/xyStds2TagAutoX", 0.014);
    private final LoggedTunableNumber xyStds2TagAutoY = new LoggedTunableNumber("Vision/xyStds2TagAutoY", 0.016);
    private final LoggedTunableNumber radStdsMultiTag = new LoggedTunableNumber("Vision/RadStdsMultiTag", Units.degreesToRadians(2));
    private final LoggedTunableNumber xyStds1TagLargeX = new LoggedTunableNumber("Vision/xyStds1TagLargeX", 0.015);
    private final LoggedTunableNumber xyStds1TagLargeY = new LoggedTunableNumber("Vision/xyStds1TagLargeY", 0.033);
    private final LoggedTunableNumber radStds1TagLarge = new LoggedTunableNumber("Vision/RaStds1TagLarge", Units.degreesToRadians(7));
    private final LoggedTunableNumber minSingleTagArea = new LoggedTunableNumber("Vision/minSingleTagArea", 0.14);


    private final SwerveDrivePoseEstimator poseEstimator;

    public Vision(VisionIO io, SwerveDrivePoseEstimator poseEstimator) {
        this.io = io;
        this.poseEstimator = poseEstimator;
    }

    @Override
    public void periodic() {

        io.updateInputs(inputs);
        Logger.processInputs("SubsystemInputs/Vision", inputs);

        if (Robot.gameMode == Robot.GameMode.DISABLED) {
            io.setMegaTag2(false);
        } else io.setMegaTag2(true);

        if (!FieldConstants.IS_SIMULATION) {
            io.setRobotOrientation(poseEstimator.getEstimatedPosition().getRotation().getDegrees());
            updatePoseEstimator();
        }
    }

    private void updatePoseEstimator() {
        boolean updateFront = hasTargetFront();
        boolean updateBack = hasTargetBack();
        int tagCount = 0;
        double tagArea = 0;

        // Use correct inputs for standard deviations
        if (updateFront && updateBack) {
            tagCount = inputs.frontTagCount + inputs.backTagCount;
            tagArea = (inputs.frontAverageTA + inputs.backAverageTA) / 2.0;
        } else if (updateFront) {
            tagCount = inputs.frontTagCount;
            tagArea = inputs.frontAverageTA;
        } else if (updateBack) {
            tagCount = inputs.backTagCount;
            tagArea = inputs.backAverageTA;
        } else {
            return;
        }

        double xyStds = 0.0;
        double radStds = 0.0;

        if ((Robot.gameMode == GameMode.DISABLED || 
            Robot.gameMode == GameMode.AUTONOMOUS
                && Robot.currentTimestamp - RobotContainer.gameModeStart < 1.75)
                && (updateFront || updateBack)) {
            xyStds = xyStdsDisabled.get();
            radStds = radStdsDisabled.get();
        } else if (updateFront || updateBack) {
            // Multiple targets detected
            if (tagCount > 1) {
                if (Robot.gameMode == GameMode.TELEOP) {
                    // Trust the vision even MORE
                    if (tagCount > 2) {
                        xyStds = Math.hypot(xyStdsMultiTagTelopX.get(), xyStdsMultiTagTelopY.get());
                    } else {
                        // We can only see two tags, (still trustable)
                        xyStds = Math.hypot(xyStds2TagTelopX.get(), xyStds2TagTelopY.get());
                    }
                } else {
                    xyStds = Math.hypot(xyStds2TagAutoX.get(), xyStds2TagAutoY.get());
                }
                radStds = radStdsMultiTag.get();
            }
            // 1 target with large area and close to estimated roxose
            else if (tagArea > minSingleTagArea.get()) {
                xyStds = Math.hypot(xyStds1TagLargeX.get(), xyStds1TagLargeY.get());
                radStds = radStds1TagLarge.get();
            }
            // Conditions don't match to add a vision measurement
            else {
                return;
            }

            poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(xyStds, xyStds, radStds));
            if (updateFront) {
                poseEstimator.addVisionMeasurement(inputs.frontRobotPose, inputs.frontTimestampSeconds);
            }
            if (updateBack) {
                poseEstimator.addVisionMeasurement(inputs.backRobotPose, inputs.backTimestampSeconds);
            }
        }
    }

    private boolean hasTargetFront() {
        if (!inputs.frontRobotPoseValid
            || Double.isNaN(inputs.frontRobotPose.getX()) 
            || Double.isNaN(inputs.frontRobotPose.getY()) 
            || Double.isNaN(inputs.frontRobotPose.getRotation().getRadians()))
        {
            return false;
        }
        return true;
    }

    private boolean hasTargetBack() {
        if (!inputs.backRobotPoseValid
            || Double.isNaN(inputs.backRobotPose.getX()) 
            || Double.isNaN(inputs.backRobotPose.getY()) 
            || Double.isNaN(inputs.backRobotPose.getRotation().getRadians()))
        {
            return false;
        }
        return true;
    }

}
