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

public class Vision extends SubsystemBase {

    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
    private final VisionIO io;

    private final SwerveDrivePoseEstimator poseEstimator;
    
    public Vision(VisionIO io, SwerveDrivePoseEstimator poseEstimator) {
        this.io = io;
        this.poseEstimator = poseEstimator;
    }

    @Override
    public void periodic() {

        io.updateInputs(inputs);
        Logger.processInputs("SubsystemInputs/Vision", inputs);

        updatePoseEstimator();
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
            xyStds = 0.001;
            radStds = 0.0002;
        } else if (updateFront || updateBack) {
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
        if (Double.isNaN(inputs.frontRobotPose.getX()) 
            || Double.isNaN(inputs.frontRobotPose.getY()) 
            || Double.isNaN(inputs.frontRobotPose.getRotation().getRadians()))
        {
            return false;
        }
        return true;
    }

    private boolean hasTargetBack() {
        if (Double.isNaN(inputs.backRobotPose.getX()) 
            || Double.isNaN(inputs.backRobotPose.getY()) 
            || Double.isNaN(inputs.backRobotPose.getRotation().getRadians()))
        {
            return false;
        }
        return true;
    }
}
