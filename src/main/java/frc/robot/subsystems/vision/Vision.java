// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

        double tagCount = inputs.frontTagCount + inputs.backTagCount;
        double tagArea = (inputs.frontAverageTA + inputs.backAverageTA) / 2.0;

        if (shouldUpdatePoseFront()) {
            poseEstimator.addVisionMeasurement(inputs.frontRobotPose, inputs.frontTimestampSeconds);
        }
        if (shouldUpdatePoseBack()) {
            poseEstimator.addVisionMeasurement(inputs.backRobotPose, inputs.backTimestampSeconds);
        }
    }

    private boolean shouldUpdatePoseFront() {
        return true;
    }

    private boolean shouldUpdatePoseBack() {
        return true;
    }
}
