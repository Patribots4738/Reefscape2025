package frc.robot.subsystems.vision;

import frc.robot.util.hardware.limelight.Limelight;

public class VisionIOLimelight implements VisionIO {
    
    private final Limelight frontLimelight;
    private final Limelight backLimelight;

    public VisionIOLimelight() {
        frontLimelight = new Limelight("Front3G", true);
        backLimelight = new Limelight("Back3G", true);
    }

    public void updateInputs(VisionIOInputs inputs) {
        frontLimelight.refreshPoseEstimate();
        inputs.frontRobotPoseValid = frontLimelight.hasValidPoseEstimate();
        inputs.frontRobotPose = frontLimelight.getRobotPose();
        inputs.frontTimestampSeconds = frontLimelight.getTimestamp();
        inputs.frontAverageTA = frontLimelight.getAverageTA();
        inputs.frontAverageTD = frontLimelight.getAverageTD();
        inputs.frontTagCount = frontLimelight.getTagCount();

        backLimelight.refreshPoseEstimate();
        inputs.backRobotPoseValid = backLimelight.hasValidPoseEstimate();
        inputs.backRobotPose = backLimelight.getRobotPose();
        inputs.backTimestampSeconds = backLimelight.getTimestamp();
        inputs.backAverageTA = backLimelight.getAverageTA();
        inputs.backAverageTD = backLimelight.getAverageTD();
        inputs.backTagCount = backLimelight.getTagCount();
    }

    public void setFrontPipelineIndex(int index) {
        frontLimelight.setPipelineIndex(index);
    }

    public void setBackPipelineIndex(int index) {
        backLimelight.setPipelineIndex(index);
    }

    public void setRobotOrientation(double yaw) {
        frontLimelight.setRobotOrientation(yaw);
        backLimelight.setRobotOrientation(yaw);
    }

}
