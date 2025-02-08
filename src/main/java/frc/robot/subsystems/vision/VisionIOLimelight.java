package frc.robot.subsystems.vision;

import frc.robot.util.hardware.limelight.Limelight;
import frc.robot.util.hardware.limelight.LimelightHelpers.RawFiducial;

public class VisionIOLimelight implements VisionIO {
    
    private final Limelight camera;

    public VisionIOLimelight(String name) {
        this.camera = new Limelight(name, false);
    }

    public void updateInputs(VisionIOInputs inputs) {
        camera.refreshPoseEstimate();
        inputs.robotPoseValid = camera.hasValidPoseEstimate();
        if (inputs.robotPoseValid) {
            inputs.robotPose = camera.getRobotPose();
            inputs.timestampSeconds = camera.getTimestamp();
            inputs.averageTA = camera.getAverageTA();
            inputs.averageTD = camera.getAverageTD();

            RawFiducial[] rawFrontFiducials = camera.getRawFiducials();
            inputs.tagIds = new int[rawFrontFiducials.length];
            for (int i = 0; i < rawFrontFiducials.length; i++) {
                RawFiducial fid = rawFrontFiducials[i];
                inputs.tagIds[i] = fid.id;
            }
        }
    }

    @Override
    public void setPipelineIndex(int index) {
        camera.setPipelineIndex(index);
    }

    @Override
    public void setRobotOrientation(double yawDegrees) {
        camera.setRobotOrientation(yawDegrees);
    }

    @Override
    public void setUseMegaTag2(boolean megaTag2) {
        camera.setUseMT2(megaTag2);
    }

    @Override 
    public void setIMUMode(String limelight, int mode) {
        if (mode == 2 || mode == 0) {
            setUseMegaTag2(true);
        }
        else {
            setUseMegaTag2(false);
        }
        camera.setIMUMode(limelight, mode);
    }


}