package frc.robot.subsystems.vision;

import frc.robot.util.hardware.limelight.Limelight;
import frc.robot.util.hardware.limelight.LimelightHelpers.RawFiducial;

public class VisionIOLimelight implements VisionIO {
    
    private final Limelight camera;

    public VisionIOLimelight(String name, boolean isLL4) {
        this.camera = new Limelight(name, isLL4);
    }

    public void updateInputs(VisionIOInputs inputs) {
        camera.refreshPoseEstimate();
        inputs.robotPoseValid = camera.hasValidPoseEstimate();
        inputs.robotPose = camera.getRobotPose();
        inputs.timestampSeconds = camera.getTimestamp();
        RawFiducial[] rawFiducials = camera.getRawFiducials();
        inputs.tagIds = new int[rawFiducials.length];
        inputs.tagAreas = new double[rawFiducials.length];
        inputs.tagDistances = new double[rawFiducials.length];
        for (int i = 0; i < rawFiducials.length; i++) {
            RawFiducial fid = rawFiducials[i];
            inputs.tagIds[i] = fid.id;
            inputs.tagAreas[i] = fid.ta;
            inputs.tagDistances[i] = fid.distToRobot;
        }
    }

    @Override
    public void setPipelineIndex(int index) {
        camera.setPipelineIndex(index);
    }

    @Override
    public void setThrottle(int throttle) {
        camera.setThrottle(throttle);
    }

    @Override
    public void setRobotOrientation(double yawDegrees) {
        camera.setRobotOrientation(yawDegrees);
    }

    @Override 
    public void setUseMegaTag2(boolean useMT2) {
        boolean isLL4 = camera.getIsLL4();
        if (useMT2) {
            if (isLL4) {
                camera.setIMUMode(2);
            }
        } else {
            if (isLL4) {
                camera.setIMUMode(1);
            } else {
                camera.setIMUMode(0);
            }
        }
        camera.setUseMT2(useMT2);
    }

    @Override
    public void setUsedTags(int[] tagIds) {
        camera.setValidTags(tagIds);
    }


}