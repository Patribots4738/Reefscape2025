package frc.robot.subsystems.vision;

import frc.robot.util.hardware.limelight.Limelight;
import frc.robot.util.hardware.limelight.LimelightHelpers.RawFiducial;

public class VisionIOLimelight implements VisionIO {
    
    private final Limelight camera;

    public VisionIOLimelight(String name, boolean isLL4) {
        this.camera = new Limelight(name, true);
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
                camera.setIMUMode(3);
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