package frc.robot.util.hardware.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.util.hardware.limelight.LimelightHelpers.PoseEstimate;
import frc.robot.util.hardware.limelight.LimelightHelpers.RawFiducial;

public class Limelight {
    
    private final String name;
    private final boolean isLimelight4;
    private boolean useMT2 = false;
    private int disabledThrottle = 0;
    private int enabledThrottle = 0;

    private PoseEstimate latestPoseEstimate;

    public Limelight(String name, boolean isLimelight4) {
        this.name = name;
        this.isLimelight4 = isLimelight4;

    }

    public void configureThrottle(int disabledThrottle, int enabledThrottle) {
        this.disabledThrottle = disabledThrottle;
        this.enabledThrottle = enabledThrottle;
    }

    public void configureThrottle(int disabledThrottle) {
        configureThrottle(disabledThrottle, 0);
    }

    public void setPipelineIndex(int newIndex) {
        LimelightHelpers.setPipelineIndex(name, newIndex);
    }

    public void setValidTags(int[] validTagIds) {
        LimelightHelpers.SetFiducialIDFiltersOverride(name, validTagIds);
    }

    public void setPriorityTag(int tagId) {
        LimelightHelpers.setPriorityTagID(name, tagId);
    }

    public void setCropWindow(double xMin, double xMax, double yMin, double yMax) {
        LimelightHelpers.setCropWindow(name, xMin, xMax, yMin, yMax);
    }

    public void setRobotRelativeCameraOffset(Pose3d offset) {
        LimelightHelpers.setCameraPose_RobotSpace(
            name, 
            offset.getX(),
            offset.getY(),
            offset.getZ(),
            offset.getRotation().getX(), 
            offset.getRotation().getY(), 
            offset.getRotation().getZ()
        );
    }

    public void setRobotOrientation(Rotation3d angularVelocity, Rotation3d angle) {
        LimelightHelpers.SetRobotOrientation(
            name, 
            angle.getZ(), 
            angularVelocity.getZ(),
            angle.getY(),
            angularVelocity.getY(),
            angle.getX(),
            angularVelocity.getX()
        );
    }

    public void setRobotOrientation(double yaw) {
        LimelightHelpers.SetRobotOrientation(
            name, 
            yaw, 
            0, 
            0, 
            0, 
            0, 
            0
        );
    }

    public void setIMUMode(int mode) {
        LimelightHelpers.SetIMUMode(this.name, mode);
    }

    public PoseEstimate refreshPoseEstimate() {
        latestPoseEstimate = useMT2 
                ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name) 
                : LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        return latestPoseEstimate;
    }

    public boolean hasValidPoseEstimate() {
        return LimelightHelpers.validPoseEstimate(latestPoseEstimate);
    }

    public Pose2d getRobotPose() {
        return latestPoseEstimate != null ? latestPoseEstimate.pose : new Pose2d();
    }

    public double getTimestamp() {
        return latestPoseEstimate != null ? latestPoseEstimate.timestampSeconds : 0;
    }

    public double getAverageTA() {
        return latestPoseEstimate != null ? latestPoseEstimate.avgTagArea : 0;
    }

    public double getAverageTD() {
        return latestPoseEstimate != null ? latestPoseEstimate.avgTagDist : 0;
    }

    public int getTagCount() {
        return latestPoseEstimate != null ? latestPoseEstimate.tagCount : 0;
    }

    public RawFiducial[] getRawFiducials() {
        return latestPoseEstimate != null ? latestPoseEstimate.rawFiducials : new RawFiducial[0];
    }

    public boolean getUseMT2() {
        return useMT2;
    }

    public boolean getIsLL4() {
        return isLimelight4;
    }
    
    public void setUseMT2(boolean megaTag2) {
        useMT2 = megaTag2;
    }
}
