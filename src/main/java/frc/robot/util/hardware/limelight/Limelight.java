package frc.robot.util.hardware.limelight;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.util.hardware.limelight.LimelightHelpers.LimelightResults;

public class Limelight {
    
    private final String name;

    private LimelightResults results;

    public Limelight(String name) {
        this.name = name;
        results = LimelightHelpers.getLatestResults(name);
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

    public LimelightResults getJSONData() {
        results = LimelightHelpers.getLatestResults(name);
        return results;
    }

}
