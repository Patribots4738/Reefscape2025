// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.custom;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.calc.PoseCalculations;

/** Add your docs here. */
public class ReefSide {
    private Pose2d center;
    private Pose2d left;
    private Pose2d right;


    public ReefSide(Pose2d center, Pose2d left, Pose2d right) {
        this.center = center;
        this.right = right;
        this.left = left;
    }

    public Pose2d getCenter() { 
        return center;
    }
    
    public Pose2d getRight() {
        return right;
    }

    public Pose2d getLeft() {
        return left;
    }

    public Rotation2d getRotation() {
        return center.getRotation();
    }

    public void flipAlliance() {
        PoseCalculations.mirrorPose(left);
        PoseCalculations.mirrorPose(center);
        PoseCalculations.mirrorPose(right);
    }

    public ReefSide otherAlliance() {
        return new ReefSide(
            PoseCalculations.mirrorPose(center),
            PoseCalculations.mirrorPose(left),
            PoseCalculations.mirrorPose(right));
    }

    



}
