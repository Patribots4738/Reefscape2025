// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.custom;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.calc.PoseCalculations;
import frc.robot.util.Constants.FieldConstants;

public class ReefSide {
    private Pose2d center;
    private Pose2d left;
    private Pose2d right;
    private int tagId;

    public ReefSide(Pose2d center, Pose2d left, Pose2d right, int tagId) {
        this.center = center;
        this.right = right;
        this.left = left;
        this.tagId = tagId;
    }

    public ReefSide(Pose2d center,boolean flipped, int tagId) {
        this.center = center;
        this.tagId = tagId;
        double adjustedRotation = center.getRotation().getRadians() - Math.toRadians(90);
        Pose2d left =
            new Pose2d(
                center.getX() + FieldConstants.POLE_DISTANCE * Math.cos(adjustedRotation),
                center.getY() + FieldConstants.POLE_DISTANCE * Math.cos(adjustedRotation),
                center.getRotation()
            );
        Pose2d right = 
            new Pose2d(
                center.getX() - FieldConstants.POLE_DISTANCE * Math.cos(adjustedRotation),
                center.getY() - FieldConstants.POLE_DISTANCE * Math.cos(adjustedRotation),
                center.getRotation()
            );
        this.left = flipped ? right : left;
        this.right = flipped ? left : right;
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

    public int getTagId() {
        return tagId;
    }

    public ReefSide otherAlliance(int newTagId) {
        return new ReefSide(
            PoseCalculations.mirrorPose(center),
            PoseCalculations.mirrorPose(left),
            PoseCalculations.mirrorPose(right),
            newTagId
        );
    }

}
