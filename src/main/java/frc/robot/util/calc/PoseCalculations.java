// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.calc;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.Constants.FieldConstants;

/** Add your docs here. */
public class PoseCalculations {



    public static Pose2d getClosestReefSide(Pose2d pos) {
        return pos.nearest(FieldConstants.REEF_POSITIONS);
    }

    public static Pose2d getClosestCage(Pose2d pos) {
        return pos.nearest(FieldConstants.GET_CAGE_POSITIONS());
    }

    public static Pose2d flipPose(Pose2d pos) {
        return new Pose2d(flipTranslation(pos.getTranslation()), flipFieldRotation(pos.getRotation()));
    }

    public static Pose2d mirrorPose(Pose2d pos) {
        return new Pose2d(mirrorTranslation(pos.getTranslation()), flipFieldRotation(pos.getRotation()));
    }

    public static Translation2d flipTranslation(Translation2d pos) {
        return new Translation2d(FieldConstants.FIELD_MAX_LENGTH - pos.getX(), pos.getY());
    }

    public static Translation2d mirrorTranslation(Translation2d pos) {
        return new Translation2d(FieldConstants.FIELD_MAX_LENGTH - pos.getX(), FieldConstants.FIELD_MAX_HEIGHT - pos.getY());
    }

    public static Rotation2d flipFieldRotation(Rotation2d rotation) {
        return new Rotation2d(Math.PI).minus(rotation);
    }
}
