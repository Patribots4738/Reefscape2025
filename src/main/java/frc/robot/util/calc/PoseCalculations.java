// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.calc;

import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.custom.ReefSide;

/** Add your docs here. */
public class PoseCalculations {



    public static ReefSide getClosestReefSide(Pose2d pos) {
        return nearest(FieldConstants.GET_REEF_FACE_POSITIONS(), pos);
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

    public static ReefSide flipAlliance(ReefSide side) {
        side.flipAlliance();
        return side;
    }

    public static Rotation2d flipFieldRotation(Rotation2d rotation) {
        return new Rotation2d(Math.PI).minus(rotation);
    }
    

    public static ReefSide nearest(List<ReefSide> poses, Pose2d pos) {
        return Collections.min(
            poses,
            Comparator.comparing(
                    (ReefSide other) -> pos.getTranslation().getDistance(other.getCenter().getTranslation()))
                .thenComparing(
                    (ReefSide other) ->
                        Math.abs(pos.getRotation().minus(other.getRotation()).getRadians())));
    }
}
