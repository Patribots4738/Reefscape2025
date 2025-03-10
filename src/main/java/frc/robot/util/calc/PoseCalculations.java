// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.calc;

import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;



import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.custom.ReefSide;

public class PoseCalculations {

    public static int nearestIndex(Pose2d pos, List<Pose2d> compareTo) {
        double minDistance = pos.getTranslation().getDistance(compareTo.get(0).getTranslation());
        int nearest = 0;
        for (int i = 0; i < compareTo.size(); i++) {
            Pose2d comparing = compareTo.get(i);
            double dist = pos.getTranslation().getDistance(comparing.getTranslation());
            if (dist < minDistance) {
                minDistance = dist;
                nearest = i;
            }
        }
        return nearest;
    }

    public static ReefSide getClosestReefSide(Pose2d pos) {
        return nearest(pos, FieldConstants.GET_REEF_FACE_POSITIONS());
    }

    public static boolean isHighAlgaeIndex(int index) {
        return (index % 2) == 0;
    }

    public static boolean isHighAlgaeReefSide(ReefSide side) {
        return isHighAlgaeIndex(FieldConstants.GET_REEF_FACE_POSITIONS().indexOf(side));
    }

    public static boolean isHighAlgaeReefSide(Pose2d pos) {
        return isHighAlgaeReefSide(getClosestReefSide(pos));
    }

    public static Pose2d getClosestCage(Pose2d pos) {
        return pos.nearest(FieldConstants.GET_CAGE_POSITIONS());
    }

    public static Pose2d flipPose(Pose2d pos) {
        return new Pose2d(flipTranslation(pos.getTranslation()), flipFieldRotation(pos.getRotation()));
    }

    // Note: this method works for the use case it was designed for (mirroring reef node poses for placing coral)
    // but I haven't tested it with any edge case, or any case where roll != 0
    public static Pose3d flipPose3d(Pose3d pos) {
        Translation2d translation = flipTranslation(pos.getTranslation().toTranslation2d());
        Rotation2d yaw = flipFieldRotation(Rotation2d.fromRadians(pos.getRotation().getZ()));
        Rotation2d pitch = flipFieldRotation(Rotation2d.fromRadians(pos.getRotation().getY())).unaryMinus();
        Rotation2d roll = flipFieldRotation(Rotation2d.fromRadians(pos.getRotation().getX()));
        Translation3d translation3d = new Translation3d(translation.getX(), translation.getY(), pos.getZ());
        Rotation3d rotation3d = new Rotation3d(roll.getRadians(), pitch.getRadians(), yaw.getRadians());
        return new Pose3d(translation3d, rotation3d);
    }

    public static Pose2d mirrorPose(Pose2d pos) {
        return new Pose2d(mirrorTranslation(pos.getTranslation()), mirrorFieldRotation(pos.getRotation()));
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

    public static Rotation2d mirrorFieldRotation(Rotation2d rotation) {
        return new Rotation2d(Math.PI).plus(rotation);
    }

    public static ReefSide nearest(Pose2d pos, List<ReefSide> poses) {
        return Collections.min(
            poses,
            Comparator.comparing(
                    (ReefSide other) -> pos.getTranslation().getDistance(other.getCenter().getTranslation()))
                .thenComparing(
                    (ReefSide other) ->
                        Math.abs(pos.getRotation().minus(other.getRotation()).getRadians())));
    }

    public static Pose3d nearestPose3d(Pose3d pos, List<Pose3d> poses) {
        if (poses.isEmpty()) {
            return pos;
        }
        return Collections.min(
            poses,
            Comparator.comparing(
                    (Pose3d other) -> pos.getTranslation().getDistance(other.getTranslation()))
                .thenComparing(
                    (Pose3d other) ->
                    // I don't want to do the math for the other axis
                        Math.abs(pos.getRotation().toRotation2d().minus(other.getRotation().toRotation2d()).getRadians())));
    }

    public static boolean nearReef(Pose2d pos) {
        return pos.getTranslation().getDistance(FieldConstants.GET_REEF_POSITION().getTranslation()) < FieldConstants.NEAR_REEF_METERS;
    }
  
    public static Pose2d getClosestCoralStation(Pose2d pos) {
        return pos.nearest(FieldConstants.GET_CORAL_STATION_POSITIONS());
    }

    public static boolean shouldReefAlign(Pose2d pos) {
        return pos.getTranslation().getDistance(getClosestCoralStation(pos).getTranslation()) > FieldConstants.INTAKE_ALIGNMENT_DISTANCE_METERS;
    }

    public static Pose3d getClosestCoralScoringNode(Pose3d pos) {
        return nearestPose3d(pos, Arrays.asList(FieldConstants.CORAL_PLACEMENT_POSITIONS));
    }

    public static Pose3d getClosestAlgaeRemovalNode(Pose3d pos) {
        Pose3d nearestPose = nearestPose3d(pos, FieldConstants.ALGAE_REMOVAL_LOCATIONS_LIST);
        FieldConstants.ALGAE_REMOVAL_LOCATIONS_LIST.remove(nearestPose);
        return nearestPose;
    }
  
}
