package frc.robot.util.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.Constants.AutoConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Robot.GameMode;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.Consumer;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This file represents all of the auto paths that we will have
 * They will be primarily compiled through
 * PathPlannerTrajectory.importChoreoPaths,
 * with each segment having its own method 
 * to make sure that the modularity stays clean
 */
public class PathPlannerStorage {

    private final LoggedDashboardChooser<Command> autoChooser;

    public static final ArrayList<Pose2d> AUTO_STARTING_POSITIONS = new ArrayList<Pose2d>();

    public static final PathConstraints PATH_CONSTRAINTS =
        new PathConstraints(
            AutoConstants.MAX_SPEED_METERS_PER_SECOND, 
            AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED, 
            AutoConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, 
            AutoConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);

    public static final HashMap<String, List<PathPlannerPath>> AUTO_PATHS = new HashMap<String, List<PathPlannerPath>>();
    /**
     * Creates a new AutoPathStorage object.
     */
    public PathPlannerStorage() {
        autoChooser = new LoggedDashboardChooser<>("Auto Routine");
    }

    public void configureAutoChooser() {
        /**
         * Warning
         * 
         * AutoBuilder::buildAutoChooser will load all autos in the deploy directory. Since the deploy
         * process does not automatically clear the deploy directory, old auto files
         * that have since been deleted from the project could remain on the RIO,
         * therefore being added to the auto chooser.
         * 
         * To remove old options, the deploy directory will need to be cleared manually
         * via SSH, WinSCP, re-imaging the RIO, etc.
         */
        
        // Good news! 
        // This auto caches our paths so we don't need to manually load them
        
        for (String autoName : AutoConstants.AUTO_NAMES) {
            System.out.println("Configuring " + autoName);
            // Load the auto and add it to the auto chooser
            Command auto = AutoBuilder.buildAuto(autoName);
            autoChooser.addOption(autoName, auto);
            // Load the auto and add it to the list of paths 
            // for trajectory visualization
            List<PathPlannerPath> paths;
            try {
                paths = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
                Pose2d startingPosition = paths.get(0).getStartingHolonomicPose().get();
                PathPlannerStorage.AUTO_STARTING_POSITIONS.add(startingPosition);
                PathPlannerStorage.AUTO_PATHS.put(autoName, paths);
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
        System.out.println("Configured auto chooser");
        bindListener(getUpdatePathViewerCommand());
    }

    public Command getSelectedAuto() {
        return autoChooser.get();
    }

    public String getSelectedAutoName() {
        return autoChooser.getSendableChooser().getSelected();
    }

    public void bindListener(Consumer<String> consumer) {
        autoChooser.getSendableChooser().onChange(consumer);
    }

    public LoggedDashboardChooser<Command> getAutoChooser() {
        return this.autoChooser;
    }

    public Command updatePathViewerCommand() {
        return Commands.either(
            Commands.runOnce(() -> {
                RobotContainer.field2d.getObject("path")
                    .setPoses(getAutoPoses(getSelectedAutoName()));
            }),
            Commands.runOnce(() -> {
                RobotContainer.field2d.getObject("path").setPoses(new ArrayList<>());
            }),
            () -> Robot.gameMode == GameMode.DISABLED
        ).ignoringDisable(true);
    }

    private Consumer<String> getUpdatePathViewerCommand() {
        return (string) -> {
            updatePathViewerCommand().schedule();
        };
    }

    public List<Pose2d> getAutoPoses(String name) {
        List<PathPlannerPath> paths = PathPlannerStorage.AUTO_PATHS.get(name);
        List<Pose2d> autoPoses = new ArrayList<>();
        if (paths == null) return autoPoses;
        // Flip for red alliance
        // and add all the poses to the list
        for (PathPlannerPath path : paths) {
            List<Pose2d> pathPoses = 
                Robot.isRedAlliance() 
                    ? path.flipPath().getPathPoses() 
                    : path.getPathPoses();
            autoPoses.addAll(pathPoses);
        }
    
        return autoPoses;
    }
}
