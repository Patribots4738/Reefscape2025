package frc.robot.util.auto;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Set;
import java.util.function.Consumer;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.Robot.GameMode;
import frc.robot.RobotContainer;
import frc.robot.util.Constants.AutoConstants;

/**
 * This file represents all of the auto paths that we will have
 * They will be primarily compiled through
 * PathPlannerTrajectory.importChoreoPaths,
 * with each segment having its own method 
 * to make sure that the modularity stays clean
 */
public class PathPlannerStorage {

    private final LoggedDashboardChooser<Command> autoChooser;
    private final Set<Subsystem> requirements;

    public static final ArrayList<Pose2d> AUTO_STARTING_POSITIONS = new ArrayList<Pose2d>();

    public static final HashMap<String, List<PathPlannerPath>> AUTO_PATHS = new HashMap<String, List<PathPlannerPath>>();

    /**
     * Creates a new AutoPathStorage object.
     */
    public PathPlannerStorage(Set<Subsystem> autoRequirements) {
        autoChooser = new LoggedDashboardChooser<>("Auto Routine");
        requirements = autoRequirements;
    }

    public Command autoCycle(char reefNode, String reefLevel) {
        String coralStation = reefNode > 'G' || reefNode == 'A' ? "CS1" : "CS2";
        String pathNameToReef = coralStation + "-" + reefNode;
        String pathNameToStation = reefNode + "-" + coralStation;
        PathPlannerPath pathToReef;
        PathPlannerPath pathToStation;
        try {
            pathToReef = PathPlannerPath.fromPathFile(pathNameToReef);
            pathToStation = PathPlannerPath.fromPathFile(pathNameToStation);
        } catch (Exception e) {
            return Commands.none();
        }

        return Commands.defer(
            () -> Commands.sequence(
                Commands.parallel(
                    AutoBuilder.followPath(pathToReef),
                    Commands.sequence(
                        Commands.waitSeconds(0.4),
                        NamedCommands.getCommand("Coral" + reefLevel)
                    )
                ),
                NamedCommands.getCommand("PlaceCoral"),
                Commands.parallel(
                    Commands.sequence(
                        AutoBuilder.followPath(pathToStation),
                        NamedCommands.getCommand("WaitForCoral")
                    ),
                    Commands.sequence(
                        Commands.waitSeconds(0.2),
                        NamedCommands.getCommand("CoralIntakeStart")
                    )   
                )
            ), requirements);
    }

    // By default we will just build this part in PP, however this will be handy in a jam
    public Command preload(int startPose, char reefNode, String reefLevel) {
        String coralStation = reefNode > 'G' || reefNode == 'A' ? "CS1" : "CS2";
        String pathNameToPreload = startPose + "-" + reefNode;
        String pathNameToStation = reefNode + "-" + coralStation;
        PathPlannerPath pathToPreload;
        PathPlannerPath pathToStation;
        try {
            pathToPreload = PathPlannerPath.fromPathFile(pathNameToPreload);
            pathToStation = PathPlannerPath.fromPathFile(pathNameToStation);
        } catch (Exception e) {
            return Commands.none();
        }

        return Commands.defer(
            () -> Commands.sequence(
                Commands.parallel(
                    AutoBuilder.followPath(pathToPreload),
                    NamedCommands.getCommand("Coral" + reefLevel)
                ),
                NamedCommands.getCommand("PlaceCoral"),
                Commands.parallel(
                    AutoBuilder.followPath(pathToStation),
                    NamedCommands.getCommand("CoralIntakeStart")
                )
            ), requirements);
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
                List<Pose2d> autoPoses = getAutoPoses(getSelectedAutoName());
                RobotContainer.field2d.getObject("path").setPoses(autoPoses);
                if (autoPoses.size() > 0)
                    RobotContainer.autoStartingPose = autoPoses.get(0);
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
