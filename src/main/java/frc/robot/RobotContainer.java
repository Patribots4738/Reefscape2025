package frc.robot;

import java.util.Set;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.characterization.StaticCharacterization;
import frc.robot.commands.characterization.WheelRadiusCharacterization;
import frc.robot.commands.drive.Drive;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.claw.algae.AlgaeClaw;
import frc.robot.subsystems.superstructure.claw.algae.AlgaeClawIOKraken;
import frc.robot.subsystems.superstructure.claw.coral.CoralClaw;
import frc.robot.subsystems.superstructure.claw.coral.CoralClawIOKraken;
import frc.robot.subsystems.superstructure.climb.Climb;
import frc.robot.subsystems.superstructure.climb.ClimbIOKraken;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOKraken;
import frc.robot.subsystems.superstructure.wrist.Wrist;
import frc.robot.subsystems.superstructure.wrist.WristIOKraken;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.util.Constants.AutoConstants;
import frc.robot.util.Constants.CoralClawConstants;
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.OIConstants;
import frc.robot.util.auto.Alignment;
import frc.robot.util.auto.PathPlannerStorage;
import frc.robot.util.auto.Alignment.AlignmentMode;
import frc.robot.util.calc.PoseCalculations;
import frc.robot.util.custom.ActiveConditionalCommand;
import frc.robot.util.custom.PatriBoxController;

public class RobotContainer {

    private PowerDistribution pdh;

    @SuppressWarnings("unused")
    private EventLoop testButtonBindingLoop = new EventLoop();

    private final PatriBoxController driver;
    private final PatriBoxController operator;

    private boolean fieldRelativeToggle = true;
    private final BooleanSupplier robotRelativeSupplier;

    private final Swerve swerve;
    @SuppressWarnings("unused")
    private final Vision vision;
    private final CoralClaw coralClaw;
    private final AlgaeClaw algaeClaw;
    private final Elevator elevator;
    private final Wrist wrist;
    private final Climb climb;
    private final Superstructure superstructure;
    private final Alignment alignment;

    public static Field2d field2d = new Field2d();

    private PathPlannerStorage pathPlannerStorage;

    // Draggables
    @AutoLogOutput (key = "Draggables/Components3d")
    public static Pose3d[] components3d = {new Pose3d(), new Pose3d(), new Pose3d(), new Pose3d()};
    @AutoLogOutput (key = "Draggables/DesiredComponents3d")
    public static Pose3d[] desiredComponents3d = {new Pose3d(), new Pose3d(), new Pose3d(), new Pose3d()};
    @AutoLogOutput (key = "Draggables/FreshCode")
    public static boolean freshCode = true;
    @AutoLogOutput (key = "Draggables/RobotPose2d")
    public static Pose2d robotPose2d = Pose2d.kZero;
    @AutoLogOutput (key = "Draggables/SwerveMeasuredStates")
    public static SwerveModuleState[] swerveMeasuredStates;
    @AutoLogOutput (key = "Draggables/SwerveDesiredStates")
    public static SwerveModuleState[] swerveDesiredStates;
    @AutoLogOutput (key = "Draggables/GameModeStart")
    public static double gameModeStart = 0;
    @AutoLogOutput (key = "Draggables/AutoStartingPose")
    public static Pose2d autoStartingPose = Pose2d.kZero;
    @AutoLogOutput (key = "Draggables/Timer")
    public static double displayTime = 0.0;
    
    public RobotContainer() {

        System.out.println("Constructing Robot Container...");

        driver = new PatriBoxController(OIConstants.DRIVER_CONTROLLER_PORT, OIConstants.DRIVER_DEADBAND);
        operator = new PatriBoxController(OIConstants.OPERATOR_CONTROLLER_PORT, OIConstants.OPERATOR_DEADBAND);

        pdh = new PowerDistribution(30, ModuleType.kRev);
        pdh.setSwitchableChannel(false);

        swerve = new Swerve();
        alignment = new Alignment(swerve);
        vision = new Vision(swerve.getPoseEstimator(), alignment::getAlignmentMode, new VisionIOLimelight("limelight-four", true));
        coralClaw = new CoralClaw(new CoralClawIOKraken());
        algaeClaw = new AlgaeClaw(new AlgaeClawIOKraken());
        elevator = new Elevator(new ElevatorIOKraken());
        wrist = new Wrist(new WristIOKraken());
        climb = new Climb(new ClimbIOKraken());
        superstructure = new Superstructure(algaeClaw, coralClaw, elevator, wrist, climb, swerve::getPose);

        SmartDashboard.putData(field2d);

        driver.back().toggleOnTrue(
            Commands.runOnce(() -> fieldRelativeToggle = !fieldRelativeToggle)
        );
        robotRelativeSupplier = () -> fieldRelativeToggle;

        swerve.setDefaultCommand(new Drive(
            swerve,
            driver::getLeftY,
            driver::getLeftX,
            () -> -driver.getRightX()/1.6,
            robotRelativeSupplier,
            () -> (robotRelativeSupplier.getAsBoolean() && Robot.isRedAlliance())
        ));

        // AutoConstants.LOGGED_TELE_XY_GAINS.onChanged(Commands.parallel(
        //     Commands.runOnce(() -> AutoConstants.TELE_HDC.getXController().setPID(
        //         AutoConstants.LOGGED_TELE_XY_GAINS.get().getP(),
        //         AutoConstants.LOGGED_TELE_XY_GAINS.get().getI(),
        //         AutoConstants.LOGGED_TELE_XY_GAINS.get().getD())),
        //     Commands.runOnce(() -> AutoConstants.TELE_HDC.getYController().setPID(
        //         AutoConstants.LOGGED_TELE_XY_GAINS.get().getP(),
        //         AutoConstants.LOGGED_TELE_XY_GAINS.get().getI(),
        //         AutoConstants.LOGGED_TELE_XY_GAINS.get().getD()))).ignoringDisable(true));

        // AutoConstants.LOGGED_TELE_THETA_GAINS.onChanged(
        //     Commands.runOnce(() -> AutoConstants.TELE_HDC.getThetaController().setPID(
        //         AutoConstants.LOGGED_TELE_THETA_GAINS.get().getP(),
        //         AutoConstants.LOGGED_TELE_THETA_GAINS.get().getI(),
        //         AutoConstants.LOGGED_TELE_THETA_GAINS.get().getD())).ignoringDisable(true));

        configureButtonBindings();
        configureMiscTriggers();

        pathPlannerStorage = new PathPlannerStorage(Set.of(swerve, coralClaw, algaeClaw, elevator, wrist, climb));

        prepareNamedCommands();

        pathPlannerStorage.configureAutoChooser();
        pathPlannerStorage.getAutoChooser().addOption("WheelRadiusCharacterization",
            swerve.setWheelsOCommand()
            .andThen(Commands.waitSeconds(0.5))
            .andThen(new WheelRadiusCharacterization(swerve)));
        pathPlannerStorage.getAutoChooser().addOption("DriveFFCharacterization",
            new StaticCharacterization(
                swerve, 
                swerve::runDriveCharacterization, 
                swerve::getDriveCharacterizationVelocity));
        pathPlannerStorage.getAutoChooser().addOption("TurnFFCharacterization",
            new StaticCharacterization(
                swerve, 
                swerve::runTurnCharacterization, 
                swerve::getTurnCharacterizationVelocity));

        // pathPlannerStorage.getAutoChooser().addOption("WristQFCharacterization", wrist.sysIdQuasistaticForward());
        // pathPlannerStorage.getAutoChooser().addOption("WristQRCharacterization", wrist.sysIdQuasistaticReverse());
        // pathPlannerStorage.getAutoChooser().addOption("WristDFCharacterization", wrist.sysIdDynamicForward());
        // pathPlannerStorage.getAutoChooser().addOption("WristDRCharacterization", wrist.sysIdDynamicReverse());

        // pathPlannerStorage.getAutoChooser().addOption("ElevatorQFCharacterization", elevator.sysIdQuasistaticForward());
        // pathPlannerStorage.getAutoChooser().addOption("ElevatorQRCharacterization", elevator.sysIdQuasistaticReverse());
        // pathPlannerStorage.getAutoChooser().addOption("ElevatorDFCharacterization", elevator.sysIdDynamicForward());
        // pathPlannerStorage.getAutoChooser().addOption("ElevatorDRCharacterization", elevator.sysIdDynamicReverse());

        // pathPlannerStorage.getAutoChooser().addOption("ClimbQFCharacterization", climb.sysIdQuasistaticForward());
        // pathPlannerStorage.getAutoChooser().addOption("ClimbQRCharacterization", climb.sysIdQuasistaticReverse());
        // pathPlannerStorage.getAutoChooser().addOption("ClimbDFCharacterization", climb.sysIdDynamicForward());
        // pathPlannerStorage.getAutoChooser().addOption("ClimbDRCharacterization", climb.sysIdDynamicReverse());

    }

    private void configureButtonBindings(){
        switch(OIConstants.DRIVER_MODE) {
            case DEV:
                configureDevBindings(driver);
                break;
            case DOUBLE:
                configureDriverBindings(driver);
                configureOperatorBindings(operator);
                break;
            case CALIBRATION:
                configureCalibrationBindings(driver);
        }
    }

    private void configureMiscTriggers() {
        new Trigger(() -> alignment.getAlignmentMode() != AlignmentMode.NONE && swerve.atHDCPose())
            .whileTrue(Commands.run(() -> driver.setRumble(0.1))
                .finallyDo(() -> driver.setRumble(0)));
    }

    private void configureDriverBindings(PatriBoxController controller) {

        controller.start().onTrue(vision.toggleMT1Command());
        
        controller.rightStick()
            .toggleOnTrue(
                new ActiveConditionalCommand(
                    alignment.reefRotationalAlignmentCommand(controller::getLeftX, controller::getLeftY),
                    alignment.intakeRotationalAlignmentCommand(controller::getLeftX, controller::getLeftY),
                    () -> PoseCalculations.shouldReefAlign(swerve.getPose()) && coralClaw.hasPiece()
                ).until(() -> Math.hypot(controller.getRightX(), controller.getRightY()) > OIConstants.DRIVER_ALIGN_CANCEL_DEADBAND));

        controller.a()
            .whileTrue(alignment.reefFullAlignmentCommand());

        controller.b()
            .whileTrue(alignment.netAlignmentCommand(controller::getLeftX));

        controller.x()
            .whileTrue(alignment.intakeAlignmentCommand());

        controller.y()
            .whileTrue(alignment.cageAlignmentCommand(controller::getLeftY));

        controller.leftBumper()
            .onTrue(alignment.updateIndexCommand(-1));

        controller.rightBumper()
            .onTrue(alignment.updateIndexCommand(1));

        controller.rightTrigger()
            .onTrue(superstructure.placeCommand(controller::getRightTrigger));

        controller.leftTrigger()
            .onTrue(superstructure.tossAlgaeCommand(controller::getLeftTrigger));

        controller.povUp()
            .onTrue(superstructure.setSuperState(superstructure.CLIMB_READY));

        controller.povDown()
            .onTrue(superstructure.setSuperState(superstructure.CLIMB_FINAL));

        controller.povRight().onTrue(coralClaw.setPercentCommand(CoralClawConstants.OUTTAKE_PERCENT));

        controller.povLeft().onTrue(coralClaw.setPercentCommand(CoralClawConstants.INTAKE_PERCENT));
    
    
    }

    private void configureOperatorBindings(PatriBoxController controller) {

        controller.leftBumper().onTrue(superstructure.coralIntakeCommand(controller::getLeftBumper));

        controller.povLeft()
            .onTrue(superstructure.setSuperState(superstructure.L1));
        
        controller.povDown()
            .onTrue(superstructure.setSuperState(superstructure.L2));

        controller.povRight()
            .onTrue(superstructure.setSuperState(superstructure.L3));

        controller.povUp()
            .onTrue(superstructure.setSuperState(superstructure.L4));
        
        controller.b()
            .onTrue(superstructure.setSuperState(superstructure.STOW));

        controller.x()
            .onTrue(superstructure.coralPrepCommand());

        controller.a()
            .onTrue(superstructure.algaeRemovalCommand(controller::getAButton));

        controller.y()
            .onTrue(superstructure.setSuperState(superstructure.NET_PREP));

        controller.start().onTrue(coralClaw.setPercentCommand(CoralClawConstants.OUTTAKE_PERCENT));
        controller.back().onTrue(coralClaw.setPercentCommand(CoralClawConstants.INTAKE_PERCENT));

    }

    private void configureDevBindings(PatriBoxController controller) {

        controller.start()
            .onTrue(swerve.resetOdometryCommand(FieldConstants::GET_RESET_ODO_POSITION));

        controller.povLeft()
            .onTrue(superstructure.setSuperState(superstructure.L1));
        
        controller.povDown()
            .onTrue(superstructure.setSuperState(superstructure.L2));

        controller.povRight()
            .onTrue(superstructure.setSuperState(superstructure.L3));

        controller.povUp()
            .onTrue(superstructure.setSuperState(superstructure.L4));
        
        controller.x()
            .onTrue(superstructure.setSuperState(superstructure.STOW));

        controller.rightTrigger()
            .onTrue(superstructure.placeCommand(controller::getRightTrigger));

        controller.leftTrigger()
            .onTrue(superstructure.coralIntakeCommand(controller::getLeftTrigger));

        controller.a()
            .whileTrue(alignment.reefAlignmentCommand());

        // controller.y()
        //     .onTrue(superstructure.algaeL3Command(controller::getYButton));

        // controller.a()
        //     .onTrue(superstructure.algaeL2Command(controller::getAButton));

        controller.rightStick()
            .toggleOnTrue(
                new ActiveConditionalCommand(
                    alignment.reefRotationalAlignmentCommand(controller::getLeftX, controller::getLeftY),
                    alignment.intakeRotationalAlignmentCommand(controller::getLeftX, controller::getLeftY),
                    () -> PoseCalculations.shouldReefAlign(swerve.getPose()) && coralClaw.hasPiece()
                ).until(() -> Math.hypot(controller.getRightX(), controller.getRightY()) > OIConstants.DRIVER_ALIGN_CANCEL_DEADBAND));

        controller.leftBumper()
            .onTrue(alignment.updateIndexCommand(-1));

        controller.rightBumper()
            .onTrue(alignment.updateIndexCommand(1));

    }

    private void configureCalibrationBindings(PatriBoxController controller) {

        controller.a().onTrue(vision.toggleMT1Command().ignoringDisable(true));

    }

    public Command getAutonomousCommand() {
        return pathPlannerStorage.getSelectedAuto();
    }

    public void onDisabled() {
        swerve.stopDriving();
        pathPlannerStorage.updatePathViewerCommand().schedule();
        pathPlannerStorage.configureAutoChooser();
    }

    public void onEnabled() {
        gameModeStart = Robot.currentTimestamp;
        pathPlannerStorage.updatePathViewerCommand().schedule();
        freshCode = false;
    }
    
    private void prepareNamedCommands() {

        NamedCommands.registerCommand("CoralIntakeStart", superstructure.coralAutoIntakeStartCommand());
        NamedCommands.registerCommand("CoralIntakeStop", superstructure.coralAutoIntakeStopCommand());
        NamedCommands.registerCommand("Stow", superstructure.setSuperState(superstructure.STOW));
        NamedCommands.registerCommand("CoralL1", superstructure.setSuperState(superstructure.L1));
        NamedCommands.registerCommand("CoralL2", superstructure.setSuperState(superstructure.L2));
        NamedCommands.registerCommand("CoralL3", superstructure.setSuperState(superstructure.L3));
        NamedCommands.registerCommand("CoralL4", superstructure.setSuperState(superstructure.L4));
        NamedCommands.registerCommand("PlaceCoral", superstructure.coralPlaceCommandAuto());
        NamedCommands.registerCommand("WaitForCoral", Commands.waitUntil(coralClaw::hasPiece));
        
        for (int i = 0; i < 12; i++) {
            char currentNode = AutoConstants.REEF_NODES.charAt(i);

            for (int level = 1; level <= 4; level++) {
                String currentLevel = "L" + level;
                String commandName = currentNode + Integer.toString(level);
            
                NamedCommands.registerCommand(commandName, pathPlannerStorage.autoCycle(currentNode, currentLevel));
            }
        }

        // for (int start = 1; start <= 7; start++) {
        //     int currentPreload = start;

        //     for (int i = 0; i < 12; i++) {
        //         char currentNode = AutoConstants.REEF_NODES.charAt(i);

        //         for (int level = 1; level <= 4; level++) {
        //             String currentLevel = "L" + level;
        //             String commandName = currentPreload + "-" + currentNode + Integer.toString(level);

        //             NamedCommands.registerCommand(commandName, pathPlannerStorage.preload(currentPreload, currentNode, currentLevel));
        //         }
        //     }
        // }
    }
}