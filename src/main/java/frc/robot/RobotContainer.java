package frc.robot;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLogOutput;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot.GameMode;
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
import frc.robot.util.Constants.FieldConstants;
import frc.robot.util.Constants.OIConstants;
import frc.robot.util.auto.Alignment;
import frc.robot.util.auto.PathPlannerStorage;
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
    public static Pose2d robotPose2d = new Pose2d();
    @AutoLogOutput (key = "Draggables/RobotPose3d")
    public static Pose3d robotPose3d = new Pose3d();
    @AutoLogOutput (key = "Draggables/SwerveMeasuredStates")
    public static SwerveModuleState[] swerveMeasuredStates;
    @AutoLogOutput (key = "Draggables/SwerveDesiredStates")
    public static SwerveModuleState[] swerveDesiredStates;
    @AutoLogOutput (key = "Draggables/GameModeStart")
    public static double gameModeStart = 0;

    
    public RobotContainer() {

        System.out.println("Constructing Robot Container...");

        driver = new PatriBoxController(OIConstants.DRIVER_CONTROLLER_PORT, OIConstants.DRIVER_DEADBAND);
        operator = new PatriBoxController(OIConstants.OPERATOR_CONTROLLER_PORT, OIConstants.OPERATOR_DEADBAND);

        pdh = new PowerDistribution(30, ModuleType.kRev);
        pdh.setSwitchableChannel(false);

        swerve = new Swerve();
        vision = new Vision(swerve.getPoseEstimator(), new VisionIOLimelight("limelight-four", true));
        coralClaw = new CoralClaw(new CoralClawIOKraken());
        algaeClaw = new AlgaeClaw(new AlgaeClawIOKraken());
        elevator = new Elevator(new ElevatorIOKraken());
        wrist = new Wrist(new WristIOKraken());
        climb = new Climb(new ClimbIOKraken());

        superstructure = new Superstructure(algaeClaw, coralClaw, elevator, wrist, climb, swerve::getPose);
        alignment = new Alignment(swerve);

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

        AutoConstants.LOGGED_TELE_XY_GAINS.onChanged(Commands.parallel(
            Commands.run(() -> AutoConstants.TELE_HDC.getXController().setPID(
                AutoConstants.LOGGED_TELE_XY_GAINS.get().getP(),
                AutoConstants.LOGGED_TELE_XY_GAINS.get().getI(),
                AutoConstants.LOGGED_TELE_XY_GAINS.get().getD())),
            Commands.run(() -> AutoConstants.TELE_HDC.getYController().setPID(
                AutoConstants.LOGGED_TELE_XY_GAINS.get().getP(),
                AutoConstants.LOGGED_TELE_XY_GAINS.get().getI(),
                AutoConstants.LOGGED_TELE_XY_GAINS.get().getD()))).ignoringDisable(true));

        AutoConstants.LOGGED_TELE_THETA_GAINS.onChanged(
            Commands.run(() -> AutoConstants.TELE_HDC.getThetaController().setPID(
                AutoConstants.LOGGED_TELE_THETA_GAINS.get().getP(),
                AutoConstants.LOGGED_TELE_THETA_GAINS.get().getI(),
                AutoConstants.LOGGED_TELE_THETA_GAINS.get().getD())).ignoringDisable(true));

        configureButtonBindings();
        configureTimedEvents();

        pathPlannerStorage = new PathPlannerStorage();

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
        pathPlannerStorage.getAutoChooser().addOption("WristFFCharacterization", wrist.sysIdQuasistatic());
        pathPlannerStorage.getAutoChooser().addOption("ElevatorFFCharacterization", elevator.sysIdQuasistatic());
        pathPlannerStorage.getAutoChooser().addOption("ClimbFFCharacterization", climb.sysIdQuasistatic());
        
        prepareNamedCommands();

    }

    private void configureButtonBindings(){
        switch(OIConstants.DRIVER_MODE) {
            case DEV:
                configureDevBindings(driver);
                break;
            default:
                configureDriverBindings(driver);
                configureOperatorBindings(operator);
                break;
        }
    }

    private void configureTimedEvents() {}

    private void configureDriverBindings(PatriBoxController controller) {
        
        controller.start()
            .onTrue(swerve.resetOdometryCommand(FieldConstants::GET_RESET_ODO_POSITION));

        controller.rightStick()
            .toggleOnTrue(alignment.intakeAlignmentCommand(controller::getLeftX, controller::getLeftY));

        controller.a()
            .whileTrue(alignment.reefAlignmentCommand(controller::getLeftX, controller::getLeftY));

        controller.leftBumper()
            .onTrue(alignment.updateIndexCommand(-1));

        controller.rightBumper()
            .onTrue(alignment.updateIndexCommand(1));
      
    }

    private void configureOperatorBindings(PatriBoxController controller) {

        controller.leftBumper().onTrue(superstructure.coralIntakeCommand(controller::getLeftBumper));

        controller.a().onTrue(superstructure.algaeL2Command(controller::getAButton));

        controller.y().onTrue(superstructure.algaeL3Command(controller::getYButton));

        controller.rightBumper().onTrue(superstructure.coralPlaceCommand(controller::getRightBumper));

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

    }

    private void configureDevBindings(PatriBoxController controller) {

        controller.start()
            .onTrue(swerve.resetOdometryCommand(FieldConstants::GET_RESET_ODO_POSITION));

        // controller.a()
        //     .whileTrue(alignment.reefAlignmentCommand(controller::getLeftX, controller::getLeftY));

        // controller.y()
        //     .whileTrue(alignment.cageAlignmentCommand(controller::getLeftY));

        // controller.leftBumper()
        //     .onTrue(alignment.updateIndexCommand(-1));

        // controller.rightBumper()
        //     .onTrue(alignment.updateIndexCommand(1));

    }

    public void updateNTGains() {
        double AUTO_HDC_XY_P = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Translation/0-P").getDouble(-1);
        double AUTO_HDC_XY_I = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Translation/1-I").getDouble(-1);
        double AUTO_HDC_XY_D = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Translation/2-D").getDouble(-1);
        double AUTO_HDC_TH_P = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Rotation/0-P").getDouble(-1);
        double AUTO_HDC_TH_I = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Rotation/1-I").getDouble(-1);
        double AUTO_HDC_TH_D = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Rotation/2-D").getDouble(-1);

        if (AUTO_HDC_XY_P == -1 || AUTO_HDC_XY_I == -1 || AUTO_HDC_XY_D == -1 || AUTO_HDC_TH_P == -1 || AUTO_HDC_TH_I == -1 || AUTO_HDC_TH_D == -1) {
            NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Translation/0-P").setDouble(AutoConstants.AUTO_XY_GAINS.getP());
            NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Translation/1-I").setDouble(AutoConstants.AUTO_XY_GAINS.getI());
            NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Translation/2-D").setDouble(AutoConstants.AUTO_XY_GAINS.getD());
            NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Rotation/0-P").setDouble(AutoConstants.AUTO_THETA_GAINS.getP());
            NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Rotation/1-I").setDouble(AutoConstants.AUTO_THETA_GAINS.getI());
            NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Rotation/2-D").setDouble(AutoConstants.AUTO_THETA_GAINS.getD());
            return;
        } else {

            AutoConstants.AUTO_HDC = new PPHolonomicDriveController(
                new PIDConstants(
                    AUTO_HDC_XY_P,
                    AUTO_HDC_XY_I,
                    AUTO_HDC_XY_D),
                new PIDConstants(
                    AUTO_HDC_TH_P,
                    AUTO_HDC_TH_I,
                    AUTO_HDC_TH_D));
        }
    }

    public Command getAutonomousCommand() {
        return pathPlannerStorage.getSelectedAuto();
    }

    public void onDisabled() {
        swerve.stopDriving();
        pathPlannerStorage.updatePathViewerCommand().schedule();
        pathPlannerStorage.configureAutoChooser();

        // TODO: Extract this into a command file
        Commands.run(this::updateNTGains)
            .until(() -> Robot.gameMode != GameMode.DISABLED)
            .ignoringDisable(true)
            .schedule();
    }

    public void onEnabled() {
        gameModeStart = Robot.currentTimestamp;
        pathPlannerStorage.updatePathViewerCommand().schedule();
        freshCode = false;
    }
    
    private void prepareNamedCommands() {
        NamedCommands.registerCommand("CoralIntakeStart", superstructure.coralAutoIntakeStartCommand());
        NamedCommands.registerCommand("CoralIntakeStop", superstructure.coralAutoIntakeStopCommand());
        NamedCommands.registerCommand("ArmStow", superstructure.setSuperState(superstructure.STOW));
        NamedCommands.registerCommand("ArmL1", superstructure.setSuperState(superstructure.L1));
        NamedCommands.registerCommand("ArmL2", superstructure.setSuperState(superstructure.L2));
        NamedCommands.registerCommand("ArmL3", superstructure.setSuperState(superstructure.L3));
        NamedCommands.registerCommand("ArmL4", superstructure.setSuperState(superstructure.L4));
        NamedCommands.registerCommand("PlaceCoral", superstructure.coralAutoPlaceCommand());
    }

}
