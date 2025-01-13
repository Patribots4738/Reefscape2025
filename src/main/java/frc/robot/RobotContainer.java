package frc.robot;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.commands.characterization.FeedForwardCharacterization;
import frc.robot.commands.characterization.StaticCharacterization;
import frc.robot.commands.characterization.WheelRadiusCharacterization;
import frc.robot.commands.drive.Drive;
import frc.robot.commands.logging.NTGainTuner;
import frc.robot.commands.managers.HDCTuner;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.ArmPosition;
import frc.robot.subsystems.superstructure.claw.Claw;
import frc.robot.subsystems.superstructure.claw.ClawIOKraken;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOKraken;
import frc.robot.subsystems.superstructure.wrist.Wrist;
import frc.robot.subsystems.superstructure.wrist.WristIOKraken;
import frc.robot.subsystems.superstructure.climb.Climb;
import frc.robot.subsystems.superstructure.climb.ClimbIOKraken;
import frc.robot.util.Constants.AutoConstants;
import frc.robot.util.Constants.ElevatorConstants;
import frc.robot.util.Constants.OIConstants;
import frc.robot.util.auto.PathPlannerStorage;
import frc.robot.util.custom.PatriBoxController;

public class RobotContainer {

    private PowerDistribution pdh;

    private EventLoop testButtonBindingLoop = new EventLoop();

    private final PatriBoxController driver;
    private final PatriBoxController operator;

    private boolean fieldRelativeToggle = true;
    private final BooleanSupplier robotRelativeSupplier;

    private final Swerve swerve;
    private final Claw claw;
    private final Elevator elevator;
    private final Wrist wrist;
    private final Climb climb;
    private final Superstructure superstructure;

    public static Field2d field2d = new Field2d();

    private PathPlannerStorage pathPlannerStorage;
    private static HDCTuner HDCTuner;

    // Draggables
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

    @AutoLogOutput (key = "Draggables/Mech2d")
    private LoggedMechanism2d mech;
    private LoggedMechanismRoot2d elevatorRoot;
    private LoggedMechanismRoot2d climbRoot;
    public static LoggedMechanismLigament2d elevatorMech;
    public static LoggedMechanismLigament2d wristMech;
    public static LoggedMechanismLigament2d climbMech;
    
    public RobotContainer() {

        System.out.println("Constructing Robot Container...");

        driver = new PatriBoxController(OIConstants.DRIVER_CONTROLLER_PORT, OIConstants.DRIVER_DEADBAND);
        operator = new PatriBoxController(OIConstants.OPERATOR_CONTROLLER_PORT, OIConstants.OPERATOR_DEADBAND);

        pdh = new PowerDistribution(30, ModuleType.kRev);
        pdh.setSwitchableChannel(false);

        swerve = new Swerve();
        claw = new Claw(new ClawIOKraken());
        elevator = new Elevator(new ElevatorIOKraken());
        wrist = new Wrist(new WristIOKraken());
        climb = new Climb(new ClimbIOKraken());

        superstructure = new Superstructure(claw, elevator, wrist, climb);

        SmartDashboard.putData(field2d);

        mech = new LoggedMechanism2d(3, 3);
        elevatorRoot = mech.getRoot("placer", 1.8, 0.1524);
        climbRoot = mech.getRoot("climber", 1.2, 0.1524);
        elevatorMech = elevatorRoot.append(new LoggedMechanismLigament2d("elevator", ElevatorConstants.ELEVATOR_BASE_HEIGHT_METERS, 90.0));
        wristMech = elevatorMech.append(new LoggedMechanismLigament2d("wrist", 0.4, 0.0));
        climbMech = climbRoot.append(new LoggedMechanismLigament2d("climb", 0.5, 0.0));

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

        HDCTuner = new HDCTuner(
            AutoConstants.TELE_HDC.getXController(),
            AutoConstants.TELE_HDC.getThetaController());

        configureButtonBindings();
        configureTimedEvents();

        pathPlannerStorage = new PathPlannerStorage();

        pathPlannerStorage.configureAutoChooser();
        pathPlannerStorage.getAutoChooser().addOption("WheelRadiusCharacterization",
            swerve.setWheelsOCommand()
            .andThen(Commands.waitSeconds(0.5))
            .andThen(new WheelRadiusCharacterization(swerve)));
        pathPlannerStorage.getAutoChooser().addOption("DriveFeedForwardCharacterization",
            new FeedForwardCharacterization(
                swerve, 
                swerve::runDriveCharacterization, 
                swerve::getDriveCharacterizationVelocity));
        pathPlannerStorage.getAutoChooser().addOption("TurnStaticCharacterization",
            new StaticCharacterization(
                swerve, 
                swerve::runTurnCharacterization, 
                swerve::getTurnCharacterizationVelocity));

        new NTGainTuner().schedule();
        
        prepareNamedCommands();

    }

    private void configureButtonBindings(){
        configureDriverBindings(driver);
        configureOperatorBindings(operator);
    }

    private void configureTimedEvents() {}

    private void configureDriverBindings(PatriBoxController controller) {

        controller.start().onTrue(
            Commands.runOnce(() -> swerve.resetOdometry(
                new Pose2d(
                    swerve.getPose().getTranslation(), 
                    Rotation2d.fromDegrees(
                        Robot.isRedAlliance() 
                        ? 0 
                        : 180))
            ), swerve)
        );

        controller.leftBumper().whileTrue(swerve.getSetWheelsX());
        controller.rightBumper().whileTrue(swerve.getSetWheelsO());
        controller.b().whileTrue(swerve.driveCharacterization());
        controller.y().whileTrue(swerve.getSetWheelsZero());
        controller.a().whileTrue(swerve.tuneTurnVelocityCommand());
    }

    private void configureOperatorBindings(PatriBoxController controller) {

        controller.povUp()
            .onTrue(superstructure.setArmPosition(ArmPosition.L4));

        controller.povLeft()
            .onTrue(superstructure.setArmPosition(ArmPosition.L3));

        controller.povRight()
            .onTrue(superstructure.setArmPosition(ArmPosition.L2));
        
        controller.povDown()
            .onTrue(superstructure.setArmPosition(ArmPosition.L1));

        controller.leftTrigger()
            .onTrue(superstructure.intakeCommand(controller::getLeftTrigger));

        controller.rightTrigger()
            .onTrue(superstructure.placeCommand(controller::getRightTrigger));

        controller.rightBumper()
            .onTrue(superstructure.outtakeCommand(controller::getRightBumper));

    }

    public void updateNTGains() {
        double AUTO_HDC_XY_P = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Translation/0-P").getDouble(-1);
        double AUTO_HDC_XY_I = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Translation/1-I").getDouble(-1);
        double AUTO_HDC_XY_D = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Translation/2-D").getDouble(-1);
        double AUTO_HDC_TH_P = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Rotation/0-P").getDouble(-1);
        double AUTO_HDC_TH_I = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Rotation/1-I").getDouble(-1);
        double AUTO_HDC_TH_D = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Rotation/2-D").getDouble(-1);

        double TELE_HDC_XY_P = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("HDC/Translation/0-P").getDouble(-1);
        double TELE_HDC_XY_I = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("HDC/Translation/1-I").getDouble(-1);
        double TELE_HDC_XY_D = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("HDC/Translation/2-D").getDouble(-1);
        double TELE_HDC_TH_P = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("HDC/Rotation/0-P").getDouble(-1);
        double TELE_HDC_TH_I = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("HDC/Rotation/1-I").getDouble(-1);
        double TELE_HDC_TH_D = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("HDC/Rotation/2-D").getDouble(-1);

        if (AUTO_HDC_XY_P == -1 || AUTO_HDC_XY_I == -1 || AUTO_HDC_XY_D == -1 || AUTO_HDC_TH_P == -1 || AUTO_HDC_TH_I == -1 || AUTO_HDC_TH_D == -1 ||
            TELE_HDC_XY_P == -1 || TELE_HDC_XY_I == -1 || TELE_HDC_XY_D == -1 || TELE_HDC_TH_P == -1 || TELE_HDC_TH_I == -1 || TELE_HDC_TH_D == -1) {
            NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Translation/0-P").setDouble(AutoConstants.XY_CORRECTION_P);
            NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Translation/1-I").setDouble(AutoConstants.XY_CORRECTION_I);
            NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Translation/2-D").setDouble(AutoConstants.XY_CORRECTION_D);
            NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Rotation/0-P").setDouble(AutoConstants.ROTATION_CORRECTION_P);
            NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Rotation/1-I").setDouble(AutoConstants.ROTATION_CORRECTION_I);
            NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Rotation/2-D").setDouble(AutoConstants.ROTATION_CORRECTION_D);

            NetworkTableInstance.getDefault().getTable("Calibration").getEntry("HDC/Translation/0-P").setDouble(AutoConstants.XY_CORRECTION_P);
            NetworkTableInstance.getDefault().getTable("Calibration").getEntry("HDC/Translation/1-I").setDouble(AutoConstants.XY_CORRECTION_I);
            NetworkTableInstance.getDefault().getTable("Calibration").getEntry("HDC/Translation/2-D").setDouble(AutoConstants.XY_CORRECTION_D);
            NetworkTableInstance.getDefault().getTable("Calibration").getEntry("HDC/Rotation/0-P").setDouble(AutoConstants.ROTATION_CORRECTION_P);
            NetworkTableInstance.getDefault().getTable("Calibration").getEntry("HDC/Rotation/1-I").setDouble(AutoConstants.ROTATION_CORRECTION_I);
            NetworkTableInstance.getDefault().getTable("Calibration").getEntry("HDC/Rotation/2-D").setDouble(AutoConstants.ROTATION_CORRECTION_D);
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

            AutoConstants.XY_PID.setP(TELE_HDC_XY_P);
            AutoConstants.XY_PID.setI(TELE_HDC_XY_I);
            AutoConstants.XY_PID.setD(TELE_HDC_XY_D);
            AutoConstants.THETA_PID.setP(TELE_HDC_TH_P);
            AutoConstants.THETA_PID.setI(TELE_HDC_TH_I);
            AutoConstants.THETA_PID.setD(TELE_HDC_TH_D);

        }
    }

    public Command getAutonomousCommand() {
        return pathPlannerStorage.getSelectedAuto();
    }

    private void configureHDCBindings(PatriBoxController controller) {
        controller.pov(0, 270, testButtonBindingLoop)
            .onTrue(HDCTuner.controllerDecrementCommand());

        controller.pov(0, 90, testButtonBindingLoop)
            .onTrue(HDCTuner.controllerIncrementCommand());

        controller.pov(0, 0, testButtonBindingLoop)
            .onTrue(HDCTuner.increaseCurrentConstantCommand(.1));

        controller.pov(0, 180, testButtonBindingLoop)
            .onTrue(HDCTuner.increaseCurrentConstantCommand(-.1));

        controller.rightBumper(testButtonBindingLoop)
            .onTrue(HDCTuner.constantIncrementCommand());

        controller.leftBumper(testButtonBindingLoop)
            .onTrue(HDCTuner.constantDecrementCommand());

        controller.a(testButtonBindingLoop)
            .onTrue(HDCTuner.logCommand());

        controller.x(testButtonBindingLoop)
            .onTrue(HDCTuner.multiplyPIDCommand(2));

        controller.b(testButtonBindingLoop)
            .onTrue(HDCTuner.multiplyPIDCommand(.5));
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

    private void prepareNamedCommands() {}

}
