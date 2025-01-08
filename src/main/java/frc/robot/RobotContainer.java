package frc.robot;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLogOutput;

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
import frc.robot.util.Constants.AutoConstants;
import frc.robot.util.Constants.OIConstants;
import frc.robot.util.auto.PathPlannerStorage;
import frc.robot.util.custom.PatriBoxController;

public class RobotContainer {

    private PowerDistribution pdh;

    private EventLoop testButtonBindingLoop = new EventLoop();

    private final PatriBoxController driver;
    @SuppressWarnings("unused")
    private final PatriBoxController operator;

    private boolean fieldRelativeToggle = true;
    private final BooleanSupplier robotRelativeSupplier;

    private final Swerve swerve;

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
    
    public RobotContainer() {

        System.out.println("Constructing Robot Container...");

        driver = new PatriBoxController(OIConstants.DRIVER_CONTROLLER_PORT, OIConstants.DRIVER_DEADBAND);
        operator = new PatriBoxController(OIConstants.OPERATOR_CONTROLLER_PORT, OIConstants.OPERATOR_DEADBAND);

        pdh = new PowerDistribution(30, ModuleType.kRev);
        pdh.setSwitchableChannel(false);

        swerve = new Swerve();

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
    }

    private void configureOperatorBindings(PatriBoxController controller) {

        // controller.povUp()
        //     .whileTrue(krakenTest.setPosition(() -> krakenTest.getPosition() + 0.2));

        // controller.povDown()
        //     .whileTrue(krakenTest.setPosition(() -> krakenTest.getPosition() - 0.2));

        // controller.a()
        //     .onTrue(krakenTest.setPosition(() -> 0));

        // controller.b()
        //     .whileTrue(krakenTest.setVelocity(() -> 500)
        //         .finallyDo(() -> krakenTest.setVelocity(() -> 0)));

        // controller.x()
        //     .whileTrue(krakenTest.setVelocity(() -> -500)
        //         .finallyDo(() -> krakenTest.setVelocity(() -> 0)));

        // controller.rightTrigger()
        //     .whileTrue(krakenTest.setPercent(controller::getRightY)
        //         .finallyDo(() -> krakenTest.setPercent(() -> 0)));

    }

    public void updateNTGains() {
        double PPHDCP = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Translation/0-P").getDouble(-1);
        double PPHDCI = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Translation/1-I").getDouble(-1);
        double PPHDCD = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Translation/2-D").getDouble(-1);
        double PPHDCP2 = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Rotation/0-P").getDouble(-1);
        double PPHDCI2 = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Rotation/1-I").getDouble(-1);
        double PPHDCD2 = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("Auto/Rotation/2-D").getDouble(-1);

        double HDCP = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("HDC/Translation/0-P").getDouble(-1);
        double HDCI = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("HDC/Translation/1-I").getDouble(-1);
        double HDCD = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("HDC/Translation/2-D").getDouble(-1);
        double HDCP2 = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("HDC/Rotation/0-P").getDouble(-1);
        double HDCI2 = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("HDC/Rotation/1-I").getDouble(-1);
        double HDCD2 = NetworkTableInstance.getDefault().getTable("Calibration").getEntry("HDC/Rotation/2-D").getDouble(-1);

        if (PPHDCP == -1 || PPHDCI == -1 || PPHDCD == -1 || PPHDCP2 == -1 || PPHDCI2 == -1 || PPHDCD2 == -1 ||
            HDCP == -1 || HDCI == -1 || HDCD == -1 || HDCP2 == -1 || HDCI2 == -1 || HDCD2 == -1) {
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
                    PPHDCP,
                    PPHDCI,
                    PPHDCD),
                new PIDConstants(
                    PPHDCP2,
                    PPHDCI2,
                    PPHDCD2));

            AutoConstants.XY_PID.setP(HDCP);
            AutoConstants.XY_PID.setI(HDCI);
            AutoConstants.XY_PID.setD(HDCD);
            AutoConstants.THETA_PID.setP(HDCP2);
            AutoConstants.THETA_PID.setI(HDCI2);
            AutoConstants.THETA_PID.setD(HDCD2);

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
