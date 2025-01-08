package frc.robot.util.custom;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.Robot.GameMode;
import frc.robot.util.Constants.OIConstants;

public class PatriBoxController extends CommandXboxController {

    private double deadband;

    public PatriBoxController(int port, double deadband) {
        super(port);

        this.deadband = deadband;
    }

    // The following 24 methods will be unnecesary in 2025.
    // They fix a loop overrun issue in 2024's wpilib
    // more info can be found at https://github.com/Patribots4738/Crescendo2024/issues/144
    public boolean getAButton() {
        return super.getHID().getAButton();
    }

    public boolean getBButton() {
        return super.getHID().getBButton();
    }

    public boolean getXButton() {
        return super.getHID().getXButton();
    }

    public boolean getYButton() {
        return super.getHID().getYButton();
    }

    public double getLeftTriggerAxis() {
        return super.getHID().getLeftTriggerAxis();
    }

    public boolean getLeftTrigger() {
        return super.getHID().getLeftTriggerAxis() > 0.25;
    }

    public double getRightTriggerAxis() {
        return super.getHID().getRightTriggerAxis();
    }

    public boolean getRightTrigger() {
        return super.getHID().getRightTriggerAxis() > 0.25;
    }

    public boolean getStartButton() {
        return super.getHID().getStartButton();
    }

    public boolean getBackButton() {
        return super.getHID().getBackButton();
    }

    public boolean getPOVUp() {
        return super.getHID().getPOV() == 0;
    }

    public boolean getPOVRight() {
        return super.getHID().getPOV() == 90;
    }

    public boolean getPOVDown() {
        return super.getHID().getPOV() == 180;
    }

    public boolean getPOVLeft() {
        return super.getHID().getPOV() == 270;
    }

    public boolean getLeftBumper() {
        return super.getHID().getLeftBumperButton();
    }

    public boolean getRightBumper() {
        return super.getHID().getRightBumperButton();
    }

    public Trigger leftY() {
        return leftY(0.3, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger leftX() {
        return leftX(0.3, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger rightY() {
        return rightY(0.3, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger rightX() {
        return rightX(0.3, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger leftY(double threshold, EventLoop loop) {
        return new Trigger(loop, () -> Math.abs(getLeftY()) > threshold);
    }

    public Trigger leftX(double threshold, EventLoop loop) {
        return new Trigger(loop, () -> Math.abs(getLeftX()) > threshold);
    }

    public Trigger rightY(double threshold, EventLoop loop) {
        return new Trigger(loop, () -> Math.abs(getRightY()) > threshold);
    }

    public Trigger rightX(double threshold, EventLoop loop) {
        return new Trigger(loop, () -> Math.abs(getRightX()) > threshold);
    }
    // This marks the end of the methods that should be unnecesary in 2025

    @Override
    public double getLeftX() {
        return getLeftAxis().getX();
    }

    @Override
    
    public double getLeftY() {
        return getLeftAxis().getY();
    }

    @Override
    public double getRightX() {
        return getRightAxis().getX();
    }

    @Override
    public double getRightY() {
        return getRightAxis().getY();
    }

    public Translation2d getLeftAxis() {
        // The Y inverted because for some reason when you
        // go forward on the controller, it returns a negative value.
        // I don't like that.
        return toCircle(MathUtil.applyDeadband(super.getLeftX(), deadband),
                MathUtil.applyDeadband(-super.getLeftY(), deadband));
    }

    public Translation2d getRightAxis() {
        // The Y inverted because for some reason when you
        // go forward on the controller, it returns a negative value.
        // I don't like that.
        return toCircle(MathUtil.applyDeadband(super.getRightX(), deadband),
                MathUtil.applyDeadband(-super.getRightY(), deadband));
    }

    // All calculations can be referenced here
    // https://www.desmos.com/calculator/e07raajzh5
    private Translation2d toCircle(double x, double y) {

        Translation2d intercept;
        Translation2d output;

        double slope = y / x;

        if (slope == 0 || Double.isNaN(slope) || Double.isInfinite(slope)) {

            output = new Translation2d(x, y);
            return output;

        } else if (0 > slope && slope >= -OIConstants.CONTROLLER_CORNER_SLOPE_2) {

            intercept = new Translation2d(-1, -slope);

        } else if (-OIConstants.CONTROLLER_CORNER_SLOPE_1 < slope && slope < -OIConstants.CONTROLLER_CORNER_SLOPE_2) {

            double intersectionX = (1.7) / (slope - 1);
            double intersectionY = (intersectionX + 1.7);

            intercept = new Translation2d(intersectionX, intersectionY);

        } else if (slope < -OIConstants.CONTROLLER_CORNER_SLOPE_1 || slope > OIConstants.CONTROLLER_CORNER_SLOPE_1) {

            intercept = new Translation2d(1 / slope, 1);

        } else if (OIConstants.CONTROLLER_CORNER_SLOPE_1 > slope && slope > OIConstants.CONTROLLER_CORNER_SLOPE_2) {

            double intersectionX = (1.7) / (slope + 1);
            double intersectionY = -intersectionX + 1.7;

            intercept = new Translation2d(intersectionX, intersectionY);

        } else if (0 < slope && slope <= OIConstants.CONTROLLER_CORNER_SLOPE_2) {

            intercept = new Translation2d(1, slope);

        } else {

            intercept = new Translation2d(0, 0);

            System.out.println("Error... Slope = " + slope);

        }

        double distance = getDistance(x, y, intercept.getX(), intercept.getY());

        distance = (distance > 1) ? 1 : distance;

        output = new Translation2d(Math.signum(x) * distance, new Rotation2d(Math.atan(y / x)));
        return output;
    }

    private double getDistance(double x1, double y1, double x2, double y2) {
        return Math.sqrt(
                (Math.pow(x1, 2) + Math.pow(y1, 2)) /
                        (Math.pow(x2, 2) + Math.pow(y2, 2)));
    }

    public Command setRumble(DoubleSupplier rumble) {
        return Commands.runOnce(() -> setRumble(rumble.getAsDouble()));
    }

    public Command setRumble(DoubleSupplier rumble, double duration) {
        return Commands.runOnce(() -> setRumble(rumble.getAsDouble()))
            .andThen(Commands.waitSeconds(duration))
            .andThen(Commands.runOnce(() -> setRumble(0)));
    }

    public void setRumble(double rumble) {
        if (Robot.gameMode == GameMode.TELEOP)
            this.getHID().setRumble(RumbleType.kBothRumble, rumble);
        else if (Robot.gameMode == GameMode.DISABLED)
            this.getHID().setRumble(RumbleType.kBothRumble, 0);
    }
}
