package frc.robot.util.custom;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.function.BooleanSupplier;

/**
 * A command composition that runs one of two commands, depending on the value of the given
 * condition while this command executes. When the condition changes, the selected command will too.
 *
 * <p>The rules for command compositions apply: command instances that are passed to it cannot be
 * added to any other composition or scheduled individually, and the composition requires all
 * subsystems its components require.
 *
 * <p>This class was created by team 4738. It is not part of the official WPILib API.
 */
public class ActiveConditionalCommand extends Command {
    
    private final Command onTrueCommand;
    private final Command onFalseCommand;
    private final BooleanSupplier condition;
    private Command selectedCommand;    

    public ActiveConditionalCommand(Command onTrueCommand, Command onFalseCommand, BooleanSupplier condition) {
        this.onTrueCommand = requireNonNullParam(onTrueCommand, "onTrue", "ConditionalCommand");
        this.onFalseCommand = requireNonNullParam(onFalseCommand, "onFalse", "ConditionalCommand");
        this.condition = requireNonNullParam(condition, "condition", "ConditionalCommand");

        CommandScheduler.getInstance().registerComposedCommands(onTrueCommand, onFalseCommand);

        getRequirements().addAll(this.onTrueCommand.getRequirements());
        getRequirements().addAll(this.onFalseCommand.getRequirements());
    }

    @Override
    public void initialize() {
        if (condition.getAsBoolean()) {
            selectedCommand = onTrueCommand;
        } else {
            selectedCommand = onFalseCommand;
        }
        selectedCommand.initialize();
    }

    @Override
    public void execute() {
        if (condition.getAsBoolean() ^ (selectedCommand == onTrueCommand))
        {
            selectedCommand.end(true);
            this.initialize();
        }
        selectedCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        selectedCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return selectedCommand.isFinished();
    }

    @Override
    public boolean runsWhenDisabled() {
        return onTrueCommand.runsWhenDisabled() && onFalseCommand.runsWhenDisabled();
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        if (onTrueCommand.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf
                || onFalseCommand.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf) {
            return InterruptionBehavior.kCancelSelf;
        } else {
            return InterruptionBehavior.kCancelIncoming;
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addStringProperty("onTrue", onTrueCommand::getName, null);
        builder.addStringProperty("onFalse", onFalseCommand::getName, null);
        builder.addStringProperty(
                "selected",
                () -> {
                    if (selectedCommand == null) {
                        return "null";
                    } else {
                        return selectedCommand.getName();
                    }
                },
                null);
    }
}
