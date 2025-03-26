package frc.robot.commands.logging;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class PieceThrower extends Command {
    private final double duration;
    private final Supplier<Pose3d> startingPoseSupplier, endingPoseSupplier;
    private Pose3d startingPose;
    private final Timer timer = new Timer();
    private final Consumer<Pose3d> loggedPoseConsumer;

    public PieceThrower(
        Supplier<Pose3d> startingPoseSupplier, 
        Supplier<Pose3d> endingPoseSupplier, 
        Consumer<Pose3d> loggedPoseConsumer) 
    {
        this(startingPoseSupplier, endingPoseSupplier, loggedPoseConsumer, 1);
    }

    public PieceThrower(
        Supplier<Pose3d> startingPoseSupplier, 
        Supplier<Pose3d> endingPoseSupplier,  
        Consumer<Pose3d> loggedPoseConsumer, 
        double durationSupplier) 
    {
        this.startingPoseSupplier = startingPoseSupplier;
        this.endingPoseSupplier = endingPoseSupplier;
        this.duration = durationSupplier;
        this.loggedPoseConsumer = loggedPoseConsumer;
    }

    public static PieceThrower throwPiece(
        Supplier<Pose3d> startingPoseSupplier, 
        Supplier<Pose3d> endingPoseSupplier,  
        Consumer<Pose3d> loggedPoseConsumer)
    {
        return new PieceThrower(startingPoseSupplier, endingPoseSupplier, loggedPoseConsumer);
    }

    public static PieceThrower throwPiece(
        Supplier<Pose3d> startingPoseSupplier, 
        Supplier<Pose3d> endingPoseSupplier,  
        Consumer<Pose3d> loggedPoseConsumer, 
        double durationSupplier)
    {
        return new PieceThrower(startingPoseSupplier, endingPoseSupplier, loggedPoseConsumer, durationSupplier);
    }

    @Override
    public void initialize() {
        startingPose = startingPoseSupplier.get();
        timer.restart();
    }

    @Override
    public void execute() {
        double percent = 1 - ((duration - timer.get()) / duration);
        loggedPoseConsumer.accept(startingPose.interpolate(endingPoseSupplier.get(), percent));
    }

    @Override
    public void end(boolean interrupted) {
        loggedPoseConsumer.accept(endingPoseSupplier.get());
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.get() > duration;
    }
}
