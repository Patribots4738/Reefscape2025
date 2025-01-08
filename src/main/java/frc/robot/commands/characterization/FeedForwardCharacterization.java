// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.characterization;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import frc.robot.util.calc.PolynomialRegression;

public class FeedForwardCharacterization extends Command {
  private static final double START_DELAY_SECS = 2.0;
  private static final double RAMP_PER_SEC = 0.1;

  private FeedForwardCharacterizationData data;
  private final DoubleConsumer inputConsumer;
  private final DoubleSupplier velocitySupplier;

  private final Timer timer = new Timer();

  /** Creates a new FeedForwardCharacterization command. */
  public FeedForwardCharacterization(
      Subsystem subsystem, DoubleConsumer inputConsumer, DoubleSupplier velocitySupplier) {
    addRequirements(subsystem);
    this.inputConsumer = inputConsumer;
    this.velocitySupplier = velocitySupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    data = new FeedForwardCharacterizationData();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() < START_DELAY_SECS) {
      inputConsumer.accept(0.0);
    } else {
      double input = (timer.get() - START_DELAY_SECS) * RAMP_PER_SEC;
      System.err.println(input);
      inputConsumer.accept(input);
      data.add(velocitySupplier.getAsDouble(), input);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    inputConsumer.accept(0.0);
    timer.stop();
    data.print();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public static class FeedForwardCharacterizationData {
    private final List<Double> velocityData = new LinkedList<>();
    private final List<Double> inputData = new LinkedList<>();

    public void add(double velocity, double input) {
      if (Math.abs(velocity) > 1E-4) {
        velocityData.add(Math.abs(velocity));
        inputData.add(Math.abs(input));
      }
    }

    public void print() {
      if (velocityData.size() == 0 || inputData.size() == 0) {
        return;
      }

      PolynomialRegression regression =
          new PolynomialRegression(
              velocityData.stream().mapToDouble(Double::doubleValue).toArray(),
              inputData.stream().mapToDouble(Double::doubleValue).toArray(),
              1);

      System.out.println("FF Characterization Results:");
      System.out.println("\tCount=" + Integer.toString(velocityData.size()) + "");
      System.out.println(String.format("\tR2=%.5f", regression.R2()));
      System.out.println(String.format("\tkS=%.5f", regression.beta(0)));
      System.out.println(String.format("\tkV=%.5f", regression.beta(1)));
    }
  }
}
