package frc.robot.commands.managers;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.AutoConstants;

public class HDCTuner extends SubsystemBase {

    @AutoLogOutput (key = "Managers//HDCTuner/PIDControllerIndex")
    public int PIDControllerIndex = 0;
    @AutoLogOutput (key = "Managers//HDCTuner/PIDIndex")
    public int PIDIndex = 0;

    public PIDController XYController = AutoConstants.HDC.getXController();

    public ProfiledPIDController thetaController = AutoConstants.HDC.getThetaController();

    public HDCTuner(PIDController XYController, ProfiledPIDController thetaController) {
        this.XYController = XYController;
        this.thetaController = thetaController;
    }

    public void updateValues() {
        AutoConstants.HDC = new HolonomicDriveController(
            XYController,
            XYController,
            thetaController
        );
    }

    public void log() {
        updateValues();
        System.out.printf("XY P: %.4f | I: %.4f | D: %.4f | Theta P: %.4f | I: %.4f | D: %.4f%n", 
            XYController.getP(), XYController.getI(), XYController.getD(), 
            thetaController.getP(), thetaController.getI(), thetaController.getD());
    }

    public void PIDIncrement() {
        if (PIDIndex < 3) {
            this.PIDIndex++;
            System.out.println("Currently editing: " + (PIDIndex == 0 ? "P" : PIDIndex == 1 ? "D" : "I"));
        }
    }

    public void PIDDecrement() {
        if (PIDIndex > 0) {
            this.PIDIndex--;
            System.out.println("Currently editing: " + (PIDIndex == 0 ? "P" : PIDIndex == 1 ? "D" : "I"));
        }
    }

    public void controllerIncrement() {
        if (PIDControllerIndex < 1) {
            this.PIDControllerIndex++;
            System.out.println("Currently editing: " + (PIDControllerIndex == 0 ? "XY" : "Theta"));
        }
    }

    public void controllerDecrement() {
        if (PIDControllerIndex > 0) {
            this.PIDControllerIndex--;
            System.out.println("Currently editing: " + (PIDControllerIndex == 0 ? "XY" : "Theta"));
        }
    }

    public void increasePID(double value) {
        if (PIDControllerIndex == 0) {
            switch (this.PIDIndex) {
                case 0:
                    XYController.setP(XYController.getP() + value);
                    break;
                case 1:
                    XYController.setD(XYController.getD() + value * 0.1);
                    break;
                case 2:
                    XYController.setI(XYController.getI() + value);
                    break;
            }
        } else {
            switch (this.PIDIndex) {
                case 0:
                    thetaController.setP(thetaController.getP() + value);
                    break;
                case 1:
                    thetaController.setD(thetaController.getD() + value);
                    break;
                case 2:
                    thetaController.setI(thetaController.getI() + value);
                    break;
            }
        }
        log();
    }

    public void multiplyPID(double value) {
        if (PIDControllerIndex == 0) {
            switch (this.PIDIndex) {
                case 0:
                    XYController.setP(XYController.getP() * value);
                    break;
                case 1:
                    XYController.setD(XYController.getD() * value);
                    break;
                case 2:
                    XYController.setI(XYController.getI() * value);
                    break;
            }
        } else {
            switch (this.PIDIndex) {
                case 0:
                    thetaController.setP(thetaController.getP() * value);
                    break;
                case 1:
                    thetaController.setD(thetaController.getD() * value);
                    break;
                case 2:
                    thetaController.setI(thetaController.getI() * value);
                    break;
            }
        }
        log();
    }

    public Command logCommand() {
        return Commands.runOnce(this::log);
    }
    
    public Command multiplyPIDCommand(double d) {
        return Commands.runOnce(() -> {
            this.multiplyPID(d);
        });
    }

    public Command constantIncrementCommand() {
        return Commands.runOnce(() -> {
            PIDIncrement();
        });
    }

    public Command constantDecrementCommand() {
        return Commands.runOnce(() -> {
            PIDDecrement();
        });
    }

    public Command controllerIncrementCommand() {
        return Commands.runOnce(() -> {
            controllerIncrement();
        });
    }

    public Command controllerDecrementCommand() {
        return Commands.runOnce(() -> {
            controllerDecrement();
        });
    }

    public Command increaseCurrentConstantCommand(double value) {
        return Commands.runOnce(() -> {
            this.increasePID(value);
        });
    }

    public Command decreaseCurrentConstantCommand(double value) {
        return Commands.runOnce(() -> {
            this.increasePID(-value);
        });
    }

}
