// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.IntakeConstants;
import frc.robot.util.hardware.phoenix.CANCoderCustom;
import frc.robot.util.hardware.phoenix.Kraken;
import frc.robot.util.Constants;

public class Pivot extends SubsystemBase {

  private Kraken pivotMotor;
  private CANCoderCustom turnEncoder;
  /** Creates a new Pivot. */
  public Pivot() {
    pivotMotor = new Kraken(IntakeConstants.PIVOT_CAN_ID, "superStructure");
    configMotors();
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void configMotors(){
    turnEncoder = new CANCoderCustom(canCoderId, "Drivebase");
    
  }

  public Command 
  public Command stow() {
    return Commands.runOnce(() -> pivotMotor.setTargetPosition(IntakeConstants.STOW_RADIANS)  );
  }

  public Command haandoff()
  
  public Comman
}
