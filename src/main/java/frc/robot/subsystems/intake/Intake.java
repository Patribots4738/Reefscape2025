// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.IntakeConstants;
import frc.robot.util.hardware.phoenix.Kraken;

public class Intake extends SubsystemBase {

  private Kraken intake;  
  private boolean hasPiece;
  /** Creates a new Intake. */
  public Intake() {
    intake = new Kraken(IntakeConstants.INTAKE_CAN_ID, "superStructure");

  } 
  
//sets the motors to spin forwards and thus intake the coral/algea  

  /**
   * 
   * @return
   */
  public Command intake() {
    return Commands.run(() -> intake.setPercentOutput(0.6))
                    .until(() -> hasPiece())
                    .andThen(() -> Commands.run(() -> intake.setPercentOutput(0.4)));
    
  }

//sets the motors to spin backwards and thus outtake the coral/alg

  public Command outtake() {
    return Commands.run(() -> intake.setTargetVelocity(-1));

  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run 
    hasPiece = hasPiece();
  }
      
  public boolean hasPiece() {
    // TODO: add logic for holding a piece
    return true;
  }

  public boolean getHasPiece() {
    return hasPiece;
  }
}
