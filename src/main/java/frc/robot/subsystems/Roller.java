// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RollerConstants;
import frc.robot.Constants.SparkMaxIDs;

public class Roller extends SubsystemBase {

  // These are the class members
  SparkMax rollerMax = new SparkMax(SparkMaxIDs.ROLLERS, MotorType.kBrushless);

  boolean rollerRunning = false;

  /** Creates a new Roller. */
  public Roller() {
    SparkMaxConfig RollerConfig = new SparkMaxConfig();
    RollerConfig.smartCurrentLimit(40);
    rollerMax.configure(RollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
 
  // This method runs the roller
  public void rollerOn() {
    rollerMax.set(RollerConstants.ROLLER_SPEED);
  }

  // This command runs the roller
  public Command rollerOnCommand() {
    return run(() -> {
      rollerMax.set(RollerConstants.ROLLER_SPEED);
    });
  }

  // This method runs the roller in reverse
  public void rollerReverse() {
    rollerMax.set(-RollerConstants.ROLLER_SPEED);
  }

  // This command runs the roller in reverse
  public Command rollerReverseCommand() {
    return run(() -> {
      rollerMax.set(-RollerConstants.ROLLER_SPEED);
    });
  }

  // This method runs the roller forwards then backwards repeatedly to keep fuel moving
  public void jiggleRoller() {

    boolean jiggleForward = true;
    boolean jiggleBackward = false;

    while (rollerRunning) {
      
      if (jiggleForward) {
        rollerMax.set(RollerConstants.ROLLER_SPEED);
        jiggleForward = false;
        jiggleBackward = true;
      }

      if (jiggleBackward) {
        rollerMax.set(-RollerConstants.ROLLER_SPEED);
        jiggleBackward = false;
        jiggleForward = true;
      }

    }

  }

  // This command runs the roller forwards then backwards repeatedly to keep fuel moving
  public Command jiggleRollerCommand() {
    return run(() -> {
      
      boolean jiggleForward = true;
      boolean jiggleBackward = false;

      while (rollerRunning) {

        if (jiggleForward) {
          rollerMax.set(RollerConstants.ROLLER_SPEED);
          jiggleForward = false;
          jiggleBackward = true;
        }

        if (jiggleBackward) {
          rollerMax.set(-RollerConstants.ROLLER_SPEED);
          jiggleBackward = false;
          jiggleForward = true;
        }

      }
    
    });
  }

  // This method stops the roller
  public void rollersOff() {
    rollerMax.set(0);
  }

  // This command stops the roller
  public Command rollerOffCommand() {
    return run(() -> {
      rollerMax.set(0);
    });
  }


  public Command rollerCommand(CommandXboxController driverController, CommandXboxController copilotController) {
    return run(() -> {

      if (driverController.leftBumper().getAsBoolean() || copilotController.leftBumper().getAsBoolean()) {
        rollersOff();
      }

      if (driverController.rightBumper().getAsBoolean() || copilotController.rightBumper().getAsBoolean()) {
        rollerOn();
      }

      if (copilotController.povUp().getAsBoolean()) {
        rollerReverse();
      }
      
      while (copilotController.povRight().getAsBoolean()) {
        rollerRunning = true;
        jiggleRoller();
      } 
      rollerRunning = false; // Runs when while loop breaks

    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
