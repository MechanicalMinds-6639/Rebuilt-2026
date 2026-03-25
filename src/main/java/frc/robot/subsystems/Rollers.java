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

public class Rollers extends SubsystemBase {

  // These are the class members
  SparkMax rollersMax = new SparkMax(SparkMaxIDs.ROLLERS, MotorType.kBrushless);

  boolean rollersRunning = false;

  /** Creates a new Rollers. */
  public Rollers() {
    SparkMaxConfig RollersConfig = new SparkMaxConfig();
    RollersConfig.smartCurrentLimit(40);
    rollersMax.configure(RollersConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // This method runs the rollers
  public void rollersOn() {
    rollersMax.set(RollerConstants.ROLLER_SPEED);
  }

  // This command runs the rollers
  public Command rollersOnCommand() {
    return run(() -> {
      rollersMax.set(RollerConstants.ROLLER_SPEED);
    });
  }

  // This method runs the rollers in reverse
  public void rollersReverse() {
    rollersMax.set(-RollerConstants.ROLLER_SPEED);
  }

  // This command runs the rollers in reverse
  public Command rollersReverseCommand() {
    return run(() -> {
      rollersMax.set(-RollerConstants.ROLLER_SPEED);
    });
  }

  // This method runs the rollers forwards then backwards repeatedly to keep fuel moving
  public void jiggleRollers() {

    boolean jiggleForward = true;
    boolean jiggleBackward = false;

    while (rollersRunning) {
      
      if (jiggleForward) {
        rollersMax.set(RollerConstants.ROLLER_SPEED);
        jiggleForward = false;
        jiggleBackward = true;
      }

      if (jiggleBackward) {
        rollersMax.set(-RollerConstants.ROLLER_SPEED);
        jiggleBackward = false;
        jiggleForward = true;
      }

    }

  }

  // This command runs the rollers forwards then backwards repeatedly to keep fuel moving
  public Command jiggleRollersCommand() {
    return run(() -> {
      
      boolean jiggleForward = true;
      boolean jiggleBackward = false;

      while (rollersRunning) {

        if (jiggleForward) {
          rollersMax.set(RollerConstants.ROLLER_SPEED);
          jiggleForward = false;
          jiggleBackward = true;
        }

        if (jiggleBackward) {
          rollersMax.set(-RollerConstants.ROLLER_SPEED);
          jiggleBackward = false;
          jiggleForward = true;
        }

      }
    
    });
  }

  // This method stops the rollers
  public void rollersOff() {
    rollersMax.set(0);
  }

  // This command stops the rollers
  public Command rollersOffCommand() {
    return run(() -> {
      rollersMax.set(0);
    });
  }


  public Command rollersCommand(CommandXboxController driverController, CommandXboxController copilotController) {
    return run(() -> {

      if (driverController.leftBumper().getAsBoolean() || copilotController.leftBumper().getAsBoolean()) {
        rollersOn();
      }

      if (driverController.rightBumper().getAsBoolean() || copilotController.rightBumper().getAsBoolean()) {
        rollersOff();
      }

      if (copilotController.povRight().getAsBoolean()) {
        rollersReverse();
      }
      
      while (copilotController.povRight().getAsBoolean()) {
        rollersRunning = true;
        jiggleRollers();
      } 
      rollersRunning = false; // Runs when while loop breaks

    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
