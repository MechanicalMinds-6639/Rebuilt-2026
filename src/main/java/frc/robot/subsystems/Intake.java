// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.SparkMaxIDs;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;

public class Intake extends SubsystemBase {

  private double SetPointHeight = 0.0;
  private boolean liftRunning = false;

  // These are the class members
  SparkMax intakeLift = new SparkMax(SparkMaxIDs.INTAKE_LIFT, MotorType.kBrushless);
  SparkMax intakeSpinny = new SparkMax(SparkMaxIDs.INTAKE_SPINNY, MotorType.kBrushless);

  /** Creates a new Intake. */
  public Intake() {
    SparkMaxConfig IntakeLiftConfig = new SparkMaxConfig();
    IntakeLiftConfig.smartCurrentLimit(40);
    intakeLift.configure(IntakeLiftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig IntakeSpinnyConfig = new SparkMaxConfig();
    IntakeSpinnyConfig.smartCurrentLimit(40);
    intakeSpinny.configure(IntakeSpinnyConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // This method lowers the intake lift
  public void IntakeLiftDown() {
    intakeLift.set(IntakeConstants.LIFT_SPEED);
  }

  // This command lowers the intake lift
  public Command IntakeLiftDownCommand() {
    return run(() -> {
      intakeLift.set(IntakeConstants.LIFT_SPEED);
    });
  }

  // This method raises the intake lift
  public void IntakeLiftUp() {
    intakeLift.set(-IntakeConstants.LIFT_SPEED);
  }

  // This command raises the intake lift
  public Command IntakeLiftUpCommand() {
    return run(() -> {
      intakeLift.set(-IntakeConstants.LIFT_SPEED);
    });
  }

  // This method makes the intake go spinny
  public void IntakeSpinny() {
    intakeSpinny.set(IntakeConstants.SPIN_SPEED);
  }

  // This command makes the intake go spinnyyyyyyyyyyyyyyyyyyyyyyyyyyyyy :)
  public Command IntakeSpinnyCommand() {
    return run(() -> {
      intakeSpinny.set(IntakeConstants.SPIN_SPEED);
    });
  }

  // This method makes the intake go reverse spinny
  public void IntakeReverseSpinny() {
    intakeSpinny.set(-IntakeConstants.SPIN_SPEED);
  }

  // This command makes the intake go reverse spinny
  public Command IntakeReverseCommand() {
    return run(() -> {
      intakeSpinny.set(-IntakeConstants.SPIN_SPEED);
    });
  }

  // This method makes the intake stop going spinny
  public void IntakeStopSpinny() {
    intakeSpinny.set(0);
  }

  // This command makes the intake stop going spinny
  public Command IntakeStopSpinnyCommand() {
    return run(() -> {
      intakeSpinny.set(0);
    });
  }

  // Command that allows joystick control of the intake lift and runs intake in reverse when the intake moves upward
  public Command IntakeCommand(CommandXboxController copilotController) {
    return run(() -> {

      if (Math.abs(copilotController.getLeftY()) > OperatorConstants.DEADBAND) {
        intakeLift.set(-copilotController.getLeftY() * IntakeConstants.LIFT_SPEED);
        liftRunning = true;
      } 
      else if (copilotController.b().getAsBoolean()) {
        IntakeStopSpinny();
      }
      else if (copilotController.x().getAsBoolean()) {
        IntakeSpinny();
      }
      else if (copilotController.y().getAsBoolean()) {
        IntakeReverseSpinny();
      }
      else {
        intakeLift.set(0);
        liftRunning = false;
      }
      
      /*
      // If statment that runs the intake spinny wheels in reverse when the intake lift moves upwards
      if ((liftRunning == true) && (Math.abs(copilotController.getLeftY()) > OperatorConstants.DEADBAND) && copilotController.getLeftY() > 0) {
        IntakeReverseSpinny();
      }
      else {
        intakeSpinny.set(0);
      }
      */
      
    
    });
  }


  // Command that allows joystick control of the intake lift and runs intake in reverse when the intake moves upward
  public Command OneControllerIntakeCommand(CommandXboxController driverController) {
    return run(() -> {

      if (driverController.leftTrigger().getAsBoolean()) {
        IntakeLiftUp();
        IntakeReverseSpinny(); // May cause problems, comment out if needed
      } 
      else if (driverController.rightTrigger().getAsBoolean()) {
        IntakeLiftDown();
      }
      else if (driverController.a().getAsBoolean()) {
        IntakeSpinny();
      }
      else if (driverController.b().getAsBoolean()) {
        IntakeStopSpinny();
      }
      else if (driverController.y().getAsBoolean()) {
        IntakeReverseSpinny();
      }
      else {
        intakeLift.set(0);
        intakeSpinny.set(0);
      }
    
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
