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
import frc.robot.Constants.SparkMaxIDs;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

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

  // This command makes the intake go down
  public Command IntakeDown() {
    return run(() -> {
      intakeLift.set(IntakeConstants.LIFT_SPEED);
    });
  }

  // This command makes the intake go up
  public Command IntakeUp() {
    return run(() -> {
      intakeLift.set(-IntakeConstants.LIFT_SPEED);
    });
  }

  // This command makes the intake go spinny
  public Command IntakeSpinny() {
    return run(() -> {
      intakeSpinny.set(IntakeConstants.SPIN_SPEED);
    });
  }

  // This command makes the intake go reverse spinny
  public Command IntakeReverseSpinny() {
    return run(() -> {
      intakeSpinny.set(-IntakeConstants.SPIN_SPEED);
    });
  }

  // This command makes the intake stop going spinny
  public Command IntakeStop() {
    return run(() -> {
    intakeSpinny.set(0);
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
