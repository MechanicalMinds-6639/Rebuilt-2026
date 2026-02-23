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
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SparkMaxIDs;

public class Shooter extends SubsystemBase {

  // These are the class members
  SparkMax leftFlyWheel = new SparkMax(SparkMaxIDs.LEFT_FLY_WHEEL, MotorType.kBrushless);
  SparkMax rightFlyWheel = new SparkMax(SparkMaxIDs.RIGHT_FLY_WHEEL, MotorType.kBrushless);

  /** Creates a new Shooter. */
  public Shooter() {
    SparkMaxConfig LeftFlyWheelConfig = new SparkMaxConfig();
    LeftFlyWheelConfig.smartCurrentLimit(40);
    leftFlyWheel.configure(LeftFlyWheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    SparkMaxConfig RightFlyWheelConfig = new SparkMaxConfig();
    RightFlyWheelConfig.smartCurrentLimit(40);
    RightFlyWheelConfig.follow(leftFlyWheel, true); // Makes the right flywheel motor follow the left one, also inverts it
    rightFlyWheel.configure(RightFlyWheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // This method makes flywheels go weeee
  public Command ShooterOn() {
    return run(() -> {
      leftFlyWheel.set(ShooterConstants.SHOOTING_SPEED);
    });
  }

  // This method turns the flywheels to the shooter off
  public Command ShooterOff() {
    return run(() -> {
      leftFlyWheel.set(0);
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
