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
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.SparkMaxIDs;

public class Climber extends SubsystemBase {

  /* 
  // These are the class members
  SparkMax leftClimberMax = new SparkMax(SparkMaxIDs.LEFT_CLIMBER, MotorType.kBrushless);
  SparkMax rightClimberMax = new SparkMax(SparkMaxIDs.RIGHT_CLIMBER, MotorType.kBrushless);
  */

  /** Creates a new Climber. */
  public Climber() {
    /*
    SparkMaxConfig LeftClimberConfig = new SparkMaxConfig();
    LeftClimberConfig.smartCurrentLimit(40);
    leftClimberMax.configure(LeftClimberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    SparkMaxConfig RightClimberConfig = new SparkMaxConfig();
    RightClimberConfig.smartCurrentLimit(40);
    RightClimberConfig.follow(leftClimberMax, true); // Makes the right climber motor follow the left one, also inverts it
    rightClimberMax.configure(RightClimberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    */
  }
    /*
  // This method makes the climber go up
  public void ClimberUp() {
    leftClimberMax.set(ClimberConstants.CLIMBER_SPEED);
  }

  // This command makes the climber go up
  public Command ClimberUpCommand() {
    return run(() -> {
      leftClimberMax.set(ClimberConstants.CLIMBER_SPEED);
    });
  }

  // This method makes the climber go down
  public void ClimberDown() {
    leftClimberMax.set(-ClimberConstants.CLIMBER_SPEED);
  }

  // The command makes the climber go down
  public Command ClimberDownCommand() {
    return run(() -> {
      leftClimberMax.set(-ClimberConstants.CLIMBER_SPEED);
    });
  }

  // This method stops the climber
  public void ClimberStop() {
    leftClimberMax.set(0);
  }

  // The command makes the climber go up
  public Command ClimberStopCommand() {
    return run(() -> {
      leftClimberMax.set(0);
    });
  }
  */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
