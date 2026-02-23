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

  // These are the class members
  SparkMax leftClimber = new SparkMax(SparkMaxIDs.LEFT_CLIMBER, MotorType.kBrushless);
  SparkMax rightClimber = new SparkMax(SparkMaxIDs.RIGHT_CLIMBER, MotorType.kBrushless);

  /** Creates a new Climber. */
  public Climber() {
    SparkMaxConfig LeftClimberConfig = new SparkMaxConfig();
    LeftClimberConfig.smartCurrentLimit(40);
    leftClimber.configure(LeftClimberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    SparkMaxConfig RightClimberConfig = new SparkMaxConfig();
    RightClimberConfig.smartCurrentLimit(40);
    RightClimberConfig.follow(leftClimber, true); // Makes the right climber motor follow the left one, also inverts it
    rightClimber.configure(RightClimberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
 
  }

  //The command makes the climber go up
  public Command ClimberUp() {
    return run(() -> {
      leftClimber.set(ClimberConstants.CLIMBER_SPEED);
    });
  }

  //This command makes the climber go down
  public Command ClimberDown() {
    return run(() -> {
      leftClimber.set(-ClimberConstants.CLIMBER_SPEED);
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

