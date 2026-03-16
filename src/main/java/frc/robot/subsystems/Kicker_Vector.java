// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.SparkMaxIDs;
import frc.robot.Constants.Kicker_Vector_Constants;
import frc.robot.Constants.ShooterConstants;

public class Kicker_Vector extends SubsystemBase {

  // These are the class members

  // Kicker
  SparkMax kicker = new SparkMax(SparkMaxIDs.KICKER, MotorType.kBrushless);

  // Vex Wheels
  SparkMax leftVectorWheels = new SparkMax(SparkMaxIDs.LEFT_VECTOR_WHEELS, MotorType.kBrushless);
  SparkMax rightVectorWheels = new SparkMax(SparkMaxIDs.RIGHT_VECTOR_WHEELS, MotorType.kBrushless);

  /** Creates a new Kicker. */
  public Kicker_Vector() {

    // Kicker
    SparkMaxConfig KickerConfig = new SparkMaxConfig();
    KickerConfig.smartCurrentLimit(40);
    kicker.configure(KickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Vex Wheels
    SparkMaxConfig LeftVectorWheelsConfig = new SparkMaxConfig();
    LeftVectorWheelsConfig.smartCurrentLimit(40);
    leftVectorWheels.configure(LeftVectorWheelsConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig RightVectorWheelsConfig = new SparkMaxConfig();
    RightVectorWheelsConfig.smartCurrentLimit(40);
    RightVectorWheelsConfig.follow(leftVectorWheels, false); // Makes the motor for the right vector wheels follow the left one
    rightVectorWheels.configure(RightVectorWheelsConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  // This method runs the kicker
  public void KickerOn() {
    kicker.set(Kicker_Vector_Constants.KICKER_SPEED);
  }

  // This command runs the kicker
  public Command KickerOnCommand() {
    return run(() -> {
      kicker.set(Kicker_Vector_Constants.KICKER_SPEED);
    });
  }

  // This method runs the kicker in reverse
  public void KickerReverse() {
    kicker.set(-Kicker_Vector_Constants.KICKER_SPEED);
  }

  // This command runs the kicker in reverse
  public Command KickerReverseCommand() {
    return run(() -> {
      kicker.set(-Kicker_Vector_Constants.KICKER_SPEED);
    });
  }

  // This method turns the kicker off
  public void KickerOff() {
    kicker.set(0);
  }

  // This command turns the kicker off
  public Command KickerOffCommand() {
    return run(() -> {
      kicker.set(0);
    });
  }

  // This method turns the vex wheels on
  public void VexWheelsOn() {
    leftVectorWheels.set(Kicker_Vector_Constants.VECTOR_WHEELS_SPEED);
  }

  // This command turns the vex wheels on
  public Command VexWheelsOnCommand() {
    return run(() -> {
      leftVectorWheels.set(Kicker_Vector_Constants.VECTOR_WHEELS_SPEED);
    });
  }

  // This method turns the vex wheels off
  public void VexWheelsOff() {
    leftVectorWheels.set(0);
  }

  // This command turns the vex wheels off
  public Command VexWheelsOffCommand() {
    return run(() -> {
      leftVectorWheels.set(0);
    });
  }

  public Command HopperCommand(CommandXboxController driverController, CommandXboxController copilotController) {
    return run(() -> {

      if (driverController.getHID().getLeftBumper() || copilotController.getHID().getLeftBumper()) {
        KickerOff();
        VexWheelsOff();
      }
      
      if (driverController.getHID().getRightBumper() || copilotController.getHID().getRightBumper()) {
        KickerOn();
        VexWheelsOn();
      }

    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}