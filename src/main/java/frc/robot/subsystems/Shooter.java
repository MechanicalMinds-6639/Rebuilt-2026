// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SparkMaxIDs;

public class Shooter extends SubsystemBase {

  // These are the class members
  SparkMax leftFlyWheel = new SparkMax(SparkMaxIDs.LEFT_FLY_WHEEL, MotorType.kBrushless);
  SparkMax rightFlyWheel = new SparkMax(SparkMaxIDs.RIGHT_FLY_WHEEL, MotorType.kBrushless);

  /* 
  // PID Controllers
  private final ProfiledPIDController ShooterController = new ProfiledPIDController(
      ShooterConstants.SHOOTER_KP,
      ShooterConstants.SHOOTER_KI,
      ShooterConstants.SHOOTER_KD,
      new Constraints(ShooterConstants.ARM_MAX_VELOCITY,
          ShooterConstants.ARM_MAX_ACCELERATION));
  private final SimpleMotorFeedforward ShooterFeedforward = new SimpleMotorFeedforward(
      ShooterConstants.SHOOTER_KS,
      ShooterConstants.SHOOTER_KG,
      ShooterConstants.SHOOTER_KV,
      ShooterConstants.SHOOTER_KA);
  */

  /** Creates a new Shooter. */
  public Shooter() {
    SparkMaxConfig LeftFlyWheelConfig = new SparkMaxConfig();
    LeftFlyWheelConfig.smartCurrentLimit(40);
    leftFlyWheel.configure(LeftFlyWheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig RightFlyWheelConfig = new SparkMaxConfig();
    RightFlyWheelConfig.smartCurrentLimit(40);
    RightFlyWheelConfig.follow(leftFlyWheel, true); // Makes the right flywheel motor follow the left one, also inverts
                                                    // it
    rightFlyWheel.configure(RightFlyWheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // This method makes flywheels go weeee
  public void ShooterOn() {
    leftFlyWheel.set(ShooterConstants.SHOOTING_SPEED);
  }

  // This command makes flywheels go weeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee ;)
  public Command ShooterOnCommand() {
    return run(() -> {
      leftFlyWheel.set(ShooterConstants.SHOOTING_SPEED);
    });
  }

  // This method turns the flywheels to the shooter off
  public void ShooterOff() {
    leftFlyWheel.set(0);
  }

  // This command turns the flywheels to the shooter off
  public Command ShooterOffCommand() {
    return run(() -> {
      leftFlyWheel.set(0);
    });
  }

  public Command ShooterCommand(CommandXboxController driverController, CommandXboxController copilotController) {
    return run(() -> {

      if (driverController.getHID().getLeftBumper() || copilotController.getHID().getLeftBumper()) {
        ShooterOff();
      }

      if (driverController.getHID().getRightBumper() || copilotController.getHID().getRightBumper()) {
        ShooterOn();
      }

    });
  }

  /*
  public void reachGoal(double goalDegrees) {
    double goal = Degrees.of(goalDegrees).in(Rotations) * CraneConstants.ARM_REDUCTION;
    double clampedValue = MathUtil.clamp(
        ArmFeedforward.calculate(ArmController.getSetpoint().position, ArmController.getSetpoint().velocity)
            + ArmController.calculate(ArmEncoder.getPosition(), goal),
        -7, 7);
    ArmMax.setVoltage(clampedValue);
  }
  */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
