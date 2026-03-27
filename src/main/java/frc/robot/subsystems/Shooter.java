// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
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
  SparkMax leftFlyWheelMax = new SparkMax(SparkMaxIDs.LEFT_FLY_WHEEL, MotorType.kBrushless);
  SparkMax rightFlyWheelMax = new SparkMax(SparkMaxIDs.RIGHT_FLY_WHEEL, MotorType.kBrushless);

  RelativeEncoder leftFlyWheelEncoder = leftFlyWheelMax.getEncoder();
  RelativeEncoder rightFlyWheelEncoder = rightFlyWheelMax.getEncoder();

  double leftFlyWheelRPM = leftFlyWheelEncoder.getVelocity();
  double rightFlyWheelRPM = rightFlyWheelEncoder.getVelocity();

  // PID Controllers
  private final ProfiledPIDController shooterController = new ProfiledPIDController(
      ShooterConstants.SHOOTER_KP,
      ShooterConstants.SHOOTER_KI,
      ShooterConstants.SHOOTER_KD,
      new Constraints(ShooterConstants.ARM_MAX_VELOCITY,
          ShooterConstants.ARM_MAX_ACCELERATION));

  private final SimpleMotorFeedforward shooterFeedforward = new SimpleMotorFeedforward(
      ShooterConstants.SHOOTER_KS,
      ShooterConstants.SHOOTER_KG,
      ShooterConstants.SHOOTER_KV,
      ShooterConstants.SHOOTER_KA);

  /** Creates a new Shooter. */
  public Shooter() {
    SparkMaxConfig LeftFlyWheelConfig = new SparkMaxConfig();
    LeftFlyWheelConfig.smartCurrentLimit(40);
    leftFlyWheelMax.configure(LeftFlyWheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig RightFlyWheelConfig = new SparkMaxConfig();
    RightFlyWheelConfig.smartCurrentLimit(40);
    rightFlyWheelMax.configure(RightFlyWheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    LeftFlyWheelConfig.inverted(false);
    RightFlyWheelConfig.inverted(true); // Inverts the right flywheel 
  }

  // This method makes flywheels go weeee
  public void shooterOn() {
    leftFlyWheelMax.set(ShooterConstants.SHOOTING_SPEED);
    rightFlyWheelMax.set(ShooterConstants.SHOOTING_SPEED);
  }

  // This command makes flywheels go weeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee ;)
  public Command shooterOnCommand() {
    return run(() -> {
      leftFlyWheelMax.set(ShooterConstants.SHOOTING_SPEED);
      rightFlyWheelMax.set(ShooterConstants.SHOOTING_SPEED);
    });
  }

  // This method turns the flywheels to the shooter off
  public void shooterOff() {
    leftFlyWheelMax.set(0);
    rightFlyWheelMax.set(0);

  }

  // This command turns the flywheels to the shooter off
  public Command shooterOffCommand() {
    return run(() -> {
      leftFlyWheelMax.set(0);
      rightFlyWheelMax.set(0);
    });
  }

  public Command shooterCommand(CommandXboxController driverController, CommandXboxController copilotController) {
    return run(() -> {

      if (driverController.leftBumper().getAsBoolean() || copilotController.leftBumper().getAsBoolean()) {
        shooterOff();
      }

      if (driverController.rightBumper().getAsBoolean() || copilotController.rightBumper().getAsBoolean()) {
        shooterOn();
      }

    });
  }

  public void reachShooterSpeed(double percent) {
    double targetRPM = percent * ShooterConstants.MAX_RPM; // Convers percentage to target RPM
    double avgRPM = (leftFlyWheelRPM + rightFlyWheelRPM) / 2.0; // Average RPM of both flywheels
    double output = MathUtil.clamp(shooterController.calculate(avgRPM, targetRPM)
        + shooterFeedforward.calculate(targetRPM), -12, 12); // PID + Feedforward

    leftFlyWheelMax.setVoltage(output); // setVoltage() accounts for battery sag
    rightFlyWheelMax.setVoltage(output);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
