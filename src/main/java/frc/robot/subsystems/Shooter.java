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

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SparkMaxIDs;

public class Shooter extends SubsystemBase {

  // These are the class members
  SparkMax leftFlywheelMax = new SparkMax(SparkMaxIDs.LEFT_FLY_WHEEL, MotorType.kBrushless);
  SparkMax rightFlywheelMax = new SparkMax(SparkMaxIDs.RIGHT_FLY_WHEEL, MotorType.kBrushless);

  RelativeEncoder leftFlywheelEncoder = leftFlywheelMax.getEncoder();
  RelativeEncoder rightFlywheelEncoder = rightFlywheelMax.getEncoder();

  double leftFlywheelRPM = 0;
  double rightFlywheelRPM = 0;
  double leftFlywheelPosition = 0;
  double rightFlywheelPosition = 0;
  double avgFlywheelPosition = 0;

  /* 
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
  */

  /** Creates a new Shooter. */
  public Shooter() {
    SparkMaxConfig LeftFlywheelConfig = new SparkMaxConfig();
    LeftFlywheelConfig.smartCurrentLimit(40);
    leftFlywheelMax.configure(LeftFlywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig RightFlywheelConfig = new SparkMaxConfig();
    RightFlywheelConfig.smartCurrentLimit(40);
    RightFlywheelConfig.follow(leftFlywheelMax, true); // Makes the right flywheel motor follow the left one, also inverts it
    rightFlywheelMax.configure(RightFlywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // This method makes flywheels go weeee
  public void shooterOn() {
    leftFlywheelMax.set(ShooterConstants.SHOOTER_ON_SPEED);
  }

  // This command makes flywheels go weeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee ;)
  public Command shooterOnCommand() {
    return run(() -> {
      leftFlywheelMax.set(ShooterConstants.SHOOTER_ON_SPEED);
    });
  }

  // This method puts the flywheels to the shooter in to rest speed
  public void shooterRest() {
    leftFlywheelMax.set(ShooterConstants.SHOOTER_REST_SPEED);
  }

  // This command puts the flywheels to the shooter in to rest speed
  public Command shooterRestCommand() {
    return run(() -> {
      leftFlywheelMax.set(ShooterConstants.SHOOTER_REST_SPEED);
    });
  }

  // This method turns the flywheels to the shooter off
  public void shooterOff() {
    leftFlywheelMax.set(0);
  }

  // This command turns the flywheels to the shooter off
  public Command shooterOffCommand() {
    return run(() -> {
      leftFlywheelMax.set(0);
    });
  }


  public Command shooterCommand(CommandXboxController driverController, CommandXboxController copilotController) {
    return run(() -> {

      if (driverController.leftBumper().getAsBoolean() || copilotController.leftBumper().getAsBoolean()) {
        shooterRest();
      }

      if (driverController.rightBumper().getAsBoolean() || copilotController.rightBumper().getAsBoolean()) {
        shooterOn();
      }

      if (driverController.povUp().getAsBoolean() || copilotController.povUp().getAsBoolean()) {
        shooterOff();
      }

    });
  }

  /* 
  public void reachShooterSpeed(double percent) {
    double targetRPM = percent * ShooterConstants.MAX_RPM; // Convers percentage to target RPM
    double avgRPM = (leftFlywheelRPM + rightFlywheelRPM) / 2.0; // Average RPM of both flywheels
    double output = MathUtil.clamp(shooterController.calculate(avgRPM, targetRPM)
        + shooterFeedforward.calculate(targetRPM), -12, 12); // PID + Feedforward

    leftFlywheelMax.setVoltage(output); // setVoltage() accounts for battery sag
    rightFlywheelMax.setVoltage(output);
  }
  */

  private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
    new SysIdRoutine.Config(),
    new SysIdRoutine.Mechanism(
        (voltage) -> {
            leftFlywheelMax.setVoltage(voltage.in(Volts));
            rightFlywheelMax.setVoltage(voltage.in(Volts));
        },
        log -> {
            log.motor("shooter")
                .voltage(Volts.of(leftFlywheelMax.getAppliedOutput() * 12))
                .angularVelocity(RotationsPerSecond.of(leftFlywheelEncoder.getVelocity()));
        },
        this
    )
  );

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    double leftFlywheelRPM = leftFlywheelEncoder.getVelocity();
    double rightFlywheelRPM = rightFlywheelEncoder.getVelocity();
    leftFlywheelPosition = leftFlywheelEncoder.getPosition();
    rightFlywheelPosition = rightFlywheelEncoder.getPosition();
    avgFlywheelPosition = (leftFlywheelPosition + rightFlywheelPosition) / 2;
    SmartDashboard.putNumber("leftFlywheelPostion", leftFlywheelPosition);
    SmartDashboard.putNumber("rightFlywheelPostion", rightFlywheelPosition);
    SmartDashboard.putNumber("avgFlywheelPostion", avgFlywheelPosition);
  }
}
