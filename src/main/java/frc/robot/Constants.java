// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class OperatorConstants {
    public static final int DRIVER = 0; // KEEP ZEROOOOOOOOO!!!!!!!!
    public static final int COPILOT = 1; // Keep One
    public static final double DEADBAND = 0.1;
  }

  public static final class SwerveDriveConstants {
    public static final double MAX_SPEED = 3.0; // Meters per second

  }
  
  public static final class SparkMaxIDs {
    public static final int LEFT_FLY_WHEEL = 9;
    public static final int RIGHT_FLY_WHEEL = 10;
    public static final int KICKER = 11;
    public static final int ROLLERS = 12;
    public static final int INTAKE_LIFT = 14;
    public static final int INTAKE_SPINNY = 15;
    public static final int LEFT_CLIMBER = 16;
    public static final int RIGHT_CLIMBER = 17;
  }

  public static final class ShooterConstants {
    public static final double SHOOTING_SPEED = 1;

    // Shooter PID
    public static final double SHOOTER_KP = 0;
    public static final double SHOOTER_KI = 0;
    public static final double SHOOTER_KD = 0;
    public static final double SHOOTER_KS = 0;
    public static final double SHOOTER_KG = 0;
    public static final double SHOOTER_KV = 0;
    public static final double SHOOTER_KA = 0;
    public static final double ARM_MAX_VELOCITY = 0;
    public static final double ARM_MAX_ACCELERATION = 0;
  }

  public static final class IntakeConstants {
    public static final double LIFT_SPEED = -0.4; // Negative to make the lift go in the correct direction
    public static final double SPIN_SPEED = -0.75; // Negative to make the wheels spin in the correct direction
  }

  public static final class ClimberConstants {
    public static final double CLIMBER_SPEED = -0.5; // Negative to make the climber go in the correct direction
  }

  public static final class KickerConstants {
    // Kicker Constants
    public static final double KICKER_SPEED = 1;
  }

  public static final class RollerConstants {
    public static final double ROLLER_SPEED = 0.5;
  }
}
