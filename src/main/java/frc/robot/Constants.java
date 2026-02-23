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
  public static class OperatorConstants {
    public static final int DRIVER = 1;
    public static final double DEADBAND = 0.1;
  }

  public static final double MAX_SPEED = Units.feetToMeters(4.5);

   public final class SparkMaxIDs {
    public static final int LEFT_FLY_WHEEL = 0;
    public static final int RIGHT_FLY_WHEEL = 0;
    public static final int INTAKE_LIFT = 0;
    public static final int INTAKE_SPINNY = 0;
    public static final int LEFT_CLIMBER = 0;
    public static final int RIGHT_CLIMBER = 0;
  }

  public final class ShooterConstants {
    public static final double SHOOTING_SPEED = 0.5;
  }

  public final class IntakeConstants {
    public static final double LIFT_SPEED = 0;
    public static final double SPIN_SPEED = 0;
  }

  public final class ClimberConstants {
    public static final double CLIMBER_SPEED = 0;
  }
}