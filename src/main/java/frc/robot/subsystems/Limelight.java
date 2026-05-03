// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;

public class Limelight extends SubsystemBase {

  // These are the class members
  private int tv;
  private double tx;
  private double rotationRate;
  private boolean hubApriltagVisible;
  private int tid;
  private double autonomousRotationRate;

  /** Creates a new Limelight. */
  public Limelight() {}

  // This method returns "tv", or if a target is visible
  public double get_tv() {
    return tv;
  }

  // This method returns "tx", or the degrees off center from the apriltag
  public double get_tx() {
    return tx;
  }

  // This method returns "roationRate", or the degrees off center depending on if the center apriltag is visible or not
  public double getRotationRate() {
    return rotationRate;
  }

  // This method returns "autonomousRotationRate", or the degrees off center depending on if corner apriltag is visible or not
  public double getAutonomousRotationRate() {
    return autonomousRotationRate;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Creates the table of variables from the NetworkTable for the Limelight
    var table = NetworkTableInstance.getDefault().getTable(LimelightConstants.LIME_LIGHT_NAME);

    // Gets the values needed from the table
    tv = (int)table.getEntry("tv").getDouble(0); // Is target visible? 1 for yes, 0 for no
    tx = table.getEntry("tx").getDouble(0); // Degrees off center
    tid = (int)table.getEntry("tid").getDouble(0); // 

    // If hub apriltag visible, set the swerve adjustment variable(rotationRate) to the degrees off from apriltag and update the boolean
    if (tv == 1 && (tid == 10 || tid == 25)) {
      rotationRate = tx * LimelightConstants.AIM_KP; // Calculation: Degrees off center * Smoothness
      hubApriltagVisible = true;
    }
    // If target not visible, set the swerve adjustment variable(rotationRate) to 0 so the robot doesn't turn
    else {
      rotationRate = 0;
      hubApriltagVisible = false;
    }

    // If corner hub apriltag visible, do the same thing as the first if statment. *For Autonomous*
    if (tv == 1 && (tid == 10 || tid == 25)) {
      autonomousRotationRate = tx * LimelightConstants.AIM_KP; // Calculation: Degrees off center * Smoothness
    }

    SmartDashboard.putNumber("tv", tv);
    SmartDashboard.putNumber("tx", tx);
    SmartDashboard.putBoolean("HUB APRILTAG VISIBLE", hubApriltagVisible);
  }
}
