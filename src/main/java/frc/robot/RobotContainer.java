// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem driveBase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final Shooter shooter = new Shooter();
  private final Intake intake = new Intake();
  private final Climber climber = new Climber();


  // Creates the Xbox Controller(s)
  private final CommandXboxController driverController = new CommandXboxController(Constants.OperatorConstants.DRIVER);

  // Code from YAGSL for the SwerveSubsytem
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(driveBase.getSwerveDrive(),
                                                                () -> driverController.getLeftY() * -1,
                                                                () -> driverController.getLeftX() * -1)
                                                                .withControllerRotationAxis(driverController::getRightX)
                                                                .deadband(OperatorConstants.DEADBAND)
                                                                .scaleTranslation(0.8) // If want to go faster, increase number
                                                                .allianceRelativeControl(true);     

  // Code from YAGSL for the SwerveSubsytem
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverController::getRightX, 
                                                                                             driverController::getRightY)
                                                                                             .headingWhile(true);

  // Code from YAGSL for the SwerveSubsytem
  Command driveFieldOrientedDirectAngle = driveBase.driveFieldOriented(driveDirectAngle);

  // Code from YAGSL for the SwerveSubsytem
  Command driveFieldOrientedAngularVelocity = driveBase.driveFieldOriented(driveAngularVelocity);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    DriverStation.silenceJoystickConnectionWarning(true);
    
    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {

    //Code from YAGSL for the SwerveSubsytem
    Command driveFieldOrientedDirectAngle = driveBase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAngularVelocity = driveBase.driveFieldOriented(driveAngularVelocity);

    driveBase.setDefaultCommand(driveFieldOrientedAngularVelocity); //Change to switch the drive control style, make sure to set heading correction to true in SwerveSubsystem

    driverController.a().whileTrue(driveBase.centerModulesCommand()); //Zeros the wheels
    driverController.y().onTrue(Commands.runOnce(driveBase::zeroGyro)); //Zeros the gyro
    //System.out.println("brad");
    driverController.x().onTrue(shooter.ShooterOn());
    driverController.rightTrigger().whileTrue(intake.IntakeDown());
    driverController.leftTrigger().whileTrue(intake.IntakeUp());
    driverController.povRight().onTrue(intake.IntakeSpinny());
    driverController.povDown().onTrue(intake.IntakeStop());
    driverController.povLeft().onTrue(intake.IntakeReverseSpinny());
    driverController.rightBumper().whileTrue(climber.ClimberUp());
    driverController.leftBumper().whileTrue(climber.ClimberDown());
  }

  private final SendableChooser<Command> autoChooser;

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
      return autoChooser.getSelected();
  }

  public void resetGyro() {
    driveBase.zeroGyro();
  } 
}