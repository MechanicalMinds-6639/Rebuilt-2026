// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Rollers;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem driveBase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final Shooter shooter = new Shooter();
  private final Intake intake = new Intake();
  private final Climber climber = new Climber();
  private final Kicker kicker = new Kicker();
  private final Rollers rollers = new Rollers();

  // Creates the Xbox Controllers
  private final CommandXboxController driverController = new CommandXboxController(Constants.OperatorConstants.DRIVER);
  private final CommandXboxController copilotController = new CommandXboxController(Constants.OperatorConstants.COPILOT);

  // Code from YAGSL for the SwerveSubsytem
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(driveBase.getSwerveDrive(),
                                          () -> driverController.getLeftY() * -1, // Multiply by -1 to fix reverse gyroscope/control directions
                                          () -> driverController.getLeftX() * -1) // Multiply by -1 to fix reverse gyroscope/control directions
                                          .withControllerRotationAxis(driverController::getRightX)
                                          .deadband(OperatorConstants.DEADBAND)
                                          .scaleTranslation(0.8) // If want to go faster, increase number
                                          .allianceRelativeControl(true);

  // Code from YAGSL for the SwerveSubsytem
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
                                      .withControllerHeadingAxis(driverController::getRightX,
                                      driverController::getRightY)
                                      .headingWhile(true);

  // Code from YAGSL for the SwerveSubsytem
  Command driveFieldOrientedDirectAngle = driveBase.driveFieldOriented(driveDirectAngle);

  // Code from YAGSL for the SwerveSubsytem
  Command driveFieldOrientedAngularVelocity = driveBase.driveFieldOriented(driveAngularVelocity);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    DriverStation.silenceJoystickConnectionWarning(true);

    // AUTO COMMANDS
    NamedCommands.registerCommand("Shoot First 8", (shooter.shooterOnCommand().andThen(kicker.kickerOnCommand()))
      .withTimeout(4));
    NamedCommands.registerCommand("Shoot First 8 For Longer", shooter.shooterOnCommand());
    NamedCommands.registerCommand("Kicker", kicker.kickerOnCommand());
    NamedCommands.registerCommand("Intake Fuel", intake.intakeLiftDownCommand().withTimeout(1).andThen(intake.intakeSpinnyCommand())
      .withTimeout(3));

    // Builds an auto chooser
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {

    // Code from YAGSL for the SwerveSubsytem
    Command driveFieldOrientedDirectAngle = driveBase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAngularVelocity = driveBase.driveFieldOriented(driveAngularVelocity);

    driveBase.setDefaultCommand(driveFieldOrientedAngularVelocity); // Change to switch the drive control style, make sure to set heading correction to true in SwerveSubsystem

    /*
    //// One Controller Set-Up
    driverController.a().onTrue(Commands.runOnce(driveBase::zeroGyro)); // Zeros the gyro
    //
    shooter.setDefaultCommand(shooter.shooterCommand(driverController, copilotController)); // Controls the shooter
    kicker.setDefaultCommand(kicker.kickerCommand(driverController, copilotController)); // Controls the kicker
    rollers.setDefaultCommand(rollers.rollersCommand(driverController, copilotController)); // Controls the rollers
    intake.setDefaultCommand(intake.OneControllerIntakeCommand(driverController)); // Controls the intake lift motion and the intake spinny
    //
    driverController.povUp().whileTrue(climber.ClimberDownCommand()).onFalse(climber.ClimberStopCommand()); // Runs the climber down when held
    driverController.povRight().whileTrue(driveBase.centerModulesCommand()); // Zeros the wheels
    driverController.povDown().whileTrue(climber.ClimberUpCommand()).onFalse(climber.ClimberStopCommand()); // Runs the climber up when held
    ////
    */


    //// Two Controller Set-Up
    driverController.a().whileTrue(driveBase.centerModulesCommand()); // Zeros the wheels
    // driverController.x().whileTrue(climber.ClimberUpCommand()).onFalse(climber.ClimberStopCommand()); // Runs the climber up when held
    // driverController.b().whileTrue(climber.ClimberDownCommand()).onFalse(climber.ClimberStopCommand()); // Runs the climber down when held
    driverController.y().onTrue(Commands.runOnce(driveBase::zeroGyro)); // Zeros the gyro
    // System.out.println("brad");
    shooter.setDefaultCommand(shooter.shooterCommand(driverController, copilotController)); // Controls the shooter
    kicker.setDefaultCommand(kicker.kickerCommand(driverController, copilotController)); // Controls the kicker
    rollers.setDefaultCommand(rollers.rollersCommand(driverController, copilotController)); // Controls the rollers
    ///
    intake.setDefaultCommand(intake.intakeCommand(copilotController)); // Controls the intake lift motion and the intake spinny 
    ////
    
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
