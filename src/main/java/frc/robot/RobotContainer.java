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
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Roller;
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
  private final Kicker kicker = new Kicker();
  private final Roller roller = new Roller();
  private final Limelight limelight = new Limelight();

  // Creates the Xbox Controllers
  private final CommandXboxController driverController = new CommandXboxController(Constants.OperatorConstants.DRIVER);
  private final CommandXboxController copilotController = new CommandXboxController(Constants.OperatorConstants.COPILOT);

  // Code from YAGSL for the SwerveSubsytem
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(driveBase.getSwerveDrive(),
                                          () -> driverController.getLeftY(), // Multiply by -1 to fix reverse gyroscope/control directions
                                          () -> driverController.getLeftX()) // Multiply by -1 to fix reverse gyroscope/control directions
                                          .withControllerRotationAxis(driverController::getRightX)
                                          .deadband(OperatorConstants.DEADBAND)
                                          .scaleTranslation(1) // If want to go faster, increase number
                                          .allianceRelativeControl(true);

  // Code from YAGSL for the SwerveSubsytem
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
                                      .withControllerHeadingAxis(driverController::getRightX,
                                      driverController::getRightY)
                                      .headingWhile(true);

  // For Limelight Autoalign
  SwerveInputStream driveAutoAlign = SwerveInputStream.of(driveBase.getSwerveDrive(),
                                          () -> driverController.getLeftY(), // Multiply by -1 to fix reverse gyroscope/control directions
                                          () -> driverController.getLeftX()) // Multiply by -1 to fix reverse gyroscope/control directions
                                          .withControllerRotationAxis(limelight::getRotationRate)
                                          .deadband(OperatorConstants.DEADBAND)
                                          .scaleTranslation(1) // If want to go faster, increase number
                                          .allianceRelativeControl(true);

  // For Limelight Autoalign during Autonomous
  SwerveInputStream autonomousDriveAutoAlign = SwerveInputStream.of(driveBase.getSwerveDrive(),
                                          () -> driverController.getLeftY(), // Multiply by -1 to fix reverse gyroscope/control directions
                                          () -> driverController.getLeftX()) // Multiply by -1 to fix reverse gyroscope/control directions
                                          .withControllerRotationAxis(limelight::getAutonomousRotationRate)
                                          .deadband(OperatorConstants.DEADBAND)
                                          .scaleTranslation(1) // If want to go faster, increase number
                                          .allianceRelativeControl(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    DriverStation.silenceJoystickConnectionWarning(true);

    // AUTO COMMANDS
    NamedCommands.registerCommand("Shoot First 8", (shooter.shooterOnCommand()).withTimeout(4));
    NamedCommands.registerCommand("Kicker For Shooting", (kicker.kickerOnCommand()).withTimeout(4));
    NamedCommands.registerCommand("Roller For Shooting", (roller.rollerOnCommand()).withTimeout(4));
    //
    NamedCommands.registerCommand("Reverse Kicker", (kicker.kickerReverseCommand()).withTimeout(.25));
    //
    NamedCommands.registerCommand("Shoot First 8 Forever", shooter.shooterOnCommand());
    NamedCommands.registerCommand("Kicker Forever", kicker.kickerOnCommand());
    NamedCommands.registerCommand("Roller Forever", roller.rollerOnCommand());
    //
    NamedCommands.registerCommand("Intake Fuel", intake.intakeLiftDownCommand().withTimeout(1).andThen(intake.intakeSpinnyCommand())
      .withTimeout(4));
    //
    NamedCommands.registerCommand("Auto Align", (driveBase.driveFieldOriented(autonomousDriveAutoAlign))
      .withTimeout(1));

    // Builds an auto chooser
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {

    // Code from YAGSL for the SwerveSubsytem
    @SuppressWarnings("unused") // To make the warning go away
    Command driveFieldOrientedDirectAngle = driveBase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAngularVelocity = driveBase.driveFieldOriented(driveAngularVelocity);
    
    // For Limelight Autoalign
    Command driveFieldOrientedAutoAlign = driveBase.driveFieldOriented(driveAutoAlign);

    // Default swerve command from YAGSL
    driveBase.setDefaultCommand(driveFieldOrientedAngularVelocity); // Change to switch the drive control style, make sure to set heading correction to true in SwerveSubsystem

    /*
    //// One Controller Set-Up
    driverController.a().onTrue(Commands.runOnce(driveBase::zeroGyro)); // Zeros the gyro
    //
    driverController.leftBumper().onTrue(kicker.kickerOffCommand()); // Turns the kicker off
    driverController.rightBumper().onTrue(roller.rollerReverseCommand().withTimeout(.5).andThen(kicker.kickerOnCommand())); // Turns the kicker on after running it in reverse for .5 seconds to clear it
    //
    driverController.rightTrigger().whileTrue(driveFieldOrientedAutoAlign).whileFalse(driveFieldOrientedAngularVelocity);
    //
    shooter.setDefaultCommand(shooter.shooterCommand(driverController, copilotController)); // Controls the shooter
    // kicker.setDefaultCommand(kicker.kickerCommand(driverController, copilotController)); // Controls the kicker
    // roller.setDefaultCommand(roller.rollerCommand(driverController, copilotController)); // Controls the roller
    intake.setDefaultCommand(intake.oneControllerIntakeCommand(driverController)); // Controls the intake lift motion and the intake spinny
    //
    driverController.povUp().onTrue(roller.rollerOnCommand()).onFalse(roller.rollerCommand(driverController, copilotController)); // Turns the roller on
    driverController.povLeft().whileTrue(driveBase.centerModulesCommand()); // Zeros the wheels
    driverController.povRight().whileTrue(roller.jiggleRollerCommand()).whileFalse(roller.rollerCommand(driverController, copilotController)); // Jiggles the roller back and forth
    ////
    */
    
    //// Two Controller Set-Up
    driverController.a().whileTrue(driveFieldOrientedAutoAlign).whileFalse(driveFieldOrientedAngularVelocity);
    driverController.b().whileTrue(driveBase.centerModulesCommand()); // Zeros the wheels
    driverController.x().whileTrue(driveBase.antiPushWheels()); // Puts the wheels in an X pattern to "lock" them
    driverController.y().onTrue(Commands.runOnce(driveBase::zeroGyro)); // Zeros the gyro
    // System.out.println("brad");
    driverController.leftBumper().onTrue(kicker.kickerOffCommand()); // Turns the kicker off
    driverController.rightBumper().onTrue(kicker.kickerReverseCommand().withTimeout(.25).andThen(kicker.kickerOnCommand())); // Turns the kicker on after running it in reverse for .5 seconds to clear it
    driverController.rightBumper().whileTrue(driveBase.antiPushWheels()); // Puts the wheels in an X pattern to "lock" them while shooting
    //
    shooter.setDefaultCommand(shooter.shooterCommand(driverController, copilotController)); // Controls the shooter
    // kicker.setDefaultCommand(kicker.kickerCommand(driverController, copilotController)); // Controls the kicker
    roller.setDefaultCommand(roller.rollerCommand(driverController, copilotController)); // Controls the roller
    ///
    copilotController.a().whileTrue(driveFieldOrientedAutoAlign).whileFalse(driveFieldOrientedAngularVelocity);
    //
    copilotController.leftBumper().onTrue(kicker.kickerOffCommand()); // Turns the kicker off
    copilotController.rightBumper().onTrue(kicker.kickerReverseCommand().withTimeout(.25).andThen(kicker.kickerOnCommand())); // Turns the kicker on after running it in reverse for .5 seconds to clear it
    copilotController.rightBumper().whileTrue(driveBase.antiPushWheels()); // Puts the wheels in an X pattern to "lock" them while shooting
    //
    intake.setDefaultCommand(intake.intakeCommand(copilotController)); // Controls the intake lift motion and the intake spinny
    //
    copilotController.povUp().onTrue(roller.rollerOnCommand()).onFalse(roller.rollerCommand(driverController, copilotController)); // Turns the roller on
    copilotController.povLeft().onTrue(driveBase.antiPushWheels()); // Puts the wheels in an X pattern and "locks" them
    copilotController.povRight().whileTrue(roller.jiggleRollerCommand()).onFalse(roller.rollerOffCommand().andThen(roller.rollerCommand(driverController, copilotController))); // Jiggles the roller back and forth 
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

  // Resets the heading of the gyroscope
  public void resetGyro() {
    driveBase.zeroGyro();
  }
}
