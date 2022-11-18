/****************************** Header ******************************\
 Class Name: RobotContainer
 File Name: RobotContainer.java
 Summary: Contains robot button bindings with commands, and autonomous commands
 Project: BroncBotzFRC2023
 Copyright (c) BroncBotz.
 All rights reserved.
 Author(s): Dylan Watson
 \********************************************************************/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.autonomous.TrajectorySampleCommand;
import frc.robot.commands.drivetrain.DrivetrainCommand;
import frc.robot.commands.drivetrain.StopDrivetrainCommand;
import frc.robot.commands.index.ForwardIndexCommand;
import frc.robot.commands.index.ReverseIndexCommand;
import frc.robot.commands.index.StopIndexCommand;
import frc.robot.commands.intake.LowerAndSuckIntakeCommand;
import frc.robot.commands.intake.MagicIntakeCommand;
import frc.robot.commands.intake.RaiseIntakeCommand;
import frc.robot.commands.shooter.PrepareShooterCommand;
import frc.robot.commands.shooter.ShootHighCommand;
import frc.robot.commands.shooter.ShootLowCommand;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.index.IndexSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionPolicy;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer
{

  // The robot's subsystems and commands are defined here...
  private final ClimberSubsystem    climberSubsystem    = new ClimberSubsystem();
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final IndexSubsystem      indexSubsystem      = new IndexSubsystem();
  private final IntakeSubsystem     intakeSubsystem     = new IntakeSubsystem();
  private final ShooterSubsystem    shooterSubsystem    = new ShooterSubsystem();
  private final XboxController      driver              = new XboxController(Constants.DriverControllerPort), // driver
      operator                                          = new XboxController(Constants.OperatorControllerPort);
  private final ShuffleboardTab intake = Shuffleboard.getTab("Intake");

  private final ShuffleboardTab commands = Shuffleboard.getTab("Commands");

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Set default commands
    setDefaultCommands();
    // Configure the button bindings
    configureButtonBindings();
    // Setup Shuffleboard Data
    setupShuffleboard();
    // Read shuffleboard data
  }

  /**
   * Setup Shuffleboard Data.
   */
  private void setupShuffleboard()
  {
    // Let the driver know if there is a ball found.
    intake.addBoolean("Ball Found", VisionPolicy::ballFound);

    commands.add(new ShootHighCommand(shooterSubsystem, indexSubsystem));
    commands.add(new ShootLowCommand(shooterSubsystem, indexSubsystem));
    commands.add(new PrepareShooterCommand(shooterSubsystem));
    commands.add(new StopIndexCommand(indexSubsystem));
    commands.add(new InstantCommand(drivetrainSubsystem::clearStickyFaults, drivetrainSubsystem));

  }

  /**
   * This method set's the default commands for subsystems defining their neutral state at any given time when not being
   * used.
   */
  public void setDefaultCommands()
  {
    // Intake is by default off and up.
    intakeSubsystem.setDefaultCommand(new RaiseIntakeCommand(intakeSubsystem));

    // Index is by default off
    indexSubsystem.setDefaultCommand(new StopIndexCommand(indexSubsystem));

    // Shooter starts spinning when there is a ball on the pressure pad, and does not run otherwise.
    shooterSubsystem.setDefaultCommand(new PrepareShooterCommand(shooterSubsystem));

    // Drive train should not be moving by default.
    drivetrainSubsystem.setDefaultCommand(new StopDrivetrainCommand(drivetrainSubsystem));

  }

  /**
   * Set the Default Command for TeleopMode, required the cancel the old default commands before reassignment.
   */
  public void setTeleopDefaultCommands()
  {
    // Cancel the previous default command that way we can override it.
    drivetrainSubsystem.getDefaultCommand().cancel();
    // The drive train defaults to responding to the controller.
    drivetrainSubsystem.setDefaultCommand(
        new DrivetrainCommand(drivetrainSubsystem, driver::getLeftY, driver::getRightY));
  }


  /**
   * Use this method to define your button->command mappings. Buttons can be created by instantiating a
   * {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and
   * then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings()
  {
    // When the A button on the driver controller is pressed it magically intakes balls as we see them.
    new Trigger(driver::getAButton).whileActiveContinuous(
        new ParallelCommandGroup(new MagicIntakeCommand(intakeSubsystem), new ReverseIndexCommand(indexSubsystem)));

    // When the X button is pressed we lower the intake and suck in balls
    new Trigger(driver::getXButton).whileActiveContinuous(
        new ParallelCommandGroup(new LowerAndSuckIntakeCommand(intakeSubsystem),
                                 new ReverseIndexCommand(indexSubsystem)));

    // When the left bumper is pressed on the driver controller it will lower a ball in the index.
    new Trigger(driver::getLeftBumper).whileActiveContinuous(new ReverseIndexCommand(indexSubsystem));

    // When the right bumper is pressed on the driver controller it wil raise the ball in the intake
    new Trigger(driver::getRightBumper).whileActiveContinuous(new ForwardIndexCommand(indexSubsystem));

    // When the B button on the driver controller is pressed shoot to the low goal.
    new Trigger(driver::getBButton).whileActiveContinuous(new ShootLowCommand(shooterSubsystem, indexSubsystem));

    // When the Y button on th e driver controller is pressed shoot to high goal.
    new Trigger(driver::getYButton).whileActiveContinuous(new ShootHighCommand(shooterSubsystem, indexSubsystem));

    // TODO: Mihir's incredibly specific climbing scenario.
    // 2 types of climbs (High % Low).
    // Button B raises it to the lower climber position.
    // Button Y will be mid climb up.
    // Button X would be mid climb down.
    // The overdrive button will be moved to the back of the controller by pressing down both back buttons
    // simultaneously with both hands and then driving normally.

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {

    // An ExampleCommand will run in autonomous
//    return new DriveForwardCommand(drivetrainSubsystem);
    return new TrajectorySampleCommand(drivetrainSubsystem);
  }
}
