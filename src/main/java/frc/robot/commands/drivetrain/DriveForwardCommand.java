package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

/**
 * Simple Drive Forward command based off time. Drives at 20% power for 1 second forward.
 */
public class DriveForwardCommand extends CommandBase
{

  private final DrivetrainSubsystem drivetrainSubsystem;
  private final Timer               timer = new Timer();

  /**
   * Constructor for DriveForwardCommand given the drive train subsystem.
   *
   * @param drivetrainSubsystem Initialized drive train susbsytem.
   */
  public DriveForwardCommand(DrivetrainSubsystem drivetrainSubsystem)
  {
    this.drivetrainSubsystem = drivetrainSubsystem;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.drivetrainSubsystem);
  }

  /**
   * The initial subroutine of a command.  Called once when the command is initially scheduled. Stops the drive train.
   */
  @Override
  public void initialize()
  {
    drivetrainSubsystem.run(0, 0);
    timer.reset();
    timer.start();
  }

  /**
   * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
   * until {@link #isFinished()}) returns true.) Runs the drive train at 20% power.
   */
  @Override
  public void execute()
  {
    drivetrainSubsystem.run(0.2, 0.2);
  }

  /**
   * <p>
   * Returns whether this command has finished. Once a command finishes -- indicated by this method returning true --
   * the scheduler will call its {@link #end(boolean)} method.
   * </p><p>
   * Returning false will result in the command never ending automatically. It may still be cancelled manually or
   * interrupted by another command. Hard coding this command to always return true will result in the command executing
   * once and finishing immediately. It is recommended to use *
   * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand} for such an operation.
   * </p>
   *
   * @return whether this command has finished. True after 1 second.
   */
  @Override
  public boolean isFinished()
  {
    return timer.hasElapsed(1);
  }

  /**
   * The action to take when the command ends. Called when either the command finishes normally -- that is it is called
   * when {@link #isFinished()} returns true -- or when  it is interrupted/canceled. This is where you may want to wrap
   * up loose ends, like shutting off a motor that was being used in the command.
   * Stops the drive train.
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted)
  {
    drivetrainSubsystem.run(0, 0);
    timer.stop();
  }
}
