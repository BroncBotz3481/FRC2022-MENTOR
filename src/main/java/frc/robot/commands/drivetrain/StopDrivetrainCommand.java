package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveTrain;
import frc.robot.subsystems.drivetrain.DrivetrainPolicy;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

/**
 * Stop the DriveTrain should be the default action when in autonomous mode.
 */
public class StopDrivetrainCommand extends CommandBase
{

  /**
   * Drive train subsystem.
   */
  private final DrivetrainSubsystem drivetrainSubsystem;

  /**
   * Command requires complete control of the drive train subsystem.
   *
   * @param drivetrainSubsystem Initialized drive train subsystem.
   */
  public StopDrivetrainCommand(DrivetrainSubsystem drivetrainSubsystem)
  {
    this.drivetrainSubsystem = drivetrainSubsystem;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.drivetrainSubsystem);
  }

  /**
   * The initial subroutine of a command.  Called once when the command is initially scheduled.
   */
  @Override
  public void initialize()
  {
    DrivetrainPolicy.forwardsPowerScale = DriveTrain.forwardsPowerScale;
    DrivetrainPolicy.backwardsScale = DriveTrain.backwardsPowerScale;
  }

  /**
   * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
   * until {@link #isFinished()}) returns true.)
   * Stops the drive train.
   */
  @Override
  public void execute()
  {
    drivetrainSubsystem.run(0, 0);
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
   * Will not exist, must be interrupted.
   * @return whether this command has finished.
   */
  @Override
  public boolean isFinished()
  {
    return false;
  }

  /**
   * The action to take when the command ends. Called when either the command finishes normally -- that is it is called
   * when {@link #isFinished()} returns true -- or when  it is interrupted/canceled. This is where you may want to wrap
   * up loose ends, like shutting off a motor that was being used in the command.
   *
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted)
  {

  }
}
