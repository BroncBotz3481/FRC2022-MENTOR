package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.index.IndexPolicy;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * Prepare the shooter if there is a ball on the pressure pad.
 */
public class PrepareShooterCommand extends CommandBase
{

  /**
   * Shooter subsystem instance.
   */
  private final ShooterSubsystem shooterSubsystem;

  /**
   * Shooter will roll at 20% speed when there is a ball on the pressure pad.
   *
   * @param shooterSubsystem Shooter subsystem instance.
   */
  public PrepareShooterCommand(ShooterSubsystem shooterSubsystem)
  {
    this.shooterSubsystem = shooterSubsystem;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.shooterSubsystem);
  }

  /**
   * The initial subroutine of a command.  Called once when the command is initially scheduled.
   */
  @Override
  public void initialize()
  {

  }

  /**
   * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
   * until {@link #isFinished()}) returns true.) Using the function
   * {@link frc.robot.subsystems.intake.IntakePolicy}.indexFull() to determine if there is a ball ready run the shooter
   * at 20%.
   */
  @Override
  public void execute()
  {
    shooterSubsystem.setShooter(IndexPolicy.indexFull() ? 0.2 : 0);
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
   * Command must be interrupted.
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
   * Stop the shooter.
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted)
  {
    shooterSubsystem.setShooter(0);
  }
}
