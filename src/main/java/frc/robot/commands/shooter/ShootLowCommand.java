package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.index.IndexPolicy;
import frc.robot.subsystems.index.IndexSubsystem;
import frc.robot.subsystems.shooter.ShooterPolicy;
import frc.robot.subsystems.shooter.ShooterSubsystem;


public class ShootLowCommand extends CommandBase
{

  /**
   * Shooter subsystem
   */
  private final ShooterSubsystem shooterSubsystem;
  /**
   * Index subsystem.
   */
  private final IndexSubsystem   indexSubsystem;

  /**
   * Run the shooter at the high velocity and roll the indexer when it reaches the velocity.
   *
   * @param shooterSubsystem Shooter subsystem instance.
   * @param indexSubsystem   Index subsystem instance.
   */
  public ShootLowCommand(ShooterSubsystem shooterSubsystem, IndexSubsystem indexSubsystem)
  {
    this.shooterSubsystem = shooterSubsystem;
    this.indexSubsystem = indexSubsystem;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.shooterSubsystem, this.indexSubsystem);
  }

  /**
   * The initial subroutine of a command.  Called once when the command is initially scheduled.
   */
  @Override
  public void initialize()
  {
    IndexPolicy.indexOverride = true;
    shooterSubsystem.raiseHood();
  }

  /**
   * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
   * until {@link #isFinished()}) returns true.)
   */
  @Override
  public void execute()
  {
    shooterSubsystem.setShooterVelocity(ShooterPolicy.lowPIDTarget);
    if (ShooterPolicy.velocityWithinRange(100))
    {
      indexSubsystem.reverse();
    } else
    {
      indexSubsystem.stop();
    }
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
    IndexPolicy.indexOverride = false;
    shooterSubsystem.setShooter(0);
    indexSubsystem.stop();
  }
}
