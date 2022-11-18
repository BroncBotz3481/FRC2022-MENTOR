package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.IntakePolicy;
import frc.robot.subsystems.intake.IntakeSubsystem;

/**
 * Lowers the intake and outtakes a ball.
 */
public class LowerAndSpitIntakeCommand extends CommandBase
{

  /**
   * Intake subsystem.
   */
  private final IntakeSubsystem intakeSubsystem;

  /**
   * Constructor for command requires control of the intake subsystem.
   *
   * @param intakeSubsystem Initialized intake subsystem.
   */
  public LowerAndSpitIntakeCommand(IntakeSubsystem intakeSubsystem)
  {
    this.intakeSubsystem = intakeSubsystem;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.intakeSubsystem);
  }

  /**
   * The initial subroutine of a command.  Called once when the command is initially scheduled. The intake will drop and
   * run for AT MOST 3 seconds.
   */
  @Override
  public void initialize()
  {
    IntakePolicy.timeoutSeconds = 3; // Set the intake to timeout after 3 seconds.
  }

  /**
   * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
   * until {@link #isFinished()}) returns true.)
   * Drop the intake and outtake the ball.
   */
  @Override
  public void execute()
  {
    intakeSubsystem.drop();
    intakeSubsystem.spit();
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
   * Exit when the intake has timed out or detected to have been raised.
   * @return whether this command has finished.
   */
  @Override
  public boolean isFinished()
  {
    return IntakePolicy.getTimeout() || IntakePolicy.intakeRaised;
  }

  /**
   * The action to take when the command ends. Called when either the command finishes normally -- that is it is called
   * when {@link #isFinished()} returns true -- or when  it is interrupted/canceled. This is where you may want to wrap
   * up loose ends, like shutting off a motor that was being used in the command.
   * Raise and stop the intake.
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted)
  {
    intakeSubsystem.raise();
    intakeSubsystem.roll(0);
  }
}
