package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.IntakePolicy;
import frc.robot.subsystems.intake.IntakeSubsystem;

/**
 * Lower the intake and suck in a ball.
 */
public class LowerAndSuckIntakeCommand extends CommandBase
{

  /**
   * Intake subsystem.
   */
  private final IntakeSubsystem intakeSubsystem;

  /**
   * Constructor for the command requires the intake subsystem.
   *
   * @param intakeSubsystem Initialized intake subsystem.
   */
  public LowerAndSuckIntakeCommand(IntakeSubsystem intakeSubsystem)
  {
    this.intakeSubsystem = intakeSubsystem;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.intakeSubsystem);
  }

  /**
   * The initial subroutine of a command.  Called once when the command is initially scheduled. Intake will run for at
   * most 3 seconds.
   */
  @Override
  public void initialize()
  {
    IntakePolicy.timeoutSeconds = 3; // Set the intake to timeout after 3 seconds.
  }

  /**
   * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
   * until {@link #isFinished()}) returns true.)
   * Lower the intake and suck in balls.
   */
  @Override
  public void execute()
  {
    intakeSubsystem.drop();
    intakeSubsystem.suck();
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
   * Finish the command when the intake times out or has been raised.
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
   * Raises the intake and stops the roller.
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted)
  {
    intakeSubsystem.raise();
    intakeSubsystem.roll(0);
  }
}
