package frc.robot.commands.index;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.index.IndexSubsystem;

/**
 * Run the index motor to send balls away from the shooter.
 */
public class ForwardIndexCommand extends CommandBase
{

  /**
   * IndexSubsystem to control.
   */
  private final IndexSubsystem indexSubsystem;

  /**
   * Constructor for command which requires the exclusive control of the index subsystem.
   *
   * @param indexSubsystem Initialized index subsystem.
   */
  public ForwardIndexCommand(IndexSubsystem indexSubsystem)
  {
    this.indexSubsystem = indexSubsystem;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.indexSubsystem);
  }

  /**
   * Stops the index when command is scheduled.
   */
  @Override
  public void initialize()
  {
    indexSubsystem.stop();
  }

  /**
   * Index's the ball as long as the command is active.
   */
  @Override
  public void execute()
  {
    indexSubsystem.forward();
  }

  /**
   * This command never finishes.
   *
   * @return false
   */
  @Override
  public boolean isFinished()
  {
    return false;
  }

  /**
   * Stop the index.
   *
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted)
  {
    indexSubsystem.stop();

  }
}
