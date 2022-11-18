package frc.robot.commands.index;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.index.IndexSubsystem;

/**
 * Reverse the index to move balls towards the shooter.
 */
public class ReverseIndexCommand extends CommandBase
{

  /**
   * Index subsystem.
   */
  private final IndexSubsystem indexSubsystem;

  /**
   * Constructor to control the index subsystem.
   *
   * @param indexSubsystem Initialized index subsystem.
   */
  public ReverseIndexCommand(IndexSubsystem indexSubsystem)
  {
    this.indexSubsystem = indexSubsystem;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.indexSubsystem);
  }

  /**
   * Stop the index subsystem when command is scheduled.
   */
  @Override
  public void initialize()
  {
    indexSubsystem.stop();
  }

  /**
   * Reverse the index when the command is running.
   */
  @Override
  public void execute()
  {
    indexSubsystem.reverse();
  }

  /**
   * Command must be interrupted and never finishes.
   *
   * @return false.
   */
  @Override
  public boolean isFinished()
  {
    return false;
  }

  /**
   * When the command ends the index stops.
   *
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted)
  {
    indexSubsystem.stop();

  }
}
