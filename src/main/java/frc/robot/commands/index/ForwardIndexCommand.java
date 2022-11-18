package frc.robot.commands.index;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.index.IndexSubsystem;


public class ForwardIndexCommand extends CommandBase
{

  private final IndexSubsystem indexSubsystem;

  public ForwardIndexCommand(IndexSubsystem indexSubsystem)
  {
    this.indexSubsystem = indexSubsystem;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.indexSubsystem);
  }

  @Override
  public void initialize()
  {
    indexSubsystem.stop();
  }

  @Override
  public void execute()
  {
    indexSubsystem.forward();
  }

  @Override
  public boolean isFinished()
  {
    return false;
  }

  @Override
  public void end(boolean interrupted)
  {
    indexSubsystem.stop();

  }
}
