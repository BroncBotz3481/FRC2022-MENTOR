package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.DrivetrainPolicy;


public class OverdriveCommand extends CommandBase
{

  public OverdriveCommand()
  {
  }

  @Override
  public void initialize()
  {

  }

  @Override
  public void execute()
  {
    DrivetrainPolicy.forwardsPowerScale = Constants.DriveTrain.overdriveForwardsPowerScale;
    DrivetrainPolicy.backwardsScale = Constants.DriveTrain.overdriveBackwardsPowerScale;
  }

  @Override
  public boolean isFinished()
  {
    return false;
  }

  @Override
  public void end(boolean interrupted)
  {
    DrivetrainPolicy.forwardsPowerScale = Constants.DriveTrain.forwardsPowerScale;
    DrivetrainPolicy.backwardsScale = Constants.DriveTrain.backwardsPowerScale;
  }
}
