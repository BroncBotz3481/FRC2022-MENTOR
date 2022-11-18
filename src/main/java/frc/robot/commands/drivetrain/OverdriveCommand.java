package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.DrivetrainPolicy;

/**
 * Overdrive Command which could probably be implemented as an instant command in the future.
 */
public class OverdriveCommand extends CommandBase
{

  /**
   * Constructor for the OverdriveCommand requires no subsystems since it is an algorithmic change.
   */
  public OverdriveCommand()
  {
  }

  /**
   * No actions are required during initialization for Overdrive.
   */
  @Override
  public void initialize()
  {

  }

  /**
   * Change the power scales to the overdrive power scale defined in {@link Constants.DriveTrain}.
   */
  @Override
  public void execute()
  {
    DrivetrainPolicy.forwardsPowerScale = Constants.DriveTrain.overdriveForwardsPowerScale;
    DrivetrainPolicy.backwardsScale = Constants.DriveTrain.overdriveBackwardsPowerScale;
  }

  /**
   * This command will not exit and must be interrupted.
   *
   * @return false.
   */
  @Override
  public boolean isFinished()
  {
    return false;
  }

  /**
   * When the command is interrupted the power scales will return to normal as defined by {@link Constants.DriveTrain}.
   *
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted)
  {
    DrivetrainPolicy.forwardsPowerScale = Constants.DriveTrain.forwardsPowerScale;
    DrivetrainPolicy.backwardsScale = Constants.DriveTrain.backwardsPowerScale;
  }
}
