/****************************** Header ******************************\
 Class Name: ExampleCommand extends CommandBase
 File Name: ExampleCommand.java
 Summary: An example command to use for learning and testing.
 Project: BroncBotzFRC2023
 Copyright (c) BroncBotz.
 All rights reserved.

 Author(s): Dylan Watson
 \********************************************************************/


package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveTrain;
import frc.robot.subsystems.drivetrain.DrivetrainPolicy;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import java.util.function.DoubleSupplier;

/**
 * DriveTrain command for a DifferentialDrive driven by tank drive controlls.
 */
public class DrivetrainCommand extends CommandBase
{

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DrivetrainSubsystem m_driveTrainSubsystem;
  private final DoubleSupplier      leftPower;
  private final DoubleSupplier      rightPower;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   * @param left      A function providing a double as the input for the left side of the robot.
   * @param right     A funuction reference providing a double as the input for the right side of the robot.
   */
  public DrivetrainCommand(DrivetrainSubsystem subsystem, DoubleSupplier left,
                           DoubleSupplier right)
  {
    leftPower = left;
    rightPower = right;
    m_driveTrainSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    DrivetrainPolicy.forwardsPowerScale = DriveTrain.forwardsPowerScale;
    DrivetrainPolicy.backwardsScale = DriveTrain.backwardsPowerScale;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    m_driveTrainSubsystem.run(leftPower.getAsDouble(), rightPower.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }
}
