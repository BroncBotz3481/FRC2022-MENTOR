package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.index.IndexPolicy;
import frc.robot.subsystems.intake.IntakePolicy;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.vision.VisionPolicy;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * Magically intake balls of our color using a improvised vision subsystem.
 */
public class MagicIntakeCommand extends CommandBase
{

  /**
   * Intake subsystem.
   */
  private final IntakeSubsystem intakeSubsystem;
  /**
   * Vision subsystem is fetched through the static reference.
   */
  private final VisionSubsystem visionSubsystem = VisionSubsystem.getInstance();

  /**
   * Timer for how long there hasn't been a ball of the appropriate color infront of the camera.
   */
  private final Timer   lostBall   = new Timer();
  /**
   * If there is a ball of the appropriate color infront of the camera.
   */
  private       boolean foundABall = false;
  /**
   * Is the robot on the blue alliance?
   */
  private       boolean blueAlliance;

  /**
   * Raises the intake when there is an appropriately collored ball infront of the intake. Intake will only be down for
   * a second at most.
   *
   * @param intakeSubsystem intake subsystem instance.
   */
  public MagicIntakeCommand(IntakeSubsystem intakeSubsystem)
  {
    this.intakeSubsystem = intakeSubsystem;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.intakeSubsystem);
  }

  /**
   * The initial subroutine of a command.  Called once when the command is initially scheduled. Intake will last at most
   * 1 second. Resets the lost ball timer.
   */
  @Override
  public void initialize()
  {
    lostBall.reset();
    IntakePolicy.timeoutSeconds = 1; // Set the intake to timeout after 1 second.
    blueAlliance = DriverStation.getAlliance() == DriverStation.Alliance.Blue;
  }

  /**
   * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
   * until {@link #isFinished()}) returns true.) NOTE: This will slow down the entire command scheduler. Execute the
   * command and process exclusively blue or red balls depending on our alliance. If a ball of the appropriate color is
   * found it will drop and intake the ball.
   */
  @Override
  public void execute()
  {
    // THIS WILL SLOW DOWN THE ENTIRE COMMAND SCHEDULER.

    if (blueAlliance)
    {
      visionSubsystem.processBlueBalls();
    } else
    {
      visionSubsystem.processRedBalls();
    }

    if (VisionPolicy.ballFound()
        &&
        !IndexPolicy.indexFull()) // If a ball is there, the index is not full and it has not timed out, set the
    // power and ensure it is dropped.
    {
      intakeSubsystem.drop();
      intakeSubsystem.suck();
      lostBall.reset();
      lostBall.start();
      foundABall = true;
    } else if (lostBall.hasElapsed(1) && foundABall)
    {
      intakeSubsystem.raise();
      intakeSubsystem.roll(0);
      lostBall.stop();
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
   * This command must be interrupted.
   *
   * @return whether this command has finished.
   */
  @Override
  public boolean isFinished()
  {
//        return !IntakePolicy.intakeRaised;
//        return !VisionPolicy.ballFound() && foundABall;
    return false;
  }

  /**
   * The action to take when the command ends. Called when either the command finishes normally -- that is it is called
   * when {@link #isFinished()} returns true -- or when  it is interrupted/canceled. This is where you may want to wrap
   * up loose ends, like shutting off a motor that was being used in the command.
   * When the command ends the intake will be raised and roller stopped.
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted)
  {
    intakeSubsystem.raise();
    intakeSubsystem.roll(0);
  }
}
