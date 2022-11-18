package frc.robot.commands.autonomous;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.DrivetrainPolicy;
import frc.robot.subsystems.drivetrain.DrivetrainPolicy.FeedForwardConstants;
import frc.robot.subsystems.drivetrain.DrivetrainPolicy.TrajectoryVelocityConstraints;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import java.util.ArrayList;
import java.util.List;


public class TrajectorySampleCommand extends CommandBase
{

  private final DrivetrainSubsystem                drivetrainSubsystem;
  /**
   * Ramsete Controller we will use.
   */
  private final RamseteController                  ramseteController;
  private final Timer                              autoTimer = new Timer();
  /**
   * Trajectory Configuration controlling the attributes of the trajectory.
   */
  private       TrajectoryConfig                   config;
  /**
   * Voltage constraint pulled from the SysID suite for differential drive.
   */
  private       DifferentialDriveVoltageConstraint voltageConstraint;
  /**
   * Trajectory to follow.
   */
  private       Trajectory                         trajectory;

  /**
   * Sample Trajectory Generation and Follower command using the Ramsete controller.
   *
   * @param drivetrainSubsystem Drivetrain Subsystem to use.
   */
  public TrajectorySampleCommand(DrivetrainSubsystem drivetrainSubsystem)
  {
    this.drivetrainSubsystem = drivetrainSubsystem;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.drivetrainSubsystem);
    generateTrajectory();
    ramseteController = new RamseteController(); // Default controller unless otherwise needed configuration.
  }

  /**
   * Generate the voltage configuration attribute.
   *
   * @return DifferentialDrive voltage constraint generated using feedforward constants in DrivetrainPolicy.
   */
  private DifferentialDriveVoltageConstraint generateVoltageConfig()
  {
    voltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(FeedForwardConstants.kS, FeedForwardConstants.kV),
        DrivetrainPolicy.driveKinematics,
        DrivetrainPolicy.maxVoltage);
    return voltageConstraint;
  }

  /**
   * Generate the trajectory configuration with limitations.
   *
   * @return Trajectory configuration with max velocity and acceleration pulled from TrajectoryVelocityConstraints.
   */
  private TrajectoryConfig generateTrajectoryConfig()
  {
    config = new TrajectoryConfig(TrajectoryVelocityConstraints.maxVelocityPerSecond,
                                  TrajectoryVelocityConstraints.maxAccelerationMetersPerSecondSquared);
    config.addConstraint(generateVoltageConfig());
    config.setKinematics(DrivetrainPolicy.driveKinematics); // Add Constraint based off our Kinematics.
    // Start and end velocity defaulted to 0, driving in reverse is set to false.
    return config;
  }

  /**
   * Generate a list a waypoints that we expect to traverse.
   *
   * @return List of waypoints absolute based off 0,0
   */
  private List<Translation2d> generateWaypoints()
  {
    ArrayList<Translation2d> waypoints = new ArrayList<Translation2d>();
    waypoints.add(new Translation2d(Units.feetToMeters(5.14), Units.feetToMeters(3.7)));
    waypoints.add(new Translation2d(Units.feetToMeters(6.14), Units.feetToMeters(8)));
    return waypoints;
  }

  /**
   * Generate the trajectory with constraints and configurations applied.
   */
  private void generateTrajectory()
  {
    Pose2d start = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    Pose2d end   = new Pose2d(7, 3, Rotation2d.fromDegrees(120));
    trajectory = TrajectoryGenerator.generateTrajectory(start, generateWaypoints(), end, generateTrajectoryConfig());
  }

  /**
   * The initial subroutine of a command.  Called once when the command is initially scheduled.
   */
  @Override
  public void initialize()
  {
    autoTimer.reset();
    autoTimer.start();
  }

  /**
   * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
   * until {@link #isFinished()}) returns true.)
   */
  @Override
  public void execute()
  {
    ChassisSpeeds speeds = ramseteController.calculate(DrivetrainPolicy.driveOdometry.getPoseMeters(),
                                                       trajectory.sample(autoTimer.get()));
    DifferentialDriveWheelSpeeds attainableSpeeds = DrivetrainPolicy.driveKinematics.toWheelSpeeds(speeds);
    drivetrainSubsystem.set(attainableSpeeds.leftMetersPerSecond, attainableSpeeds.rightMetersPerSecond);
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
    // TODO: Make this return true when this Command no longer needs to run execute()
    return ramseteController.atReference() || DriverStation.isTeleop() || autoTimer.hasElapsed(15);
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
    autoTimer.stop();
  }
}
