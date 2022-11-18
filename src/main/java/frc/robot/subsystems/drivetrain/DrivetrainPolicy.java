/****************************** Header ******************************\
 Class Name: DriveTrain
 File Name: Drivetrain.java
 Summary: Contains constant subclasses and variables for commands, subsystems, and utility methods
 Project: BroncBotzFRC2023
 Copyright (c) BroncBotz.
 All rights reserved.

 Author(s): Samuel Zhao and Shruti Venkat
 \********************************************************************/

package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import frc.robot.Constants.DriveTrain;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class DrivetrainPolicy
{

  /**
   * Drivetrain odometry for keeping track of the pose.
   */
  public static DifferentialDriveOdometry   driveOdometry;
  /**
   * Drivetrain kinematics for easy chasis speed calculations.
   */
  public static DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(
      DriveTrain.driveTrainWidth);
  /**
   * Minimum amount of power required before the controller moves. AKA "Deadzone"
   */
  public static double deadZone             = 0.05;
  /**
   * Encoder values continously updates for your convienence. Velocity = meters/second
   */
  public static double rightEncoderPosition = 0, rightEncoderVelocity = 0, leftEncoderPositionDelta = 0,
      rightEncoderPositionDelta             = 0, leftEncoderPosition = 0, leftEncoderVelocity = 0;

  /**
   * Power for the left side of the drive train.
   */
  public static  double powerLeft;
  /**
   * Power for the right side of the drive train.
   */
  public static  double powerRight; //power for right motors
  /**
   * The speed to apply to the right drive train in meters per second.
   */
  public static  double rightSpeedMPS;
  /**
   * The speed to apply to the left drive train in meters per second.
   */
  public static  double leftSpeedMPS;
  /**
   * Power scale to be applied to the output when given that way the motors don't draw too much power.
   */
  public static  double forwardsPowerScale = DriveTrain.forwardsPowerScale;
  /**
   * Additional power scale applied when going backwards. to avoid flipping. (Multiplier of the scale)
   */
  public static  double backwardsScale     = DriveTrain.backwardsPowerScale;
  /**
   * Calculated applied power scale based off directions.
   */
  private static double appliedPowerScale  = forwardsPowerScale;

  /**
   * Left drive train power as a percentage scaled between -1 and 1.
   *
   * @return value between -1 and 1.
   */
  public static double getLeftPower()
  {
    DrivetrainPolicy.powerLeft =
        DrivetrainPolicy.powerLeft > DrivetrainPolicy.deadZone ? DrivetrainPolicy.powerLeft : 0;
    if (DrivetrainPolicy.powerLeft < 0 && DrivetrainPolicy.powerRight < 0)
    {
      // We are going backwards.
      DrivetrainPolicy.appliedPowerScale = DrivetrainPolicy.backwardsScale;
    } else
    {
      DrivetrainPolicy.appliedPowerScale = DrivetrainPolicy.forwardsPowerScale;
    }
    return MathUtil.clamp(DrivetrainPolicy.powerLeft * DrivetrainPolicy.appliedPowerScale, -1, 1);
  }

  /**
   * Right drive train power as a percentage scaled between -1 and 1. Minimum amount required is in the
   * DrivetrainPolicy.deadZone attribute.
   *
   * @return value -1 and 1
   */
  public static double getRightPower()
  {
    DrivetrainPolicy.powerRight =
        DrivetrainPolicy.powerRight > DrivetrainPolicy.deadZone ? DrivetrainPolicy.powerRight : 0;
    if (DrivetrainPolicy.powerLeft < 0 && DrivetrainPolicy.powerRight < 0)
    {
      // We are going backwards.
      DrivetrainPolicy.appliedPowerScale = DrivetrainPolicy.backwardsScale;
    } else
    {
      DrivetrainPolicy.appliedPowerScale = DrivetrainPolicy.forwardsPowerScale;
    }
    return MathUtil.clamp(DrivetrainPolicy.powerRight * DrivetrainPolicy.appliedPowerScale, -1, 1);
  }

  /**
   * Get the velocity based on the left speed in the drive train policy by converting meters per second to RPM.
   *
   * @return RPM for the left motor.
   */
  public static double getLeftVelocityRPM()
  {
    return MPStoRPM(DrivetrainPolicy.leftSpeedMPS);
  }

  /**
   * Get the velocity based off the right speed in the drivetrainpolicy by converting meters per second to RPM.
   *
   * @return RPM for the right motor.
   */
  public static double getRightVelocityRPM()
  {
    return MPStoRPM(DrivetrainPolicy.rightSpeedMPS);

  }

  /**
   * Rotations per Minute to Meters per Second, using wheel diameter.
   *
   * @param RPM Rotations Per Minute
   * @return Meters per second.
   */
  public static double RPMtoMPS(double RPM)
  {
    return (RPM / 60) * DriveTrain.wheelDiameter * Math.PI;
  }

  /**
   * Meters per second to rotations per minute using wheel diameter.
   *
   * @param MPS Meters per second.
   * @return Rotations per minute.
   */
  public static double MPStoRPM(double MPS)
  {
    return ((MPS * 60) / (Math.PI * DriveTrain.wheelDiameter));
  }

  /**
   * Convert rotations to meters travelled
   *
   * @param rotations rotations of the motor.
   * @return Meters travelled.
   */
  public static double RotationsToMeters(double rotations)
  {
    return rotations * DriveTrain.wheelDiameter * Math.PI;
  }

  /**
   * Update the Policy classes encoder positions and delta's using raw inputs from the drive train encoders.
   *
   * @param leftRotations  Left encoder total rotations.
   * @param rightRotations Right encoder total rotations.
   */
  public static void UpdateEncoderPositions(double leftRotations, double rightRotations)
  {
    DrivetrainPolicy.rightEncoderPositionDelta = DrivetrainPolicy.rightEncoderPosition;
    DrivetrainPolicy.leftEncoderPositionDelta = DrivetrainPolicy.leftEncoderPosition;

    /** Set the position based off total ticks given wheel diameter and pi for circumference. **/
    DrivetrainPolicy.rightEncoderPosition = DrivetrainPolicy.RotationsToMeters(rightRotations);
    DrivetrainPolicy.leftEncoderPosition = DrivetrainPolicy.RotationsToMeters(leftRotations);

    /** Get distance travelled for odometry updates. **/
    DrivetrainPolicy.rightEncoderPositionDelta -= DrivetrainPolicy.rightEncoderPosition;
    DrivetrainPolicy.leftEncoderPositionDelta -= DrivetrainPolicy.leftEncoderPosition;
  }

  /**
   * Get the current Chassis Speeds
   *
   * @return ChassisSpeeds object based off left and right meters per second.
   */
  public static ChassisSpeeds getChassisSpeeds()
  {
    return driveKinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(leftEncoderVelocity, rightEncoderVelocity));
  }

  /**
   * Maximum voltage the drive train should use in Autonomous.
   */
  public static double maxVoltage = 10;

  /**
   * FeedForward Constants generated by path finder or manually configured.
   */
  public static class FeedForwardConstants
  {

    /**
     * Minimum voltage required to move the motor.
     */
    public static double kS;
    /**
     * Units to smoothly follow points, units are voltage * (seconds/distance).
     */
    public static double kV;
    // kA shouldn't be needed but it is the unit voltage * ((seconds^2)/distance)
  }

  /**
   * Trajectory generation constraints.
   */
  public static class TrajectoryVelocityConstraints
  {

    /**
     * Max velocity per second for the trajectory. Used in
     * {@link frc.robot.commands.autonomous.TrajectorySampleCommand}
     */
    public static double maxVelocityPerSecond                  = 12;
    /**
     * Max acceleration in meters per second for the trajectory. Used in
     * {@link frc.robot.commands.autonomous.TrajectorySampleCommand}
     */
    public static double maxAccelerationMetersPerSecondSquared = 12;
  }
}
