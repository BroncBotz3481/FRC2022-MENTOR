// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  /**
   * USB Controller ports on the driver station laptop.
   */
  public static final int DriverControllerPort = 0, OperatorControllerPort = 1;
  /**
   * Shooter Motor CAN ID for Talon SRX shooter.
   */
  public static final int ShooterMotorCANID         = 1;
  /**
   * Shooter motor which follows the main shooter motor.
   */
  public static final int ShooterFollowerMotorCANID = 2;
  /**
   * Intake motor on drop down intake
   */
  public static final int IntakeMotorCANID          = 7;
  /**
   * Index Motor for the conveyer belt that feeds into shooter.
   */
  public static final int IndexMotorCANID           = 6;
  /**
   * Index pressure pad PWM port for detection of wether or not a ball is currently in place for shooting.
   */
  public static final int IndexPressurePadPWMPort   = 0;

  /**
   * PDP or PDH CAN ID.
   */
  public static final int PowerDistributionCANID = 20;

  /**
   * REV Pneumatic HUB CANID
   */
  public static final int PneumaticHubCANID = 40;

  /**
   * Ensure the value given does not exceed or go bellow the range given, if it does return the boundaries.
   *
   * @param val Value to check
   * @param min Minimum value.
   * @param max Maximum value.
   * @return value or minimum/maximum if range is exceeded/bellow target.
   */
  public static double clamp(double val, double min, double max)
  {
    return Math.max(min, Math.min(max, val));
  }

  /**
   * Drive Train Motor CAN ID's which are used in the Drive Train Subsystem
   */
  public static class DriveTrain
  {

    /**
     * SparkMax CAN ID for each motor controller, assuming standard drive train. []---------[] | | | | |          |
     * []---------[]
     */
    public static final int FrontLeftCANID = 4,
        FrontRightCANID                    = 1,
        BackLeftCANID                      = 3,
        BackRightCANID                     = 2;
    /**
     * PigeonIMU 2.0 CAN ID.
     */
    public static final int PigeonIMUCANID = 34;

    /**
     * Drive train width from left side to right side.
     */
    public static final double driveTrainWidth = Units.inchesToMeters(27.0);
    /**
     * Wheel diameter in meters.
     */
    public static final double wheelDiameter   = Units.inchesToMeters(4.0);
    /**
     * Length of robot along the wheels
     */
    public static final double wheelBase       = Units.inchesToMeters(24);

    /**
     * The power scale to apply to the robot when it is going forwards.
     */
    public static final double forwardsPowerScale           = 0.5;
    /**
     * The power scale to apply to the robot when it is going backwards.
     */
    public static final double backwardsPowerScale          = 0.5;
    /**
     * Overdrive forwards power scale.
     */
    public static final double overdriveForwardsPowerScale  = 0.8;
    /**
     * Overdrive backwards power scale.
     */
    public static final double overdriveBackwardsPowerScale = 0.7;
  }

  /**
   * Intake Pnuematics channels on the CTRE PCM.
   */
  public static class IntakePneumatics
  {

    /**
     * Channels for each the pneumatics on the PCM.
     */
    public static final int ForwardChannel = 7, ReverseChannel = 6;
  }

  /**
   * Hood pneumatics on the CTRE PCM.
   */
  public static class HoodPneumatics
  {

    /**
     * Channels for each of the pneumatics on the PCM for the HOOD.
     */
    public static final int ForwardChannel = 0, ReverseChannel = 1;
  }

  /**
   * Climber motors for Talon SRX's controlling the Climber.
   */
  public static class Climber
  {
    /**
     * Left climber motor CANID.
     */
    public static final int LeftClimberMotorCANID = 4, RightClimberMotorCANID = 5;
  }


}
