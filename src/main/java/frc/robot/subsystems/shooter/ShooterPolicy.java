/****************************** Header ******************************\
 Class Name: IntakeModule [final]
 File Name: IntakeModule.java
 Summary: Contains constant subclasses and variables for commands, subsystems, and utility methods
 Project: BroncBotzFRC2023
 Copyright (c) BroncBotz.
 All rights reserved.

 Author(s): Shruti Venkat and Samuel Zhao
 \********************************************************************/

package frc.robot.subsystems.shooter;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import frc.robot.Constants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class ShooterPolicy implements Sendable, AutoCloseable
{

  /**
   * Is the sendable initialized.
   */
  public static final ShooterPolicy self                   = new ShooterPolicy();
  /**
   * Ticks per rotation on the encoder.
   */
  public static final int           ticksPerRotation       = 1023;
  /**
   * CAN Bus communication timeout.
   */
  public static final int           communicationTimeoutMs = 50;
  /**
   * High target PID RPM
   */
  public static       int           highPIDTarget          = 12000;
  /**
   * Low target PID RPM.
   */
  public static       int           lowPIDTarget           = 6914;
  /**
   * Raw power applied to the shooter.
   */
  public static       double        shooterPower           = 0;
  /**
   * Shooter RPM pid target.
   */
  public static       int           pidTarget              = 0;
  /**
   * Shooter ticks per 100ms.
   */
  public static       double        shooterSensorVelocity  = 0;
  /**
   * Shooter RPM
   */
  public static       double        shooterVelocity        = 0;

  /**
   * Get the real velocity to send to the TalonSRX from the target RPM. The velocity is how many ticks per 100
   * milliseconds.
   *
   * @param targetRPM Rotations per minute
   * @return ticks per 100 milliseconds.
   */
  public static double getRealVelocity(double targetRPM)
  {
        /*
        // 1m = 60s
        // 1s = 1000ms
        // 1m = 60000ms
        double targetTicksPerMinute = targetRPM * ticksPerRotation;
        double targetTicksPerSecond = targetTicksPerMinute / 60;
        double targetTicksPerMillisecond = targetTicksPerSecond / 1000;
        // The TalonSRX samples velocity every 100ms with the getSelectedSensorPosition() function.
        double targetTicksPer100Milliseconds = targetTicksPerMillisecond * 100;
        */

    return (targetRPM * ticksPerRotation) / 600;
  }

  /**
   * Return the shooter power percentage as a value between -1 and 1.
   *
   * @return the shooter power percentage.
   */
  public static double getShooterPower()
  {
    return Constants.clamp(ShooterPolicy.shooterPower, -1, 1);
  }

  /**
   * Get the RPM from the velocity of the TalonSRX. Converts ticks per 100ms to RPM.
   *
   * @param realVelocity ticks per 100ms
   * @return RPM
   */
  public static double fromRealVelocity(double realVelocity)
  {
        /*
        // 1m = 60s
        // 1s = 1000ms
        // 1m = 60000ms
        double targetTicksPer100Milliseconds = realVelocity;
        double targetTicksPerMillisecond = targetTicksPer100Milliseconds / 100;
        double targetTicksPerSecond = targetTicksPerMillisecond * 1000;
        double targetTicksPerMinute = targetTicksPerSecond * 60;
        double rotationsPerMinute = targetTicksPerMinute / ticksPerRotation;
        */

    return realVelocity / ticksPerRotation * 600;
  }

  /**
   * Is the velocity within the range of acceptable parameters?
   *
   * @param range RPM range of acceptable parameters +-
   * @return bool if is in range.
   */
  public static boolean velocityWithinRange(double range)
  {
    return pidTarget - range < shooterSensorVelocity && pidTarget + range > shooterSensorVelocity;
  }

  /**
   * Initializes this {@link Sendable} object.
   *
   * @param builder sendable builder
   */
  @Override
  public void initSendable(SendableBuilder builder)
  {
    builder.setSmartDashboardType("RobotPreferences");
    builder.addDoubleProperty("setpoint", () -> {
      return pidTarget;
    }, (double x) -> {
      pidTarget = (int) x;
    });
    builder.addDoubleProperty("HighShot", () -> {
      return highPIDTarget;
    }, (double x) -> {
      highPIDTarget = (int) x;
    });
    builder.addDoubleProperty("LowShot", () -> {
      return lowPIDTarget;
    }, (double x) -> {
      lowPIDTarget = (int) x;
    });
    builder.addDoubleProperty("Velocity", () -> {
      return shooterVelocity;
    }, (x) -> {
    });
  }

  /**
   * Breaks down the sendable.
   *
   * @throws Exception if it can't remove the sendable.
   */
  @Override
  public void close() throws Exception
  {
    SendableRegistry.remove(this);
  }
}
