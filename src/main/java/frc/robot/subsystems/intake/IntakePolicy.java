/****************************** Header ******************************\
 Class Name: IntakePolicyClass
 File Name: IntakePolicy.java
 Summary: Contains constant subclasses and variables for commands, subsystems, and utility methods
 Project: BroncBotzFRC2023
 Copyright (c) BroncBotz.
 All rights reserved.

 Author(s): Shruti Venkat and Samuel Zhao
 \********************************************************************/

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.index.IndexPolicy;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class IntakePolicy
{

  /**
   * Timer is started whenever the intake is dropped and stopped whenever it is raised.
   */
  public static final Timer  timeout     = new Timer();
  /**
   * Power to set to the intake motors, usually [1, 0.8, or 0]
   */
  public static       double intakePower = 0;

  /**
   * Intake Solenoid value to set the intake too (NOTE: It may not be set to this if the timer runs out).
   */
  public static DoubleSolenoid.Value intakeSolenoid = IntakeSolenoidMode.UP;

  /**
   * Periodically updates with sensor based information on whether the intake is raised.
   */
  public static boolean intakeRaised = true;

  /**
   * How long the intake can be down for before retracting, set in seconds.
   */
  public static int timeoutSeconds = 3;

  /**
   * Whether the timeout has occurred.
   *
   * @return timeout occurred.
   */
  public static boolean getTimeout()
  {
    return IntakePolicy.timeout.hasElapsed(IntakePolicy.timeoutSeconds);
  }

  /**
   * Is the intake allowed to drop down. Depending on the timeout and if the index is full.
   *
   * @return allowed to drop.
   */
  public static boolean dropdownInvalid()
  {
    return IndexPolicy.indexFull() || IntakePolicy.getTimeout();
  }

  /**
   * Gets the power the will be supplied to the intake motors in percentages and prevents the motors from being ran if
   * the Index is full or if the timer has run out.
   *
   * @return The intake power to set.
   */
  public static double getIntakePower()
  {
    if (IntakePolicy.dropdownInvalid())
    {
      IntakePolicy.intakePower = 0;
    }
    return Constants.clamp(IntakePolicy.intakePower, -1, 1);
  }

  /**
   * Gets the value to set the solenoid to and forces the intake to raise if the timeout has occurred.
   *
   * @return the intake solenoid value.
   */
  public static DoubleSolenoid.Value getIntakeSolenoid()
  {
    if (IntakePolicy.dropdownInvalid())
    {
      IntakePolicy.intakeSolenoid = IntakeSolenoidMode.UP;
    }
    return IntakePolicy.intakeSolenoid;
  }

  /**
   * Aliases for the Solenoid value that represents whether the intake is up or down.
   */
  public static class IntakeSolenoidMode
  {

    /**
     * DoubleSolenoid value for intake up.
     */
    public static final DoubleSolenoid.Value UP   = DoubleSolenoid.Value.kReverse;
    /**
     * DoubleSolenoid value for intake down.
     */
    public static final DoubleSolenoid.Value DOWN = DoubleSolenoid.Value.kForward;
  }
}
