/****************************** Header ******************************\
 Class Name: IntakePolicyClass
 File Name: IntakePolicy.java
 Summary: Contains constant subclasses and variables for commands, subsystems, and utility methods
 Project: BroncBotzFRC2023
 Copyright (c) BroncBotz.
 All rights reserved.

 Author(s): Shruti Venkat and Samuel Zhao
 \********************************************************************/

package frc.robot.subsystems.climber;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class ClimberPolicy
{

  public static double leftPowerClimb  = 0;
  public static double rightPowerClimb = 0;
  public static double power           = 0;

  public static double getPower()
  {
    if (LeftColorSensor.red < 50)
    {
      return 0;
    }
    return ClimberPolicy.power;
  }

  public static class LeftColorSensor
  {

    public static int red, green, blue, ir, proximity;
    public static boolean connected;
  }

}
