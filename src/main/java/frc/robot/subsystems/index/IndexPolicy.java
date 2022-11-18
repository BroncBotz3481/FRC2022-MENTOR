/****************************** Header ******************************\
 Class Name: IntakePolicyClass
 File Name: IntakePolicy.java
 Summary: Contains constant subclasses and variables for commands, subsystems, and utility methods
 Project: BroncBotzFRC2023
 Copyright (c) BroncBotz.
 All rights reserved.

 Author(s): Shruti Venkat and Samuel Zhao
 \********************************************************************/

package frc.robot.subsystems.index;

import frc.robot.Constants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class IndexPolicy
{

  /**
   * The power to supply to the index, may not be set if the index is full unless the override is set.
   */
  public static double indexPower;

  /**
   * Override stopping the index when the pressure pad is triggered.
   */
  public static boolean indexOverride = false;

  /**
   * Constantly updates if the pressure pad is currently set indicating a ball is in the upper position.
   */
  public static boolean pressurePadSet = false;

  /**
   * Whether the index is full.
   *
   * @return bool where true = ball present in upper position, false = no ball in upper position.
   */
  public static boolean indexFull()
  {
    return pressurePadSet;
  }

  /**
   * The power supplied to the index, will be 0 if the index is full and the override is not set.
   *
   * @return double representing the percentage of power to supply to the index motor.
   */
  public static double getIndexPower()
  {
    if (indexFull() && !IndexPolicy.indexOverride)
    {
      IndexPolicy.indexPower = 0;
    }
    return Constants.clamp(IndexPolicy.indexPower, -1, 1);
  }
}
