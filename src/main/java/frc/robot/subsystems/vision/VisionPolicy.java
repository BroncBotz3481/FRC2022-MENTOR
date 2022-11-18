package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Vision policy class containing up to date data on the red and blue balls visible by the camera.
 */
public class VisionPolicy
{

  /**
   * Red balls that are detected based off contours.
   */
  public static int redBalls  = 0;
  /**
   * Blue balls detected based off contours.
   */
  public static int blueBalls = 0;

  /**
   * Find out if there are balls found depending on the alliance color.
   *
   * @return if there are one or more balls found.
   */
  public static boolean ballFound()
  {
    if (DriverStation.getAlliance() == DriverStation.Alliance.Blue)
    {
      System.out.println(
          "WARNING: The intake rollers are blue so this will detect them as well as blue balls and bumpers");
      return VisionPolicy.blueBalls > 0;
    }
    if (VisionPolicy.redBalls > 0 && VisionPolicy.blueBalls > 0)
    {
      System.out.println("WARNING: MULTIPLE BALLS FOUND, MIGHT BE INTAKING THE WRONG COLOR!");
    }
    return VisionPolicy.redBalls > 0;
  }


}
