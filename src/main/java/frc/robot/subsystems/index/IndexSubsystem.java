/****************************** Header ******************************\
 Class Name: IndexSubsystem extends SubsystemBase
 File Name: IndexSubsystem.java
 Summary: Practice
 Project: BroncBotzFRC2023
 Copyright (c) BroncBotz.
 All rights reserved.

 Author(s): Shruti Venkat and Samuel Zhao
 \********************************************************************/
package frc.robot.subsystems.index;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Subsystem to control the index of balls to the shooter on the robot.
 */
public class IndexSubsystem extends SubsystemBase
{

  /**
   * Index motor
   */
  public WPI_VictorSPX indexMotor; // This is the motor controller
  /**
   * Pressure pad for balls in the upper position, value is true when less than 10.
   */
  public AnalogInput   upperPositionPressurePad;

  //public static boolean isPressed; //if pressure pad is pressed or not

  /**
   * Creates the index subsystem and initializes the index VictorSPX, and pressure pad.
   */
  public IndexSubsystem()
  {
    // Create the new motor controller (make sure you check your ID!)
    indexMotor = new WPI_VictorSPX(Constants.IndexMotorCANID);
    upperPositionPressurePad = new AnalogInput(Constants.IndexPressurePadPWMPort);
  }

  // This could be "runintake" or "stopintake" or "liftclimber"

  /**
   * Run the index at the specified power as a percentage value between -1 and 1.
   *
   * @param power Power inbetween -1 and 1.
   */
  public void run(double power)
  {
    IndexPolicy.indexPower = power;
    indexMotor.set(IndexPolicy.getIndexPower());

  }

  /**
   * Alias to reverse the index. Reverse the index for intaking until the pressure pad is hit unless an override is set.
   * Moves the balls toward the shooter.
   */
  public void reverse()
  {
    this.run(-1);
  }

  /**
   * Alias to outtake a ball. Moves the ball away from the shooter.
   */
  public void forward()
  {
    this.run(1);
  }

  /**
   * Alias to stop the index.
   */
  public void stop()
  {
    this.run(0);
  }


  /**
   * The periodic function that runs every 20ms. Continuously updates the pressure pad with if there is a ball in the
   * upper position.
   */
  @Override
  public void periodic()
  {
    IndexPolicy.pressurePadSet = upperPositionPressurePad.getValue() <= 10;
  }
}
