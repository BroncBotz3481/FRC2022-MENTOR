/****************************** Header ******************************\
 Class Name: IntakeSubsystem extends SubsystemBase
 File Name: IntakeSubsystem.java
 Summary: An example subsystem to use for learning and testing.
 Project: BroncBotzFRC2023
 Copyright (c) BroncBotz.
 All rights reserved.

 Author(s): Shruti Venkat and Samuel Zhao
 \********************************************************************/
package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase
{

  /**
   * The intake motor.
   */
  private final WPI_VictorSPX intakeMotor;

  /**
   * The piston which controls the intake.
   */
  private final DoubleSolenoid piston;

  /**
   * Initialize the subsystem by creating the motor controller and solenoids up, the intake motor is inverted by default
   * to suck in. Initializes the WPI_VictorSPX and Double Solenoid.
   */
  public IntakeSubsystem()
  {
    intakeMotor = new WPI_VictorSPX(Constants.IntakeMotorCANID);
    intakeMotor.setInverted(true);
    piston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
                                Constants.IntakePneumatics.ForwardChannel,
                                Constants.IntakePneumatics.ReverseChannel);
  }

  /**
   * Set the intake motor power based off percent output in the IntakePolicy.getIntakePower() function.
   *
   * @param speed Set the intake speed as a percentage.
   */
  public void roll(double speed)
  {
    IntakePolicy.intakePower = speed;
    intakeMotor.set(IntakePolicy.getIntakePower());
  }

  /**
   * Alias to suck in a ball through the roller.
   */
  public void suck()
  {
    this.roll(1);
  }

  /**
   * Alias to spit out a ball through the roller.
   */
  public void spit()
  {
    this.roll(-1);
  }

  /**
   * Set the intake piston based off of the IntakePolicy.
   */
  public void setIntakePiston()
  {
    piston.set(IntakePolicy.getIntakeSolenoid());
  }

  /**
   * Raise the Intake by setting the IntakePOlicy.
   */
  public void raise()
  {
    IntakePolicy.intakeSolenoid = IntakePolicy.IntakeSolenoidMode.DOWN;
    this.setIntakePiston();
  }

  /**
   * Set the IntakePolicy piston to drop and call setIntakePiston.
   */
  public void drop()
  {
    IntakePolicy.timeout.start();
    IntakePolicy.intakeSolenoid = IntakePolicy.IntakeSolenoidMode.UP;
    this.setIntakePiston();
  }


  /**
   * The periodic function which continuously updates the IntakePolicy based off of and with sensor data. Stops the
   * timeout timer and sets the intakeRaised attribute of IntakePolicy.
   */
  @Override
  public void periodic()
  {
    IntakePolicy.intakeRaised = (piston.get() == IntakePolicy.IntakeSolenoidMode.UP
                                 || piston.get() == Value.kOff);

    if (IntakePolicy.intakeRaised)
    {
      IntakePolicy.timeout.stop();
      IntakePolicy.timeout.reset();
    }
  }
}
  

