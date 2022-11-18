/****************************** Header ******************************\
 Class Name: ShooterSubsystem extends SubsystemBase
 File Name: ExampleSubsystem.java
 Summary: An example subsystem to use for learning and testing.
 Project: BroncBotzFRC2023
 Copyright (c) BroncBotz.
 All rights reserved.

 Author(s): Shruti Venkat and Samuel Zhao
 \********************************************************************/
package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase
{

  private final WPI_TalonSRX    shooterMotor;
  private final WPI_VictorSPX   secondMotor;
  private final DoubleSolenoid  hood;
  private final ShuffleboardTab shooterTab;

  public ShooterSubsystem()
  {
    hood = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.HoodPneumatics.ForwardChannel,
                              Constants.HoodPneumatics.ReverseChannel);

    // Shuffleboard Tab Setup
    shooterTab = Shuffleboard.getTab("Shooter");

    // Supply with continuously updated value of the sensor velocity given from the sensorVelocity calculation.
    shooterTab.addNumber("Encoder", () -> {
      return ShooterPolicy.shooterSensorVelocity;
    });

    shooterTab.addNumber("Velocity", () -> {
      return ShooterPolicy.shooterVelocity;
    });

    secondMotor = new WPI_VictorSPX(Constants.ShooterFollowerMotorCANID);
    shooterMotor = new WPI_TalonSRX(Constants.ShooterMotorCANID);

    secondMotor.setInverted(true);

    secondMotor.follow(shooterMotor);

    // P was gathered via SysId, we dont need I or D and the Feed Forward takes into account of acceleration
    this.setPIDF(slotIdx.Velocity, pidIdx.PRIMARY_PID, 0.08712591, 0, 0, .0365, 300);

    this.zeroEncoder();
    this.setClosedLoopPeriod(1);
    this.setStatusFrames();

    // Add the Sendable ShooterPolicy to the subsystem
    super.addChild("Policy", ShooterPolicy.self);
  }

  /**
   * Set the PIDF coefficients for the closed loop PID onboard the TalonSRX.
   *
   * @param profile      The {@link slotIdx} to use.
   * @param pid          The PID to use.
   * @param P            Proportional gain for closed loop. This is multiplied by closed loop error in sensor units.
   *                     Note the closed loop output interprets a final value of 1023 as full output. So use a gain of
   *                     '0.25' to get full output if err is 4096u (Mag Encoder 1 rotation)
   * @param I            Integral gain for closed loop. This is multiplied by closed loop error in sensor units every
   *                     PID Loop. Note the closed loop output interprets a final value of 1023 as full output. So use a
   *                     gain of '0.00025' to get full output if err is 4096u (Mag Encoder 1 rotation) after 1000 loops
   * @param D            Derivative gain for closed loop. This is multiplied by derivative error (sensor units per PID
   *                     loop). Note the closed loop output interprets a final value of 1023 as full output. So use a
   *                     gain of '250' to get full output if derr is 4096u per (Mag Encoder 1 rotation) per 1000 loops
   *                     (typ 1 sec)
   * @param F            Feed Fwd gain for Closed loop. See documentation for calculation details. If using velocity,
   *                     motion magic, or motion profile, use (1023 * duty-cycle /
   *                     sensor-velocity-sensor-units-per-100ms)
   * @param integralZone Integral Zone can be used to auto clear the integral accumulator if the sensor pos is too far
   *                     from the target. This prevents unstable oscillation if the kI is too large. Value is in sensor
   *                     units. (ticks per 100ms)
   */
  public void setPIDF(slotIdx profile, pidIdx pid, double P, double I, double D, double F, double integralZone)
  {
    shooterMotor.selectProfileSlot(profile.ordinal(), pid.ordinal());
    // More Closed-Loop Configs at
    // https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html#closed-loop-configs-per-slot-four-slots-available
    // Example at
    // https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java%20General/VelocityClosedLoop_ArbFeedForward/src/main/java/frc/robot/Robot.java
    shooterMotor.config_kP(profile.ordinal(), P);
    shooterMotor.config_kI(profile.ordinal(), I);
    shooterMotor.config_kD(profile.ordinal(), D);
    shooterMotor.config_kF(profile.ordinal(), F);

    shooterMotor.config_IntegralZone(profile.ordinal(), integralZone);
    // If the closed loop error is within this threshold, the motor output will be neutral. Set to 0 to disable.
    // Value is in sensor units.
    shooterMotor.configAllowableClosedloopError(profile.ordinal(), 0);
  }

  /**
   * How fast the INTERNAL PID IS RUNNING (DOES NOT USE CAN!).
   *
   * @param ms period in milliseconds, should be 1.
   */
  public void setClosedLoopPeriod(int ms)
  {
    shooterMotor.configClosedLoopPeriod(slotIdx.Velocity.ordinal(), ms);

  }

  /**
   * Set the max motor output.
   *
   * @param maximumOutput Absolute max motor output during closed-loop control modes only. A value of '1' represents
   *                      full output in both directions.
   */
  public void setVelocityVoltageMaximumOutput(double maximumOutput)
  {
    shooterMotor.configClosedLoopPeakOutput(slotIdx.Velocity.ordinal(), maximumOutput);
  }

  /**
   * Set the maximum RPM that the motor can increase by.
   *
   * @param maximumStepRPM Cap on the integral accumulator in sensor units. Note accumulator is multiplied by kI AFTER
   *                       this cap takes effect.
   */
  public void setVelocityMaximumStep(double maximumStepRPM)
  {
    shooterMotor.configMaxIntegralAccumulator(slotIdx.Velocity.ordinal(),
                                              ShooterPolicy.getRealVelocity(maximumStepRPM));
  }

  /**
   * Set the shooter power using percentage
   *
   * @param speed Shooter speed as a percentage.
   */
  public void setShooter(double speed)
  {
    ShooterPolicy.shooterPower = speed;
    shooterMotor.set(ControlMode.PercentOutput, ShooterPolicy.getShooterPower());
  }

  /**
   * Set the shooter to a specified velocity using the primary PID
   *
   * @param targetVelocity Target velocity to send the shooter to using the PID specified during pid setup.
   */
  public void setShooterVelocity(int targetVelocity)
  {
    ShooterPolicy.pidTarget = targetVelocity;
    shooterMotor.set(ControlMode.Velocity, ShooterPolicy.getRealVelocity(targetVelocity));
  }

  /**
   * Lower shooting hood for high shots.
   */
  public void lowerHood()
  {
    hood.set(DoubleSolenoid.Value.kForward);
  }

  /**
   * Raise shooting hood for low shots.
   */
  public void raiseHood()
  {
    hood.set(DoubleSolenoid.Value.kReverse);
  }

  /**
   * Zero's the attached encoder to the shooter motor.
   */
  public void zeroEncoder()
  {
    shooterMotor.getSensorCollection().setQuadraturePosition(0, ShooterPolicy.communicationTimeoutMs);
  }

  /**
   * Set status frames to the default period 20ms for PID 0 updates on the shooter motor.
   */
  public void setStatusFrames()
  {
    shooterMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20, ShooterPolicy.communicationTimeoutMs);
    shooterMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, ShooterPolicy.communicationTimeoutMs);
  }

  /**
   * Periodic function running every 20ms.
   */
  @Override
  public void periodic()
  {
    ShooterPolicy.shooterVelocity = ShooterPolicy.fromRealVelocity(
        shooterMotor.getSelectedSensorVelocity(pidIdx.PRIMARY_PID.ordinal()));
    ShooterPolicy.shooterSensorVelocity = shooterMotor.getSelectedSensorVelocity(pidIdx.PRIMARY_PID.ordinal());
  }

  /**
   * The TalonSRX PID to use onboard.
   */
  enum pidIdx
  {
    PRIMARY_PID, AUXILIARY_PID, THIRD_PID, FOURTH_PID
  }

  /**
   * The Talon SRX Slot profile used to configure the motor to use for the PID.
   */
  enum slotIdx
  {
    Distance, Turning, Velocity, MotionProfile
  }


}
