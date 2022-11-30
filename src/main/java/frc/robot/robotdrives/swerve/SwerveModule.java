package frc.robot.robotdrives.swerve;

import static java.util.Objects.requireNonNull;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.robot.Constants.DriveTrain;
import java.io.Closeable;


/**
 * Swerve module for representing a single swerve module of the robot.
 *
 * @param <DriveMotorType>      Main motor type that drives the wheel.
 * @param <AngleMotorType>      Motor that controls the angle of the wheel.
 * @param <AbsoluteEncoderType> Absolute encoder for the swerve drive module.
 */
public class SwerveModule<DriveMotorType extends MotorController, AngleMotorType extends MotorController,
    AbsoluteEncoderType extends CANCoder>
    implements MotorController, Sendable, AutoCloseable
{

  /**
   * Swerve Module location object relative to the center of the robot.
   */
  public final Translation2d swerveModuleLocation;
  /**
   * The drive gear ratio that is used during configuration of the off-board encoders in the motor controllers.
   */
  public       double        driveGearRatio = 1;
  /**
   * Angle offset of the CANCoder at initialization.
   */
  public       double        angleOffset    = 0;

  /**
   * Motor Controllers for drive motor of the swerve module.
   */
  private final DriveMotorType        m_driveMotor;
  /***
   * Motor Controller for the spin motor of the swerve drive module.
   */
  private final AngleMotorType        m_spinMotor;
  private final SwerveModuleLocation  swerveLocation;
  /**
   * Absolute encoder for the swerve module.
   */
  private final AbsoluteEncoderType   absoluteEncoder;
  /**
   * Inverted drive motor.
   */
  private       boolean               inverted   = false;
  /**
   * Power to drive motor from -1 to 1.
   */
  private       double                drivePower = 0;
  private       SparkMaxPIDController m_drivePIDController;
  private       SparkMaxPIDController m_spinPIDContrller;

  /**
   * Swerve module constructor. Both motors <b>MUST</b> be a {@link MotorController} class. It is recommended to create
   * a command to reset the encoders when triggered and
   *
   * @param mainMotor      Main drive motor. Must be a {@link MotorController} type.
   * @param angleMotor     Angle motor for controlling the angle of the swerve module.
   * @param encoder        Absolute encoder for the swerve module.
   * @param gearRatio      Drive gear ratio to get the encoder ticks per rotation.
   * @param swervePosition Swerve Module position on the robot.
   * @param steeringOffset The current offset of the absolute encoder from 0.
   * @throws Exception if an assertion fails.
   */
  public SwerveModule(DriveMotorType mainMotor, AngleMotorType angleMotor, AbsoluteEncoderType encoder,
                      SwerveModuleLocation swervePosition, double gearRatio, double steeringOffset) throws Exception
  {
    requireNonNull(mainMotor);
    requireNonNull(angleMotor);
    requireNonNull(encoder);

    m_driveMotor = mainMotor;
    m_spinMotor = angleMotor;
    swerveLocation = swervePosition;

    assert isCTREDriveMotor() || isREVDriveMotor();
    assert isCTRESpinMotor() || isREVSpinMotor();

    driveGearRatio = gearRatio;
    absoluteEncoder = encoder;
    swerveModuleLocation = getSwerveModulePosition(swervePosition);
    setAngleOffset(steeringOffset);

    if (isREVDriveMotor())
    {
      setupREVMotor(((CANSparkMax) mainMotor), SwerveModuleMotorType.DRIVE, gearRatio);
      // Might need to sleep for 200ms to 1s before this.
      ((CANSparkMax) mainMotor).burnFlash();
    } else
    {
      setupCTREMotor(((BaseTalon) mainMotor), SwerveModuleMotorType.DRIVE, gearRatio);
    }

    if (isREVSpinMotor())
    {
      // MK4 spin motor gear ratio is 12.8:1
      setupREVMotor(((CANSparkMax) angleMotor), SwerveModuleMotorType.SPIN, 12.8);
      // Might need to sleep for 200ms to 1s before this.
      ((CANSparkMax) angleMotor).burnFlash();
    } else if (encoder instanceof CANCoder)
    {
      setupCTREMotor(((BaseTalon) angleMotor), SwerveModuleMotorType.SPIN, 1);
      setupCANCoderRemoteSensor(((BaseTalon) angleMotor), encoder);
    }

    encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    // Convert CANCoder to read data as in 0 to 360
  }

  /**
   * Configure the magnetic offset in the CANCoder.
   *
   * @param offset Magnetic offset in degrees.
   * @return SwerveModule for one line configuration.
   */
  public SwerveModule setAngleOffset(double offset)
  {
    angleOffset = offset;
    absoluteEncoder.configMagnetOffset(offset);
    resetEncoders();
    return this;
  }

  /**
   * Reset the REV encoders onboard the NEO's and SparkMax's to 0, and set's the drive motor to position to 0.
   */
  public void resetEncoders()
  {
    if (isREVDriveMotor())
    {
      ((CANSparkMax) m_driveMotor).getEncoder().setPosition(0);
    }
    if (isREVSpinMotor())
    {
      ((CANSparkMax) m_spinMotor).getEncoder().setPosition(0);
    }

    // TODO: Add reset to CTRE motors, and selectively reset the spinMotor and driveMotor.

  }

  /**
   * Set the voltage compensation for the swerve module motor.
   *
   * @param nominalVoltage Nominal voltage for operation to output to.
   * @param type           Swerve Module Motor to configure.
   * @return Self for one line configuration.
   */
  public SwerveModule setVoltageCompensation(double nominalVoltage, SwerveModuleMotorType type)
  {
    if (isREVDriveMotor() || isREVSpinMotor())
    {
      ((CANSparkMax) (type == SwerveModuleMotorType.DRIVE ? m_driveMotor : m_spinMotor))
          .enableVoltageCompensation(nominalVoltage);
    }
    // TODO: Add CTRE voltage compensation.
    return this;
  }

  /**
   * Set the current limit for the swerve drive motor, remember this may cause jumping if used in conjunction with
   * voltage compensation. This is useful to protect the motor from current spikes.
   *
   * @param currentLimit Current limit in AMPS at free speed.
   * @param type         Swerve Drive Motor type to configure.
   * @return Self for one line configuration.
   */
  public SwerveModule setCurrentLimit(int currentLimit, SwerveModuleMotorType type)
  {
    if (isREVSpinMotor() || isREVDriveMotor())
    {
      ((CANSparkMax) (type == SwerveModuleMotorType.SPIN ? m_spinMotor : m_driveMotor))
          .setSmartCurrentLimit(currentLimit);
    }

    // TODO: Add CTRE current limits.
    return this;
  }

  /**
   * Setup REV motors and configure the values in the class for them. Set's the driveMotorTicksPerRotation, and
   * m_drivePIDController for the class. Assumes the absolute encoder reads from 0 to 360.
   *
   * @param motor                 Motor controller.
   * @param swerveModuleMotorType Spin motor or drive motor.
   * @param gearRatio             Gear ratio for the motor.
   */
  private void setupREVMotor(CANSparkMax motor, SwerveModuleMotorType swerveModuleMotorType, double gearRatio)
  {
    RelativeEncoder encoder = motor.getEncoder();

    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100); // Applied Output, Faults, Sticky Faults, Is Follower
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1,
                                 20); // Motor Velocity, Motor Temperature, Motor Voltage, Motor Current
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20); // Motor Position
    // TODO: Configure Status Frame 3 and 4 if necessary
    //  https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces

    motor.setIdleMode(IdleMode.kBrake);

    if (swerveModuleMotorType == SwerveModuleMotorType.DRIVE)
    {

      // motor.getEncoder().getCountsPerRevolution()
      m_drivePIDController = motor.getPIDController();
      m_drivePIDController.setFeedbackDevice(encoder);

      // Based off https://github.com/AusTINCANsProgrammingTeam/2022Swerve/blob/main/2022Swerve/src/main/java/frc/robot/Constants.java
      // Math set's the coefficient to the OUTPUT of the ENCODER (RPM == rot/min) which is the INPUT to the PID.
      // We want to set the PID to use MPS == meters/second :)
      // Dimensional Analysis
      // r/min * K = m/s
      // r/min * 1min/60s * (pi*diameter*gear)/r = m/s
      // r/min * (pi*diameter*gear)/60 = m/s
      configureSparkMax(motor, (Math.PI * DriveTrain.wheelDiameter * gearRatio) / 60, SwerveModuleMotorType.DRIVE);
    } else
    {
      m_spinPIDContrller = motor.getPIDController();
      m_spinPIDContrller.setFeedbackDevice(encoder);

      // Math set's the coefficient to the OUTPUT of the ENCODER (ticks) which is the INPUT to the PID.
      // We want to set the PID to use degrees :)
      // Dimensional Analysis
      // deg * K = ticks
      // deg * (360deg/(42*gearRatio)ticks) = ticks
      // K = 360/(42*gearRatio)
      configureSparkMax(motor, 360 / (42 * gearRatio), SwerveModuleMotorType.SPIN);
      setPIDF(1, 0, 0.1, 0, 100, SwerveModuleMotorType.SPIN);
    }

  }

  /**
   * Get the swerve module position in {@link Translation2d} from the enum passed.
   *
   * @param swerveLocation Swerve module location enum.
   * @return Location as {@link Translation2d}.
   * @throws Exception If Enum value is not defined.
   */
  private Translation2d getSwerveModulePosition(SwerveModuleLocation swerveLocation) throws Exception
  {
    // Modeling off of https://github.com/Stampede3630/2022-Code/blob/master/src/main/java/frc/robot/SwerveDrive.java
    switch (swerveLocation)
    {
      case FrontLeft:
        return new Translation2d(DriveTrain.wheelBase / 2, DriveTrain.driveTrainWidth / 2);
      case BackLeft:
        return new Translation2d(-DriveTrain.wheelBase / 2, DriveTrain.driveTrainWidth / 2);
      case FrontRight:
        return new Translation2d(DriveTrain.wheelBase / 2, -DriveTrain.driveTrainWidth / 2);
      case BackRight:
        return new Translation2d(-DriveTrain.wheelBase / 2, -DriveTrain.driveTrainWidth / 2);
      default:
        throw new Exception("Invalid location given");
    }
  }

  /**
   * Set the CANCoder to be the primary PID on the motor controller and configure the PID to accept inputs in degrees.
   * The talon will communicate independently of the roboRIO to fetch the current CANCoder position (which will result
   * in PID adjustments when using a CANivore).
   *
   * @param motor   Talon Motor controller to configure.
   * @param encoder CANCoder to use as the remote sensor.
   */
  private void setupCANCoderRemoteSensor(BaseTalon motor, CANCoder encoder)
  {
    motor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
    motor.configRemoteFeedbackFilter(encoder, CTRE_remoteSensor.REMOTE_SENSOR_0.ordinal());
    motor.configSelectedFeedbackCoefficient((double) 360 / 4096); // Degrees/Ticks
    // The CANCoder has 4096 ticks per 1 revolution.
  }

  /**
   * Set up the CTRE motors and configure class attributes correspondingly
   *
   * @param motor                 Motor controller to configure.
   * @param swerveModuleMotorType Motor type to configure
   * @param gearRatio             Gear ratio of the motor for one revolution.
   */
  private void setupCTREMotor(BaseTalon motor, SwerveModuleMotorType swerveModuleMotorType, double gearRatio)
  {
    if (swerveModuleMotorType == SwerveModuleMotorType.DRIVE)
    {
      // Math set's the coefficient to the OUTPUT of the ENCODER (ticks/100ms) which is the INPUT to the PID.
      // We want to set the PID to use MPS == meters/second :)
      // Dimensional analysis, solve for K
      // ticks/100ms * K = meters/second
      // ticks/100ms * 100ms/(1s=1000ms) * (pi*diameter)meters/(ticks[4096]*gearRatio)ticks = meters/second
      // ticks/100ms * 1/10 * (pi*diameter)/(ticks[4096]*gearRatio)ticks = meters/second
      // ticks/100ms * (pi*diameter)/((ticks[4096]*gearRatio)*10) = meters/second
      // K = (pi*diameter)/((ticks[4096]*gearRatio)*10)
      // TODO: Select the feedback sensor.
      motor.configSelectedFeedbackCoefficient((Math.PI * DriveTrain.wheelDiameter) / ((4096 * gearRatio) * 10));
    }
  }

  /**
   * Set's the general configuration for the SparkMax. Configures motor controller in brake mode. Set's the amperage
   * limits to the PDP fuses. Configures the conversion factor based upon which motor.
   *
   * @param motor                 motor controller to configure
   * @param conversionFactor      Conversion from RPM to MPS for drive motor, and rotations to degrees for the spin
   *                              motor.
   * @param swerveModuleMotorType Spin motor or drive motor for conversion factor setting.
   */
  private void configureSparkMax(CANSparkMax motor, double conversionFactor,
                                 SwerveModuleMotorType swerveModuleMotorType)
  {
    motor.setSmartCurrentLimit(40, 60); // Might need to remove this.
    if (swerveModuleMotorType == SwerveModuleMotorType.SPIN)
    {
      motor.getEncoder().setPositionConversionFactor(conversionFactor);
    } else
    {
      motor.getEncoder().setVelocityConversionFactor(conversionFactor);
    }

  }

  /**
   * Convert {@link SwerveModuleLocation} to {@link String} representation.
   *
   * @param swerveLocation Swerve position to convert.
   * @return {@link String} name of the {@link SwerveModuleLocation} enum.
   */
  public static String SwerveModuleLocationToString(SwerveModuleLocation swerveLocation)
  {
    switch (swerveLocation)
    {
      case FrontLeft:
        return "Front Left";
      case BackLeft:
        return "Back Left";
      case FrontRight:
        return "Front Right";
      case BackRight:
        return "Back Right";
      default:
        return "Unknown";
    }
  }

  /**
   * Set the PIDF coefficients for the closed loop PID onboard the SparkMax.
   *
   * @param P                     Proportional gain for closed loop. This is multiplied by closed loop error in sensor
   *                              units. Default is 1.0
   * @param I                     Integral gain for closed loop. This is multiplied by closed loop error in sensor units
   *                              every PID Loop.
   * @param D                     Derivative gain for closed loop. This is multiplied by derivative error (sensor units
   *                              per PID loop). Default is 0.1
   * @param F                     Feed Fwd gain for Closed loop.
   * @param integralZone          Integral Zone can be used to auto clear the integral accumulator if the sensor pos is
   *                              too far from the target. This prevents unstable oscillation if the kI is too large.
   *                              Value is in sensor units.
   * @param swerveModuleMotorType Swerve drive motor type.
   */
  private void setREVPIDF(double P, double I, double D, double F, double integralZone,
                          SwerveModuleMotorType swerveModuleMotorType)
  {
    // Example at
    // https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Velocity%20Closed%20Loop%20Control/src/main/java/frc/robot/Robot.java#L65-L71
    (swerveModuleMotorType == SwerveModuleMotorType.DRIVE ? m_drivePIDController : m_spinPIDContrller).setP(P);
    (swerveModuleMotorType == SwerveModuleMotorType.DRIVE ? m_drivePIDController : m_spinPIDContrller).setI(I);
    (swerveModuleMotorType == SwerveModuleMotorType.DRIVE ? m_drivePIDController : m_spinPIDContrller).setD(D);
    (swerveModuleMotorType == SwerveModuleMotorType.DRIVE ? m_drivePIDController : m_spinPIDContrller).setFF(F);
    (swerveModuleMotorType == SwerveModuleMotorType.DRIVE ? m_drivePIDController : m_spinPIDContrller).setIZone(
        integralZone);
    ((CANSparkMax) (swerveModuleMotorType == SwerveModuleMotorType.DRIVE ? m_driveMotor : m_spinMotor)).burnFlash();
  }


  /**
   * Set the angle using the onboard controller when working with CTRE Talons
   *
   * @param angle Angle in degrees
   */
  private void setCTREAngle(double angle)
  {
    ((BaseTalon) m_spinMotor).set(ControlMode.Position, angle);
  }

  /**
   * Set the velocity of the drive motor.
   *
   * @param velocity Velocity in meters per second.
   */
  private void setCTREDrive(double velocity)
  {
    ((BaseTalon) m_driveMotor).set(ControlMode.Velocity, velocity);
  }

  /**
   * Set the angle using the onboard controller when working with REV SparkMax's
   *
   * @param angle angle to set the motor too in degrees.
   */
  private void setREVAngle(double angle)
  {
    m_spinPIDContrller.setReference(angle, ControlType.kPosition);
  }

  /**
   * Set the REV meters per second for the drive motor.
   *
   * @param velocity Velocity in meters per second.
   */
  private void setREVDrive(double velocity)
  {
//    double rotationsPerSecond = (DriveTrain.wheelDiameter * Math.PI) * velocity;
//    double rotationsPerMinute = rotationsPerSecond / 60;
    m_drivePIDController.setReference(velocity, ControlType.kVelocity);
  }

  /**
   * Set the PIDF coefficients for the closed loop PID onboard the TalonSRX.
   *
   * @param profile               The {@link CTRE_slotIdx} to use.
   * @param P                     Proportional gain for closed loop. This is multiplied by closed loop error in sensor
   *                              units. Note the closed loop output interprets a final value of 1023 as full output. So
   *                              use a gain of '0.25' to get full output if err is 4096u (Mag Encoder 1 rotation)
   * @param I                     Integral gain for closed loop. This is multiplied by closed loop error in sensor units
   *                              every PID Loop. Note the closed loop output interprets a final value of 1023 as full
   *                              output. So use a gain of '0.00025' to get full output if err is 4096u (Mag Encoder 1
   *                              rotation) after 1000 loops
   * @param D                     Derivative gain for closed loop. This is multiplied by derivative error (sensor units
   *                              per PID loop). Note the closed loop output interprets a final value of 1023 as full
   *                              output. So use a gain of '250' to get full output if derr is 4096u per (Mag Encoder 1
   *                              rotation) per 1000 loops (typ 1 sec)
   * @param F                     Feed Fwd gain for Closed loop. See documentation for calculation details. If using
   *                              velocity, motion magic, or motion profile, use (1023 * duty-cycle /
   *                              sensor-velocity-sensor-units-per-100ms)
   * @param integralZone          Integral Zone can be used to auto clear the integral accumulator if the sensor pos is
   *                              too far from the target. This prevents unstable oscillation if the kI is too large.
   *                              Value is in sensor units. (ticks per 100ms)
   * @param swerveModuleMotorType Motor Type for swerve module.
   */
  private void setCTREPIDF(CTRE_slotIdx profile, double P, double I, double D, double F, double integralZone,
                           SwerveModuleMotorType swerveModuleMotorType)
  {
    ((BaseTalon) (swerveModuleMotorType == SwerveModuleMotorType.DRIVE ? m_driveMotor : m_spinMotor)).selectProfileSlot(
        profile.ordinal(), CTRE_pidIdx.PRIMARY_PID.ordinal());
    // More Closed-Loop Configs at
    // https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html#closed-loop-configs-per-slot-four-slots-available
    // Example at
    // https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java%20General/VelocityClosedLoop_ArbFeedForward/src/main/java/frc/robot/Robot.java
    ((BaseTalon) (swerveModuleMotorType == SwerveModuleMotorType.DRIVE ? m_driveMotor : m_spinMotor)).config_kP(
        profile.ordinal(), P);
    ((BaseTalon) (swerveModuleMotorType == SwerveModuleMotorType.DRIVE ? m_driveMotor : m_spinMotor)).config_kI(
        profile.ordinal(), I);
    ((BaseTalon) (swerveModuleMotorType == SwerveModuleMotorType.DRIVE ? m_driveMotor : m_spinMotor)).config_kD(
        profile.ordinal(), D);
    ((BaseTalon) (swerveModuleMotorType == SwerveModuleMotorType.DRIVE ? m_driveMotor : m_spinMotor)).config_kF(
        profile.ordinal(), F);

    ((BaseTalon) (swerveModuleMotorType == SwerveModuleMotorType.DRIVE ? m_driveMotor
                                                                       : m_spinMotor)).config_IntegralZone(
        profile.ordinal(), integralZone);

    // If the closed loop error is within this threshold, the motor output will be neutral. Set to 0 to disable.
    // Value is in sensor units.

    ((BaseTalon) (swerveModuleMotorType == SwerveModuleMotorType.DRIVE ? m_driveMotor
                                                                       : m_spinMotor)).configAllowableClosedloopError(
        profile.ordinal(), 0);

  }

  /**
   * Set the PIDF coefficients for the closed loop PID onboard the motor controller. Tuning the PID
   * <p>
   * <b>P</b> = .5 and increase it by .1 until oscillations occur, then decrease by .05 then .005 until oscillations
   * stop and angle is perfect or near perfect.
   * </p>
   * <p>
   * <b>I</b> = 0 tune this if your PID never quite reaches the target, after tuning <b>D</b>. Increase this by
   * <b>P</b>*.01 each time and adjust accordingly.
   * </p>
   * <p>
   * <b>D</b> = 0 tune this if the PID accelerates too fast, it will smooth the motion. Increase this by <b>P</b>*10
   * and adjust accordingly.
   * </p>
   * <p>
   * <b>F</b> = 0 tune this if the PID is being used for velocity, the <b>F</b> is multiplied by the target and added
   * to the voltage output. Increase this by 0.01 until the PIDF reaches the desired state in a fast enough manner.
   * </p>
   * Documentation for this is best described by CTRE <a
   * href="https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html#position-closed-loop-control-mode">here</a>.
   *
   * @param p                     Proportional gain for closed loop. This is multiplied by closed loop error in sensor
   *                              units.
   * @param i                     Integral gain for closed loop. This is multiplied by closed loop error in sensor units
   *                              every PID Loop.
   * @param d                     Derivative gain for closed loop. This is multiplied by derivative error (sensor units
   *                              per PID loop).
   * @param f                     Feed Fwd gain for Closed loop.
   * @param integralZone          Integral Zone can be used to auto clear the integral accumulator if the sensor pos is
   *                              too far from the target. This prevents unstable oscillation if the kI is too large.
   *                              Value is in sensor units.
   * @param swerveModuleMotorType Swerve drive motor type.
   * @return self for one line configuration.
   */
  public SwerveModule setPIDF(double p, double i, double d, double f, double integralZone,
                              SwerveModuleMotorType swerveModuleMotorType)
  {
    if (isREVSpinMotor() || isREVDriveMotor())
    {
      setREVPIDF(p, i, d, f, integralZone, swerveModuleMotorType);
    } else
    {
      setCTREPIDF(swerveModuleMotorType == SwerveModuleMotorType.DRIVE ? CTRE_slotIdx.Velocity : CTRE_slotIdx.Distance,
                  p, i, d, f, integralZone, swerveModuleMotorType);
    }
    
    return this;
  }

  // TODO: Replace with Oblog eventually.

  /**
   * Initializes this {@link Sendable} object.
   *
   * @param builder sendable builder
   */
  @Override
  public void initSendable(SendableBuilder builder)
  {
    builder.setSmartDashboardType(SwerveModuleLocationToString(swerveLocation) + " SwerveDriveModule");
    builder.setActuator(true);
    builder.setSafeState(this::stopMotor);
    if (isCTREDriveMotor())
    {
      builder.addDoubleProperty("Drive Motor Velocity MPS", ((BaseTalon) m_driveMotor)::getSelectedSensorVelocity,
                                this::setCTREDrive);

    } else
    {
      builder.addDoubleProperty("Drive Motor Velocity MPS", ((CANSparkMax) m_driveMotor).getEncoder()::getVelocity,
                                this::setREVDrive);
    }
    if (isCTRESpinMotor())
    {
      builder.addDoubleProperty("Steering Motor Angle Degrees", ((BaseTalon) m_spinMotor)::getSelectedSensorPosition,
                                this::setCTREAngle);
    } else
    {
      builder.addDoubleProperty("Steering Motor Angle Degrees", ((CANSparkMax) m_spinMotor).getEncoder()::getVelocity,
                                this::setREVAngle);
    }
  }

  /**
   * Set the angle of the swerve module.
   *
   * @param angle Angle in degrees.
   */
  public void setAngle(double angle)
  {
    try
    {
      angle = SwerveModuleState.optimize(getState(), Rotation2d.fromDegrees(angle)).angle.getDegrees();
    } catch (Exception e)
    {
      System.err.println("Could not fetch module state!");
    }

    if (isREVSpinMotor())
    {
      setREVAngle(angle);
    } else
    {
      setCTREAngle(angle);
    }
  }

  /**
   * Set the drive motor velocity in MPS.
   *
   * @param velocity Velocity in MPS.
   */
  public void setVelocity(double velocity)
  {
    if (isCTREDriveMotor())
    {
      setCTREDrive(velocity);
    } else
    {
      setREVDrive(velocity);
    }
  }

  /**
   * Common interface for setting the speed of a motor controller.
   *
   * @param speed The speed to set. Value should be between -1.0 and 1.0.
   */
  @Override
  public void set(double speed)
  {
    drivePower = speed;
    m_driveMotor.set(speed);
  }

  /**
   * Common interface for getting the current set speed of a motor controller.
   *
   * @return The current set speed. Value is between -1.0 and 1.0.
   */
  @Override
  public double get()
  {
    return drivePower;
  }

  /**
   * Returns whether the spin motor is a CTRE motor.
   *
   * @return is the spin motor a CTRE motor?
   */
  private boolean isCTRESpinMotor()
  {
    return m_spinMotor instanceof BaseMotorController;
  }

  /**
   * Returns whether the drive motor is a CTRE motor. All CTRE motors implement the {@link BaseMotorController} class.
   * We will only support the TalonSRX and TalonFX.
   *
   * @return is the drive motor a CTRE motor?
   */
  private boolean isCTREDriveMotor()
  {
    return m_driveMotor instanceof TalonFX || m_driveMotor instanceof TalonSRX;
  }

  /**
   * Returns whether the drive motor is a REV motor. The only REV Motor Controller is the SparkMax. We will not support
   * {@link PWMSparkMax}
   *
   * @return is the drive motor a SparkMax?
   */
  private boolean isREVSpinMotor()
  {
    return m_spinMotor instanceof CANSparkMax;
  }

  /**
   * Returns whether the drive motor is a CTRE motor.
   *
   * @return is the drive motor a SparkMax?
   */
  private boolean isREVDriveMotor()
  {
    return m_driveMotor instanceof CANSparkMax;
  }

  /**
   * Common interface for returning if a motor controller is in the inverted state or not.
   *
   * @return isInverted The state of the inversion true is inverted.
   */
  @Override
  public boolean getInverted()
  {
    return inverted;
  }

  /**
   * Common interface for inverting direction of a motor controller.
   *
   * @param isInverted The state of inversion, true is inverted.
   */
  @Override
  public void setInverted(boolean isInverted)
  {
    inverted = isInverted;
    m_driveMotor.setInverted(isInverted);
  }

  /**
   * Set the steering motor to be inverted.
   *
   * @param isInverted The state of inversion, true is inverted.
   */
  public void setInvertedSteering(boolean isInverted)
  {
    m_spinMotor.setInverted(isInverted);
  }

  /**
   * Disable the motor controller.
   */
  @Override
  public void disable()
  {
    stopMotor();
  }

  /**
   * Get the module state.
   *
   * @return SwerveModuleState with the encoder inputs.
   * @throws RuntimeException Exception if CANCoder doesnt exist
   */
  public SwerveModuleState getState()
  {
    double     mps = 0;
    Rotation2d angle;
    if (absoluteEncoder instanceof CANCoder)
    {
      angle = new Rotation2d(absoluteEncoder.getAbsolutePosition());
    } else
    {
      throw new RuntimeException("No CANCoder attached.");
    }
    if (isCTREDriveMotor())
    {
      mps = (((BaseTalon) m_driveMotor).getSelectedSensorVelocity());
    } else
    {
      mps = (((CANSparkMax) m_driveMotor).getEncoder().getVelocity());
    }
    return new SwerveModuleState(mps, angle);
  }

  /**
   * Set the module speed and angle based off the module state.
   *
   * @param state Module state.
   */
  public void setState(SwerveModuleState state)
  {
    state = SwerveModuleState.optimize(state, getState().angle);
    setAngle(state.angle.getDegrees());
    setVelocity(state.speedMetersPerSecond);
  }

  /**
   * Stops motor movement. Motor can be moved again by calling set without having to re-enable the motor.
   */
  @Override
  public void stopMotor()
  {
    m_driveMotor.stopMotor();
    m_spinMotor.stopMotor();
  }

  /**
   * Closes this resource, relinquishing any underlying resources. This method is invoked automatically on objects
   * managed by the {@code try}-with-resources statement.
   *
   * <p>While this interface method is declared to throw {@code
   * Exception}, implementers are <em>strongly</em> encouraged to declare concrete implementations of the {@code close}
   * method to throw more specific exceptions, or to throw no exception at all if the close operation cannot fail.
   *
   * <p> Cases where the close operation may fail require careful
   * attention by implementers. It is strongly advised to relinquish the underlying resources and to internally
   * <em>mark</em> the resource as closed, prior to throwing the exception. The {@code close} method is unlikely to be
   * invoked more than once and so this ensures that the resources are released in a timely manner. Furthermore it
   * reduces problems that could arise when the resource wraps, or is wrapped, by another resource.
   *
   * <p><em>Implementers of this interface are also strongly advised
   * to not have the {@code close} method throw {@link InterruptedException}.</em>
   * <p>
   * This exception interacts with a thread's interrupted status, and runtime misbehavior is likely to occur if an
   * {@code InterruptedException} is {@linkplain Throwable#addSuppressed suppressed}.
   * <p>
   * More generally, if it would cause problems for an exception to be suppressed, the {@code AutoCloseable.close}
   * method should not throw it.
   *
   * <p>Note that unlike the {@link Closeable#close close}
   * method of {@link Closeable}, this {@code close} method is <em>not</em> required to be idempotent.  In other words,
   * calling this {@code close} method more than once may have some visible side effect, unlike {@code Closeable.close}
   * which is required to have no effect if called more than once.
   * <p>
   * However, implementers of this interface are strongly encouraged to make their {@code close} methods idempotent.
   *
   * @throws Exception if this resource cannot be closed
   */
  @Override
  public void close() throws Exception
  {
    SendableRegistry.remove(this);
  }

  /**
   * Motor type for the swerve drive moduule
   */
  public enum SwerveModuleMotorType
  {
    /**
     * Drive Motor
     */
    DRIVE,
    /**
     * Steering Motor
     */
    SPIN
  }

  /**
   * The Talon SRX Slot profile used to configure the motor to use for the PID.
   */
  enum CTRE_slotIdx
  {
    Distance, Turning, Velocity, MotionProfile
  }

  /**
   * The TalonSRX PID to use onboard.
   */
  enum CTRE_pidIdx
  {
    PRIMARY_PID, AUXILIARY_PID, THIRD_PID, FOURTH_PID
  }

  enum CTRE_remoteSensor
  {
    REMOTE_SENSOR_0, REMOTE_SENSOR_1
  }

  /**
   * Swerve Module location on the robot.
   */
  public enum SwerveModuleLocation
  {
    /**
     * Swerve Module for the front left of the robot chassis.
     */
    FrontLeft,
    /**
     * Swerve Module for the back left of the robot chassis.
     */
    BackLeft,
    /**
     * Swerve Module for the front right of the robot chassis.
     */
    FrontRight,
    /**
     * Swerve Module for the back right of the robot chassis.
     */
    BackRight
  }
}
