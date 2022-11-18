/****************************** Header ******************************\
 Class Name: DrivetrainSubsystem extends SubsystemBase
 File Name: DrivetrainSubsystem.java
 Summary: Contains constant subclasses and variables for commands, subsystems, and utility methods
 Project: BroncBotzFRC2023
 Copyright (c) BroncBotz.
 All rights reserved.

 Author(s): Samuel Zhao and Shruti Venkat
 \********************************************************************/


package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrain;
import java.util.Map;

/**
 * Differential Drive subsystem which also records odometry.
 */
public class DrivetrainSubsystem extends SubsystemBase
{

  /**
   * Front left spark max.
   */
  private final CANSparkMax frontLeftMotor;
  /**
   * Back left spark max.
   */
  private final CANSparkMax backLeftMotor;
  /**
   * Front right spark max.
   */
  private final CANSparkMax frontRightMotor;
  /**
   * Back right spark max.
   */
  private final CANSparkMax backRightMotor;

  /**
   * Differential drive class to help control the robot.
   */
  private final DifferentialDrive driveTrain;

  /**
   * NEO Brushless Built-in Hall encoder
   */
  private final RelativeEncoder       leftEncoder;
  /**
   * NEO Brushless Built-in Hall encoder
   */
  private final RelativeEncoder       rightEncoder;
  /**
   * The PigeonIMU used to get the current robot angle.
   */
  private final WPI_Pigeon2           pigeonIMU;
  /**
   * SparkMAX PID Controllers
   */
  private final SparkMaxPIDController leftPIDController, rightPIDController;
  /**
   * Simple widgets for controlling the power scale during testing, should not be used during matches.
   */
  private final SimpleWidget powerScaleWidget, backwardsScaling;
  /**
   * Field widget to display our position on the field.
   */
  private final Field2d fieldWidget;

  /**
   * DriveTrain subsystem using a DifferentialDrive base. Initializes the SparkMax's and sends widgets to Shuffleboard.
   */
  public DrivetrainSubsystem()
  {
    frontLeftMotor = new CANSparkMax(Constants.DriveTrain.FrontLeftCANID, MotorType.kBrushless);
    backLeftMotor = new CANSparkMax(Constants.DriveTrain.BackLeftCANID, MotorType.kBrushless);
    frontRightMotor = new CANSparkMax(Constants.DriveTrain.FrontRightCANID, MotorType.kBrushless);
    backRightMotor = new CANSparkMax(Constants.DriveTrain.BackRightCANID, MotorType.kBrushless);

    backLeftMotor.follow(frontLeftMotor);//frontLeftMotor is the leader
    backRightMotor.follow(frontRightMotor);//frontRightMotor is the leader

    frontLeftMotor.setInverted(true);
    frontRightMotor.setInverted(false);

    leftEncoder = frontLeftMotor.getEncoder();
    rightEncoder = frontRightMotor.getEncoder();

    leftPIDController = frontLeftMotor.getPIDController();
    rightPIDController = frontRightMotor.getPIDController();

    driveTrain = new DifferentialDrive(frontLeftMotor, backRightMotor);

    pigeonIMU = new WPI_Pigeon2(DriveTrain.PigeonIMUCANID);

    // Initialize Odometry with state 0,0 all positions are relative to robot placement.
    DrivetrainPolicy.driveOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(pigeonIMU.getAngle()));

    this.setPIDF(0.08172, 0, 0, 0.372, 200);
    this.setOutputRange(-1, 1);

    ShuffleboardTab driveTrainTab = Shuffleboard.getTab("DriveTrain");
    powerScaleWidget = driveTrainTab.add("Power Scale", DrivetrainPolicy.forwardsPowerScale);
    powerScaleWidget.withWidget(BuiltInWidgets.kNumberSlider).withProperties(
        Map.of("min", 0, "max", 1, "blockIncrement", 0.05));

    backwardsScaling = driveTrainTab.add("Backwards Scale", DrivetrainPolicy.backwardsScale);
    backwardsScaling.withWidget(BuiltInWidgets.kNumberSlider).withProperties(
        Map.of("min", 0, "max", 1, "blockIncrement", 0.05));

    // Properties are described in the docstring of the BuiltInWidgets enum.
    // Properties are not case-sensitive or white space sensitive, so "Max" and "max" are the same thing and
    // "block increment", "blockIncrement", and "Block Increment" are the same thing!

    fieldWidget = new Field2d();
    SmartDashboard.putData("field", fieldWidget);
  }

  /**
   * Run in tank drive
   *
   * @param leftPower  Power to supply to the left side of the drive train.
   * @param rightPower Power to supply to the right side of the drive train.
   */
  public void run(double leftPower, double rightPower)
  {
    DrivetrainPolicy.powerLeft = leftPower;
    DrivetrainPolicy.powerRight = rightPower;
    DrivetrainPolicy.forwardsPowerScale = powerScaleWidget.getEntry().getDouble(DrivetrainPolicy.forwardsPowerScale);
    DrivetrainPolicy.backwardsScale = backwardsScaling.getEntry().getDouble(DrivetrainPolicy.backwardsScale);
    driveTrain.tankDrive(DrivetrainPolicy.getLeftPower(), DrivetrainPolicy.getRightPower());

  }

  /**
   * Using the motor controllers PID we set the left and right side of the robot to go a certain speed.
   *
   * @param leftSpeed  left drive train speed in meters per second.
   * @param rightSpeed right drive train speed in meters per second.
   */
  public void set(double leftSpeed, double rightSpeed)
  {
    DrivetrainPolicy.leftSpeedMPS = leftSpeed;
    DrivetrainPolicy.rightSpeedMPS = rightSpeed;
    // getLeftVelocityRPM will convert leftSpeedMPS to RPM
    leftPIDController.setReference(DrivetrainPolicy.getLeftVelocityRPM(), ControlType.kVelocity);
    rightPIDController.setReference(DrivetrainPolicy.getRightVelocityRPM(), ControlType.kVelocity);
  }

  /**
   * Set the gear ratio on both the left and right encoders, ASSUMING BOTH ARE THE SAME GEARING!!
   *
   * @param gearRatio Gear ratio to apply, AKA how many motor rotations it takes until the wheel makes a complete
   *                  rotation.
   */
  public void setGearRatio(double gearRatio)
  {
    leftEncoder.setVelocityConversionFactor(gearRatio); // Motor RPM * gearRatio[1/12]  = Wheel RPM
    // (12 motor rotations = 1 wheel rotation)
    rightEncoder.setVelocityConversionFactor(gearRatio);
    leftEncoder.setPositionConversionFactor(gearRatio); // Rotations * gearRatio[1/12] = Wheel Rotations
    // (12 motor rotations = 1 wheel rotation)
    rightEncoder.setPositionConversionFactor(gearRatio);

    burnFlashSafe(); // Safely burn the flash by waiting 1 second before burning it.
  }

  /**
   * Safely burn the flash of the motors by waiting a second out of loop using a lock.
   */
  public void burnFlashSafe()
  {
    new Thread(() -> {
      try
      {
        Thread.sleep(1000);
      } catch (InterruptedException e)
      {
        throw new RuntimeException(e);
      }

      synchronized (frontLeftMotor)
      {
        frontLeftMotor.burnFlash();
        frontRightMotor.burnFlash();
      }

    }).start();
  }

  /**
   * Set the PIDF coefficients for the closed loop PID onboard the SparkMax.
   *
   * @param P            Proportional gain for closed loop. This is multiplied by closed loop error in sensor units.
   * @param I            Integral gain for closed loop. This is multiplied by closed loop error in sensor units every
   *                     PID Loop.
   * @param D            Derivative gain for closed loop. This is multiplied by derivative error (sensor units per PID
   *                     loop).
   * @param F            Feed Fwd gain for Closed loop.
   * @param integralZone Integral Zone can be used to auto clear the integral accumulator if the sensor pos is too far
   *                     from the target. This prevents unstable oscillation if the kI is too large. Value is in sensor
   *                     units.
   */
  public void setPIDF(double P, double I, double D, double F, double integralZone)
  {
    // Example at
    // https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Velocity%20Closed%20Loop%20Control/src/main/java/frc/robot/Robot.java#L65-L71
    leftPIDController.setP(P);
    leftPIDController.setI(I);
    leftPIDController.setD(D);
    leftPIDController.setFF(F);
    leftPIDController.setIZone(integralZone);

    rightPIDController.setP(P);
    rightPIDController.setI(I);
    rightPIDController.setD(D);
    rightPIDController.setFF(F);
    rightPIDController.setIZone(integralZone);

    burnFlashSafe();
  }

  /**
   * Set the maximum output range of the left and right motors with PID, values are in power percentages.
   *
   * @param minimum Minimum power allowed to be supplied to the motor. Should not be bellow -1.
   * @param maximum Maximum power allowed to be supplied to the motor. Should not be above 1.
   */
  public void setOutputRange(double minimum, double maximum)
  {
    leftPIDController.setOutputRange(minimum, maximum);

    burnFlashSafe();
  }


  /**
   * Clear all sticky faults on the drive train motor controllers and the REV PH, PCM.
   */
  public void clearStickyFaults()
  {
    if (frontLeftMotor.clearFaults() != REVLibError.kOk)
    {
      System.out.println(
          "Could not clear sticky faults on the front left SparkMax CANID: " + DriveTrain.FrontLeftCANID);
    }
    if (frontRightMotor.clearFaults() != REVLibError.kOk)
    {
      System.out.println(
          "Could not clear sticky faults on the front right SparkMax CANID: " + DriveTrain.FrontRightCANID);
    }
    if (backLeftMotor.clearFaults() != REVLibError.kOk)
    {
      System.out.println("Could not clear sticky faults on the back left SparkMax CANID: " + DriveTrain.BackLeftCANID);
    }
    if (backRightMotor.clearFaults() != REVLibError.kOk)
    {
      System.out.println(
          "Could not clear sticky faults on the back right SparkMax CANID: " + DriveTrain.BackRightCANID);
    }

    burnFlashSafe();

//    PowerDistribution pdp = new PowerDistribution(Constants.PowerDistributionCANID, ModuleType.kRev);
//    pdp.clearStickyFaults(); // Can't check if it was successful programmatically.

//    PneumaticHub ph = new PneumaticHub(Constants.PneumaticHubCANID);
//    ph.clearStickyFaults(); // Very intuitive but not informative...
  }

  /**
   * Periodic function which updates odometry and sensor values in the DrivetrainPolicy class.
   */
  @Override
  public void periodic()
  {
    /**
     * Update encoder positions in the drive train policy class.
     */
    DrivetrainPolicy.UpdateEncoderPositions(leftEncoder.getPosition(), rightEncoder.getPosition());

    /** Convert RPM to Meters per second. **/
    DrivetrainPolicy.rightEncoderVelocity = DrivetrainPolicy.RPMtoMPS(rightEncoder.getVelocity());
    DrivetrainPolicy.leftEncoderVelocity = DrivetrainPolicy.RPMtoMPS(leftEncoder.getVelocity());

    DrivetrainPolicy.driveOdometry.update(Rotation2d.fromDegrees(pigeonIMU.getAngle()),
                                          DrivetrainPolicy.leftEncoderPositionDelta,
                                          DrivetrainPolicy.rightEncoderPositionDelta);

    /** Update our position on the field widget :3 **/
    fieldWidget.setRobotPose(DrivetrainPolicy.driveOdometry.getPoseMeters());
  }

}

