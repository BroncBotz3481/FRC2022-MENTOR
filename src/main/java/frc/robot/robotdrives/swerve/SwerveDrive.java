package frc.robot.robotdrives.swerve;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.io.Closeable;

public class SwerveDrive<DriveMotorType extends MotorController, SteeringMotorType extends MotorController>
    extends RobotDriveBase implements Sendable, AutoCloseable
{


  /**
   * Count of SwerveModule instances created.
   */
  private static int instances;

  /**
   * Front left swerve drive
   */
  private final SwerveModule<DriveMotorType, SteeringMotorType, CANCoder> m_frontLeft;
  /**
   * Back left swerve drive
   */
  private final SwerveModule<DriveMotorType, SteeringMotorType, CANCoder> m_backLeft;
  /**
   * Front right swerve drive
   */
  private final SwerveModule<DriveMotorType, SteeringMotorType, CANCoder> m_frontRight;
  /**
   * Back right swerve drive
   */
  private final SwerveModule<DriveMotorType, SteeringMotorType, CANCoder> m_backRight;
  private final SwerveDriveKinematics                                     swerveKinematics;
  private final SwerveDriveOdometry                                       swerveOdometry;
  private final WPI_Pigeon2                                               pigeonIMU;
  private final Field2d                                                   field       = new Field2d();
  /**
   * Maximum speed in meters per second.
   */
  public        int                                                       maxSpeedMPS = 5;

  /**
   * Constructor for Swerve Drive assuming modules have been created and configured with PIDF and conversions.
   *
   * @param frontLeft               Front left swerve module configured.
   * @param backLeft                Back left swerve module.
   * @param frontRight              Front right swerve module.
   * @param backRight               Back right swerve moduule
   * @param pigeon                  Pigeon IMU.
   * @param maxSpeedMetersPerSecond Maximum speed for all modules to follow.
   */
  public SwerveDrive(SwerveModule<DriveMotorType, SteeringMotorType, CANCoder> frontLeft,
                     SwerveModule<DriveMotorType, SteeringMotorType, CANCoder> backLeft,
                     SwerveModule<DriveMotorType, SteeringMotorType, CANCoder> frontRight,
                     SwerveModule<DriveMotorType, SteeringMotorType, CANCoder> backRight, WPI_Pigeon2 pigeon,
                     int maxSpeedMetersPerSecond)
  {
    instances++;
    m_frontLeft = frontLeft;
    m_backRight = backRight;
    m_backLeft = backLeft;
    m_frontRight = frontRight;
    swerveKinematics = new SwerveDriveKinematics(frontLeft.swerveModuleLocation,
                                                 frontRight.swerveModuleLocation,
                                                 backLeft.swerveModuleLocation,
                                                 backRight.swerveModuleLocation);
    pigeonIMU = pigeon;
    swerveOdometry = new SwerveDriveOdometry(swerveKinematics, getRotation());
    maxSpeedMPS = maxSpeedMetersPerSecond;

    configurePigeonIMU();

    // Inspired by https://github.com/Team364/BaseFalconSwerve/blob/main/src/main/java/frc/robot/subsystems/Swerve.java
  }

  /**
   * Configure the PigeonIMU with factory default settings and a zeroed gyroscope.
   */
  public void configurePigeonIMU()
  {
    pigeonIMU.configFactoryDefault();
    zeroGyro();
  }

  /**
   * Update the swerve drive odometry.
   *
   * @return Swerve drive odometry.
   */
  public SwerveDriveOdometry update()
  {
    swerveOdometry.update(getRotation(),
                          m_frontLeft.getState(),
                          m_frontRight.getState(),
                          m_backLeft.getState(),
                          m_backRight.getState());
    return swerveOdometry;
  }


  /**
   * Swerve drive function
   *
   * @param x               x meters per second
   * @param y               y meters per second
   * @param radianPerSecond radians per second
   * @param fieldRelative   field relative
   */
  public void drive(double x, double y, double radianPerSecond, boolean fieldRelative)
  {

    SwerveModuleState[] moduleStates = swerveKinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(x, y, radianPerSecond, getRotation())
                      : new ChassisSpeeds(x, y, radianPerSecond));
    try
    {
      setModuleStates(moduleStates);
    } catch (Exception e)
    {
      System.err.println("Cannot set swerve module states!");
    }

    try
    {
      this.update();
      field.setRobotPose(swerveOdometry.getPoseMeters());
    } catch (Exception e)
    {
      System.err.println("Cannot update SwerveDrive Odometry!");
    }

  }

  /**
   * Set the swerve module states given an array of states. Normalize the wheel speeds to abide by maximum supplied
   *
   * @param states Module states in a specified order. [front left, front right, back left, back right]
   * @throws RuntimeException If the CANCoder is inaccessible or not configured.
   */
  public void setModuleStates(SwerveModuleState[] states)
  {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, maxSpeedMPS);
    m_frontLeft.setState(states[0]);
    m_frontRight.setState(states[1]);
    m_backLeft.setState(states[2]);
    m_backRight.setState(states[3]);
  }

  /**
   * Get the current robot rotation.
   *
   * @return {@link Rotation2d} of the robot.
   */
  public Rotation2d getRotation()
  {
    return new Rotation2d(pigeonIMU.getYaw());
  }

  /**
   * Get the current Pose, used in autonomous.
   *
   * @return Current pose based off odometry.
   */
  public Pose2d getPose()
  {
    return swerveOdometry.getPoseMeters();
  }

  /**
   * Reset the odometry given the position and using current rotation from the PigeonIMU 2.
   *
   * @param pose Current position on the field.
   */
  public void resetOdometry(Pose2d pose)
  {
    swerveOdometry.resetPosition(pose, getRotation());
  }

  /**
   * Set the current rotation of the gyroscope (pigeonIMU 2) to 0.
   */
  public void zeroGyro()
  {
    pigeonIMU.setYaw(0);
  }

  /**
   * Stop all running and turning motors.
   */
  @Override
  public void stopMotor()
  {
    m_frontRight.stopMotor();
    m_backLeft.stopMotor();
    m_frontLeft.stopMotor();
    m_backRight.stopMotor();
  }

  /**
   * Get the description of the robot drive base.
   *
   * @return string of the RobotDriveBase
   */
  @Override
  public String getDescription()
  {
    return "Swerve Drive Base";
  }


  /**
   * Initializes this {@link Sendable} object.
   *
   * @param builder sendable builder
   */
  @Override
  public void initSendable(SendableBuilder builder)
  {
    builder.setSmartDashboardType("SwerveDrive");
    SendableRegistry.addChild(this, m_frontLeft);
    SendableRegistry.addChild(this, m_frontRight);
    SendableRegistry.addChild(this, m_backLeft);
    SendableRegistry.addChild(this, m_backRight);
    SendableRegistry.addChild(this, field);
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
}
