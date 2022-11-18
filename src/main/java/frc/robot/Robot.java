// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot
{

  private Command autonomousCommand;

  private RobotContainer robotContainer;

  /**
   * This method is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit()
  {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
  }


  /**
   * This method is called every robot packet, no matter the mode. Use this for items like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic methods, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic()
  {

    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }


  /**
   * This method is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit()
  {
  }

  /**
   * This runs every 20ms when disabled assuming there is no extraneous load on the robot.
   */
  @Override
  public void disabledPeriodic()
  {
  }


  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit()
  {
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null)
    {
      autonomousCommand.schedule();
    }
  }


  /**
   * This method is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic()
  {
  }


  /**
   * This runs when "TeleOperated" mode is initialized.
   */
  @Override
  public void teleopInit()
  {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null)
    {
      autonomousCommand.cancel();
    }
    robotContainer.setTeleopDefaultCommands();
  }


  /**
   * This method is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic()
  {
  }

  /*
  SwerveModule<CANSparkMax, CANSparkMax, CANCoder> swerve1;
  NetworkTableEntry                                swerve1Angle;
  NetworkTableEntry                                swerve1Speed;
  */

  /**
   * This code runs when the "Test" mode is initialized.
   */
  @Override
  public void testInit()
  {
    /*
    swerve1Angle = Shuffleboard.getTab("Testing").add("Angle", 0)
                               .withWidget(BuiltInWidgets.kDial)
                               .withProperties(Map.of("min", -180, "max", 180))
                               .getEntry();
    swerve1Speed = Shuffleboard.getTab("Testing").add("Speed", 0)
                               .withWidget(BuiltInWidgets.kNumberSlider)
                               .withProperties(Map.of("min", -1, "max", 1))
                               .getEntry();
    */
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    /*
    CANSparkMax driveMotorSwerve1 = new CANSparkMax(4, MotorType.kBrushless);
    CANSparkMax steerMotorSwerve1 = new CANSparkMax(5, MotorType.kBrushless);
    CANCoder    canCoderSwerve1   = new CANCoder(6);
    try
    {
      swerve1 = new SwerveModule<>(driveMotorSwerve1,
                                   steerMotorSwerve1, canCoderSwerve1,
                                   SwerveModuleLocation.FrontRight,
                                   8.14);
      swerve1.setPIDF(0.5, 0, 0, 0.362, 300, SwerveModuleMotorType.SPIN);
      swerve1.setInverted(true);
      swerve1.stopMotor();
      swerve1.setAngle(90);
    } catch (Exception e)
    {
      System.err.println("CANNOT INITIALIZE THE SWERVE MODULE!");
    }
    */
  }


  /**
   * This method is called periodically during test mode.
   */
  @Override
  public void testPeriodic()
  {
    /*
    swerve1.setAngle(swerve1Angle.getDouble(0));
    swerve1.set(swerve1Speed.getDouble(0));
    */
  }
}
