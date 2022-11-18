package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.ColorSensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ColorSensorResolution;
import com.revrobotics.ColorSensorV3.GainFactor;
import com.revrobotics.ColorSensorV3.LEDCurrent;
import com.revrobotics.ColorSensorV3.LEDPulseFrequency;
import com.revrobotics.ColorSensorV3.ProximitySensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ProximitySensorResolution;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.climber.ClimberPolicy.LeftColorSensor;

/**
 * Climber subsystem for the robot.
 */
public class ClimberSubsystem extends SubsystemBase
{

  /**
   * Victor SPX with WPI protections for the left motor.
   */
  private final WPI_VictorSPX        leftMotorClimb;
  /**
   * Victor SPX with WPI protections for the right motor.
   */
  private final WPI_VictorSPX        rightMotorClimb;
  /**
   * Motor Controller group for both left and right motors.
   */
  private final MotorControllerGroup climber;
  /**
   * Left color sensor.
   */
  private final ColorSensorV3        leftColor;//, rightColor;

  /**
   * Constructor for the ClimberSubsystem initializes the VictorSPX's with the CANID's from {@link Constants.Climber},
   * the color sensor with the onboard I2C, and the MotorControllerGroup.
   */
  public ClimberSubsystem()
  {
    leftMotorClimb = new WPI_VictorSPX(Constants.Climber.LeftClimberMotorCANID);
    rightMotorClimb = new WPI_VictorSPX(Constants.Climber.RightClimberMotorCANID);

    leftMotorClimb.setInverted(true);

    leftColor = new ColorSensorV3(Port.kOnboard);
//    rightColor = new ColorSensorV3(Port.kMXP); // WARNING: THE I2C ON THE MXP HAS A LOCKUP ERROR!!
    climber = new MotorControllerGroup(leftMotorClimb, rightMotorClimb);
  }

  /**
   * Manually set the power to the left and right climber motors, normally not recommended unless in extreme situation.
   *
   * @param left  Left power percentage
   * @param right Right power percentage.
   */
  public void runManual(double left, double right)
  {
    ClimberPolicy.leftPowerClimb = left;
    ClimberPolicy.rightPowerClimb = right;
    leftMotorClimb.set(ClimberPolicy.leftPowerClimb);
    rightMotorClimb.set(ClimberPolicy.rightPowerClimb);
  }

  /**
   * Run both motors at the same power, recommended to use this to prevent the climber from breaking.
   *
   * @param power Power as a percentage to supply.
   */
  public void run(double power)
  {
    ClimberPolicy.rightPowerClimb = power;
    ClimberPolicy.leftPowerClimb = power;
    ClimberPolicy.power = power;
    climber.set(ClimberPolicy.getPower());
  }

  /**
   * Configurations for the color sensor.
   */
  public void configureColorSensor()
  {
    leftColor.configureColorSensor(ColorSensorResolution.kColorSensorRes13bit,
                                   ColorSensorMeasurementRate.kColorRate25ms, GainFactor.kGain6x);
    leftColor.configureProximitySensor(ProximitySensorResolution.kProxRes8bit,
                                       ProximitySensorMeasurementRate.kProxRate100ms);
    leftColor.configureProximitySensorLED(LEDPulseFrequency.kFreq100kHz, LEDCurrent.kPulse25mA, 100);
  }

  /**
   * Continuously grab the states of the color sensors.
   */
  @Override
  public void periodic()
  {
    LeftColorSensor.red = leftColor.getRed();
    LeftColorSensor.green = leftColor.getGreen();
    LeftColorSensor.blue = leftColor.getBlue();
    LeftColorSensor.ir = leftColor.getIR();
    LeftColorSensor.proximity = leftColor.getProximity();
    LeftColorSensor.connected = leftColor.isConnected();
  }
}
