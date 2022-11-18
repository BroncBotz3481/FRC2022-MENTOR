package frc.robot.subsystems.vision;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.opencv.core.Mat;

public class VisionSubsystem extends SubsystemBase
{

  /**
   * The Singleton instance of this VisionSubsystem. Code should use the {@link #getInstance()} method to get the single
   * instance (rather than trying to construct an instance of this class.)
   */
  private final static VisionSubsystem  INSTANCE         = new VisionSubsystem();
  /**
   * GRIP pipeline generated to detect both red and blue balls.
   */
  public final         BallGripPipeline ballGripPipeline = new BallGripPipeline();
  // With eager singleton initialization, any static variables/fields used in the
  // constructor must appear before the "INSTANCE" variable so that they are initialized
  // before the constructor is called when the "INSTANCE" variable initializes.
  public               UsbCamera        camera;
  /**
   * Stream to upload what the redBall filter looks like.
   */
  // Create a new stream for sending our output and not being blind.
  public               CvSource         redBallStream    = CameraServer.putVideo("RedBallCam", 640,
                                                                                 480);
  /**
   * Stream to upload what the blueBall filter looks like.
   */
  public               CvSource         blueBallStream   = CameraServer.putVideo("BlueBallCam", 640,
                                                                                 480);
  /**
   * Matrix used to store the image.
   */
  public               Mat              mat;
  /**
   * Camera Video to pull frames from.
   */
  public               CvSink           computerVisionSink;

  /**
   * Creates a new instance of this VisionSubsystem. This constructor is private since this class is a Singleton. Code
   * should use the {@link #getInstance()} method to get the singleton instance.
   */
  private VisionSubsystem()
  {
    mat = new Mat();
    camera = CameraServer.startAutomaticCapture();
    camera.setExposureHoldCurrent();
    computerVisionSink = CameraServer.getVideo();
    // Adds custom periodic function for vision processing running every millisecond.
    // Lelz, probs too fast for the poor processor.
  }

  /**
   * Returns the Singleton instance of this VisionSubsystem. This static method should be used, rather than the
   * constructor, to get the single instance of this class. For example: {@code VisionSubsystem.getInstance();}
   */
  @SuppressWarnings("WeakerAccess")
  public static VisionSubsystem getInstance()
  {
    return INSTANCE;
  }

  /**
   * Process only red balls from the camera feed.
   */
  public void processRedBalls()
  {
    try
    {

      if (computerVisionSink.grabFrame(mat) == 0)
      {
        redBallStream.notifyError(computerVisionSink.getError());
        return;
      }
      ballGripPipeline.process(mat, Color.kRed);
      redBallStream.putFrame(mat);
      VisionPolicy.redBalls = ballGripPipeline.filterContoursRedOutput().size();
    } catch (Exception e)
    {
      System.out.println("ERROR: Vision Processing Failure, possibly no camera found.");
      System.out.println(e);
    }
  }

  /**
   * Process blue balls from the Camera feed.
   */
  public void processBlueBalls()
  {
    try
    {
      if (computerVisionSink.grabFrame(mat) == 0)
      {
        blueBallStream.notifyError(computerVisionSink.getError());
        return;
      }
      ballGripPipeline.process(mat, Color.kBlue);
      blueBallStream.putFrame(mat);
      VisionPolicy.blueBalls = ballGripPipeline.filterContoursBlueOutput().size();
    } catch (Exception e)
    {
      System.out.println("ERROR: Vision Processing Failure, possibly no camera found.");
      System.out.println(e);
    }
  }

}

