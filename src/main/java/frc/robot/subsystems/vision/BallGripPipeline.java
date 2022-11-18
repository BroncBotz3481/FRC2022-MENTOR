package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.util.Color;
import java.util.ArrayList;
import java.util.List;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

/**
 * BallGripPipeline class.
 *
 * <p>An OpenCV pipeline generated by GRIP.
 *
 * @author GRIP
 */
public class BallGripPipeline
{

  /**
   * Main outputs of filtered contours that were identified for red balls.
   */
  private final ArrayList<MatOfPoint> filterContoursRedOutput  = new ArrayList<MatOfPoint>();
  /**
   * Main outputs of filtered contours that were identified for blue balls.
   */
  private final ArrayList<MatOfPoint> filterContoursBlueOutput = new ArrayList<MatOfPoint>();

//    static
//    {
//        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
//    }

  /**
   * This is the primary method that runs the entire pipeline and updates the outputs.
   * @param source0 Matrix source to perform processing on.
   * @param ball Ball color to care about.
   */
  public void process(Mat source0, Color ball)
  {
    // Step Blur0:
//        Mat blurInput = source0;
    BlurType blurType   = BlurType.get("Gaussian Blur");
    double   blurRadius = 5.405405405405405;
    blur(source0, blurType, blurRadius, source0);

    if (ball == Color.kRed)
    {
      // Step HSL_Threshold0:
//        Mat hslThreshold0Input = blurOutput;
      double[] hslThreshold0Hue        = {0.0, 20.273037542662117};
      double[] hslThreshold0Saturation = {98.60611510791367, 255.0};
      double[] hslThreshold0Luminance  = {41.276978417266186, 194.07849829351537};
      hslThreshold(source0, hslThreshold0Hue, hslThreshold0Saturation, hslThreshold0Luminance,
                   source0);

      // Step Find_Contours0:
//        Mat findContours0Input = hslThresholdRedOutput;
      boolean findContours0ExternalOnly = false;
      findContours(source0, findContours0ExternalOnly, filterContoursRedOutput);

      // Step Filter_Contours0:
//        ArrayList<MatOfPoint> filterContours0Contours = ;
      double   filterContours0MinArea      = 400.0;
      double   filterContours0MinPerimeter = 100.0;
      double   filterContours0MinWidth     = 100.0;
      double   filterContours0MaxWidth     = 1000.0;
      double   filterContours0MinHeight    = 100.0;
      double   filterContours0MaxHeight    = 1000.0;
      double[] filterContours0Solidity     = {0, 100};
      double   filterContours0MaxVertices  = 1000000.0;
      double   filterContours0MinVertices  = 0.0;
      double   filterContours0MinRatio     = 0.0;
      double   filterContours0MaxRatio     = 1000.0;
      filterContours(filterContoursRedOutput, filterContours0MinArea, filterContours0MinPerimeter,
                     filterContours0MinWidth, filterContours0MaxWidth, filterContours0MinHeight,
                     filterContours0MaxHeight, filterContours0Solidity, filterContours0MaxVertices,
                     filterContours0MinVertices, filterContours0MinRatio, filterContours0MaxRatio,
                     filterContoursRedOutput);
    } else if (ball == Color.kBlue)
    {
      // Step HSL_Threshold1:
//        Mat hslThreshold1Input = blurOutput;
      double[] hslThreshold1Hue        = {59.89208633093525, 110.88737201365187};
      double[] hslThreshold1Saturation = {77.96762589928058, 255.0};
      double[] hslThreshold1Luminance  = {59.62230215827338, 194.07849829351537};
      hslThreshold(source0, hslThreshold1Hue, hslThreshold1Saturation, hslThreshold1Luminance,
                   source0);

      // Step Find_Contours1:
//        Mat findContours1Input = hslThresholdBlueOutput;
      boolean findContours1ExternalOnly = false;
      findContours(source0, findContours1ExternalOnly, filterContoursBlueOutput);

      // Step Filter_Contours1:
//        ArrayList<MatOfPoint> filterContours1Contours = ;
      double   filterContours1MinArea      = 400.0;
      double   filterContours1MinPerimeter = 100.0;
      double   filterContours1MinWidth     = 100.0;
      double   filterContours1MaxWidth     = 1000;
      double   filterContours1MinHeight    = 0;
      double   filterContours1MaxHeight    = 1000;
      double[] filterContours1Solidity     = {0, 100};
      double   filterContours1MaxVertices  = 1000000;
      double   filterContours1MinVertices  = 0;
      double   filterContours1MinRatio     = 0;
      double   filterContours1MaxRatio     = 1000;
      filterContours(filterContoursBlueOutput, filterContours1MinArea, filterContours1MinPerimeter,
                     filterContours1MinWidth, filterContours1MaxWidth, filterContours1MinHeight,
                     filterContours1MaxHeight, filterContours1Solidity, filterContours1MaxVertices,
                     filterContours1MinVertices, filterContours1MinRatio, filterContours1MaxRatio,
                     filterContoursBlueOutput);
    }
  }


  /**
   * This method is a generated getter for the output of a Filter_Contours.
   *
   * @return {@link ArrayList<MatOfPoint>} output from Filter_Contours.
   */
  public ArrayList<MatOfPoint> filterContoursRedOutput()
  {
    return filterContoursRedOutput;
  }

  /**
   * This method is a generated getter for the output of a Filter_Contours.
   *
   * @return {@link ArrayList<MatOfPoint>} output from Filter_Contours.
   */
  public ArrayList<MatOfPoint> filterContoursBlueOutput()
  {
    return filterContoursBlueOutput;
  }

  /**
   * Softens an image using one of several filters.
   *
   * @param input        The image on which to perform the blur.
   * @param type         The blurType to perform.
   * @param doubleRadius The radius for the blur.
   * @param output       The image in which to store the output.
   */
  private void blur(Mat input, BlurType type, double doubleRadius,
                    Mat output)
  {
    int radius = (int) (doubleRadius + 0.5);
    int kernelSize;
    switch (type)
    {
      case BOX:
        kernelSize = 2 * radius + 1;
        Imgproc.blur(input, output, new Size(kernelSize, kernelSize));
        break;
      case GAUSSIAN:
        kernelSize = 6 * radius + 1;
        Imgproc.GaussianBlur(input, output, new Size(kernelSize, kernelSize), radius);
        break;
      case MEDIAN:
        kernelSize = 2 * radius + 1;
        Imgproc.medianBlur(input, output, kernelSize);
        break;
      case BILATERAL:
        Imgproc.bilateralFilter(input, output, -1, radius, radius);
        break;
    }
  }

  /**
   * Segment an image based on hue, saturation, and luminance ranges.
   *
   * @param input The image on which to perform the HSL threshold.
   * @param hue   The min and max hue
   * @param sat   The min and max saturation
   * @param lum   The min and max luminance //	 * @param output The image in which to store the output.
   */
  private void hslThreshold(Mat input, double[] hue, double[] sat, double[] lum,
                            Mat out)
  {
    Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2HLS);
    Core.inRange(out, new Scalar(hue[0], lum[0], sat[0]),
                 new Scalar(hue[1], lum[1], sat[1]), out);
  }

  /**
   * Sets the values of pixels in a binary image to their distance to the nearest black pixel.
   *
   * @param input The image on which to perform the Distance Transform. //	 * @param type The Transform. //	 * @param
   *              maskSize the size of the mask. //	 * @param output The image in which to store the output.
   */
  private void findContours(Mat input, boolean externalOnly,
                            List<MatOfPoint> contours)
  {
    Mat hierarchy = new Mat();
    contours.clear();
    int mode;
    if (externalOnly)
    {
      mode = Imgproc.RETR_EXTERNAL;
    } else
    {
      mode = Imgproc.RETR_LIST;
    }
    int method = Imgproc.CHAIN_APPROX_SIMPLE;
    Imgproc.findContours(input, contours, hierarchy, mode, method);
  }

  /**
   * Filters out contours that do not meet certain criteria.
   *
   * @param inputContours  is the input list of contours
   * @param output         is the the output list of contours
   * @param minArea        is the minimum area of a contour that will be kept
   * @param minPerimeter   is the minimum perimeter of a contour that will be kept
   * @param minWidth       minimum width of a contour
   * @param maxWidth       maximum width
   * @param minHeight      minimum height
   * @param maxHeight      maximimum height //	 * @param Solidity the minimum and maximum solidity of a contour
   * @param minVertexCount minimum vertex Count of the contours
   * @param maxVertexCount maximum vertex Count
   * @param minRatio       minimum ratio of width to height
   * @param maxRatio       maximum ratio of width to height
   */
  private void filterContours(List<MatOfPoint> inputContours, double minArea,
                              double minPerimeter, double minWidth, double maxWidth,
                              double minHeight, double
                                  maxHeight, double[] solidity, double maxVertexCount,
                              double minVertexCount, double
                                  minRatio, double maxRatio, List<MatOfPoint> output)
  {
    final MatOfInt hull = new MatOfInt();
    output.clear();
    //operation
    for (int i = 0; i < inputContours.size(); i++)
    {
      final MatOfPoint contour = inputContours.get(i);
      final Rect       bb      = Imgproc.boundingRect(contour);
      if (bb.width < minWidth || bb.width > maxWidth)
      {
        continue;
      }
      if (bb.height < minHeight || bb.height > maxHeight)
      {
        continue;
      }
      final double area = Imgproc.contourArea(contour);
      if (area < minArea)
      {
        continue;
      }
      if (Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true) < minPerimeter)
      {
        continue;
      }
      Imgproc.convexHull(contour, hull);
      MatOfPoint mopHull = new MatOfPoint();
      mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
      for (int j = 0; j < hull.size().height; j++)
      {
        int      index = (int) hull.get(j, 0)[0];
        double[] point = new double[]{contour.get(index, 0)[0], contour.get(index, 0)[1]};
        mopHull.put(j, 0, point);
      }
      final double solid = 100 * area / Imgproc.contourArea(mopHull);
      if (solid < solidity[0] || solid > solidity[1])
      {
        continue;
      }
      if (contour.rows() < minVertexCount || contour.rows() > maxVertexCount)
      {
        continue;
      }
      final double ratio = bb.width / (double) bb.height;
      if (ratio < minRatio || ratio > maxRatio)
      {
        continue;
      }
      output.add(contour);
    }
  }


  /**
   * An indication of which type of filter to use for a blur. Choices are BOX, GAUSSIAN, MEDIAN, and BILATERAL
   */
  enum BlurType
  {
    BOX("Box Blur"), GAUSSIAN("Gaussian Blur"), MEDIAN("Median Filter"),
    BILATERAL("Bilateral Filter");

    private final String label;

    BlurType(String label)
    {
      this.label = label;
    }

    public static BlurType get(String type)
    {
      if (BILATERAL.label.equals(type))
      {
        return BILATERAL;
      } else if (GAUSSIAN.label.equals(type))
      {
        return GAUSSIAN;
      } else if (MEDIAN.label.equals(type))
      {
        return MEDIAN;
      } else
      {
        return BOX;
      }
    }

    @Override
    public String toString()
    {
      return this.label;
    }
  }


}

