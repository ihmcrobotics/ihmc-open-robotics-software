package us.ihmc.ihmcPerception.vision.shapes;

import boofcv.gui.image.ImagePanel;
import boofcv.gui.image.ShowImages;
import org.opencv.core.*;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import us.ihmc.ihmcPerception.OpenCVTools;
import us.ihmc.ihmcPerception.vision.HSVValue;
import us.ihmc.tools.nativelibraries.NativeLibraryLoader;

import javax.vecmath.Point2d;
import java.awt.image.BufferedImage;
import java.util.*;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class OpenCVColoredCircularBlobDetector
{
   static
   {
      NativeLibraryLoader.loadLibrary("org.opencv", OpenCVTools.OPEN_CV_LIBRARY_NAME);
   }

   private final String videoFileName;
   private final int cameraIndex;
   private final CaptureSource captureSource;

   private final VideoCapture videoCapture;
   private final Map<HSVRange, Mat> rangeOutputMaterials = new HashMap<>();

   private final Mat tmpMat = new Mat();
   private final Mat currentCameraFrameMatInBGR = new Mat();
   private final Mat currentCameraFrameMatInHSV = new Mat();
   private final Mat medianBlurredMat = new Mat();
   private final Mat houghCirclesOutputMat = new Mat();

   private final ArrayList<HoughCircleResult> circles = new ArrayList<>();

   public enum CaptureSource
   {
      CAMERA, FILE, JAVA_BUFFERED_IMAGES
   }

   /* package private */ OpenCVColoredCircularBlobDetector(String videoFileName, int cameraIndex, CaptureSource captureSource)
   {
      this.videoFileName = videoFileName;
      this.cameraIndex = cameraIndex;
      this.captureSource = captureSource;

      switch (captureSource)
      {
      case CAMERA:
         videoCapture = new VideoCapture(cameraIndex);
         break;
      case FILE:
         videoCapture = new VideoCapture(videoFileName);
         break;
      default:
         videoCapture = null;
         break;
      }
   }

   public void addHSVRange(HSVRange range)
   {
      if(!rangeOutputMaterials.containsKey(range))
      {
         rangeOutputMaterials.put(range, new Mat());
      }
   }

   public ArrayList<HoughCircleResult> getCircles()
   {
      return circles;
   }

   public Mat getCurrentCameraFrameMatInBGR()
   {
      return currentCameraFrameMatInBGR;
   }

   public void updateFromVideo()
   {
      switch (captureSource)
      {
      case CAMERA:
      case FILE:

         boolean updateSuccessful = updateCurrentFrameMaterialFromVideoCapture();

         if (!updateSuccessful)
         {
            break;
         }

         updateListOfCircles();

         break;
      default:
         throw new RuntimeException("Cannot process video stream when in CaptureSource.JAVA_BUFFERED_IMAGES mode!");
      }
   }

   public void updateFromBufferedImage(BufferedImage bufferedImage)
   {
      switch (captureSource)
      {
      case JAVA_BUFFERED_IMAGES:
         OpenCVTools.convertBufferedImageToMat(bufferedImage).copyTo(currentCameraFrameMatInBGR);

         updateListOfCircles();
         break;
      default:
         throw new RuntimeException("Cannot process BufferedImage when capturing from video stream or video file!");
      }
   }

   private boolean updateCurrentFrameMaterialFromVideoCapture()
   {
      if(videoCapture == null || !videoCapture.isOpened())
      {
         System.err.println("Video capture stream isn't open! Cannot update OpenCV Material for processing!");
         return false;
      }

      boolean readSuccessful = videoCapture.read(tmpMat);
      if(!readSuccessful)
      {
         System.err.println("Reading from video capture stream failed, cannot update OpenCV Material for processing!");
         return false;
      }

      tmpMat.copyTo(currentCameraFrameMatInBGR);
      return true;
   }

   private void updateListOfCircles()
   {
      Imgproc.cvtColor(currentCameraFrameMatInBGR, currentCameraFrameMatInHSV, Imgproc.COLOR_BGR2HSV);
      Imgproc.medianBlur(currentCameraFrameMatInHSV, medianBlurredMat, 5);

      if(rangeOutputMaterials.isEmpty())
      {
         System.err.println("No HSV ranges to check against. Set at least 1 range with addHSVRange()");
         return;
      }

      int count = 0;
      for (Map.Entry<HSVRange, Mat> hsvRangeMatEntry : rangeOutputMaterials.entrySet())
      {
         HSVRange hsvRange = hsvRangeMatEntry.getKey();
         Mat mat = hsvRangeMatEntry.getValue();

         Core.inRange(medianBlurredMat, hsvRange.getLowerBoundOpenCVScalar(), hsvRange.getUpperBoundOpenCVScalar(), mat);
         if(count == 0)
         {
            mat.copyTo(tmpMat);
         }
         else
         {
            Core.addWeighted(tmpMat, 1.0, mat, 1.0, 0.0, tmpMat);
         }
         count++;
      }

      Imgproc.GaussianBlur(tmpMat, tmpMat, new Size(13, 13), 2, 2);

      Imgproc.HoughCircles(tmpMat, houghCirclesOutputMat, Imgproc.CV_HOUGH_GRADIENT, 1, tmpMat.rows() / 8, 100, 15, 0, 0);

      circles.clear();
      for (int i = 0; i < houghCirclesOutputMat.cols(); i++)
      {
         double[] circleData = houghCirclesOutputMat.get(0, i);
         Point2d circleCenter = new Point2d(circleData[0], circleData[1]);
         double circleRadius = circleData[2];

         circles.add(new HoughCircleResult(circleCenter, circleRadius));
      }
   }

   public static void main(String[] args)
   {
      OpenCVColoredCircularBlobDetectorFactory factory = new OpenCVColoredCircularBlobDetectorFactory();

      factory.setCameraIndex(0);
      factory.setCaptureSource(CaptureSource.CAMERA);
      OpenCVColoredCircularBlobDetector openCVColoredCircularBlobDetector = factory.buildBlobDetector();

      HSVRange greenRange = new HSVRange(new HSVValue(78, 100, 100), new HSVValue(83, 255, 255));

      openCVColoredCircularBlobDetector.addHSVRange(greenRange);

      openCVColoredCircularBlobDetector.updateFromVideo();

      ImagePanel imagePanel = ShowImages.showWindow(OpenCVTools.convertMatToBufferedImage(openCVColoredCircularBlobDetector.getCurrentCameraFrameMatInBGR()), "Circle Detector");

      Scalar circleColor = new Scalar(160, 0, 0);

      while (true)
      {
         openCVColoredCircularBlobDetector.updateFromVideo();

         ArrayList<HoughCircleResult> circles = openCVColoredCircularBlobDetector.getCircles();

         for (HoughCircleResult circle : circles)
         {
            Point openCVPoint = new Point(circle.getCenter().x, circle.getCenter().y);
            Imgproc.circle(openCVColoredCircularBlobDetector.getCurrentCameraFrameMatInBGR(), openCVPoint, (int) circle.getRadius(), circleColor, 1);
         }

         imagePanel.setBufferedImage(OpenCVTools.convertMatToBufferedImage(openCVColoredCircularBlobDetector.getCurrentCameraFrameMatInBGR()));
      }
   }
}
