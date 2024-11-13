package us.ihmc.perception.opencv;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.global.opencv_objdetect;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.MatVector;
import org.bytedeco.opencv.opencv_objdetect.ArucoDetector;
import org.bytedeco.opencv.opencv_objdetect.DetectorParameters;
import org.bytedeco.opencv.opencv_objdetect.Dictionary;
import org.bytedeco.opencv.opencv_objdetect.RefineParameters;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.perception.camera.CameraIntrinsics;

/**
 * Thin wrapper over OpenCV's {@link ArucoDetector} to make it
 * easier to use in our context.
 *
 * This class should be limited to detection only and should not
 * contain any threading related functionality or functionality
 * related to computing the results.
 */
public class OpenCVArUcoMarkerDetector
{
   public static final int DEFAULT_DICTIONARY = opencv_objdetect.DICT_4X4_100;

   private final ArucoDetector arucoDetector;
   private final Dictionary dictionary;
   private final MatVector corners;
   private final Mat ids;
   private final MatVector rejectedImagePoints;
   private final DetectorParameters detectorParameters;
   private final Mat cameraMatrix;
   private final Mat distortionCoefficients;
   private final Stopwatch stopwatch = new Stopwatch();
   private Mat rgb8ImageForDetection;
   private Mat optionalSourceColorImage;

   public OpenCVArUcoMarkerDetector()
   {
      dictionary = opencv_objdetect.getPredefinedDictionary(DEFAULT_DICTIONARY);
      corners = new MatVector();
      ids = new Mat();
      rejectedImagePoints = new MatVector();
      detectorParameters = new DetectorParameters();
      // We don't need refine parameters
      arucoDetector = new ArucoDetector(dictionary, detectorParameters, (RefineParameters) null);
      cameraMatrix = new Mat(3, 3, opencv_core.CV_64F);
      distortionCoefficients = new Mat(1, 4, opencv_core.CV_32FC1);
      distortionCoefficients.ptr(0, 0).putFloat(0.0f);
      distortionCoefficients.ptr(0, 1).putFloat(0.0f);
      distortionCoefficients.ptr(0, 2).putFloat(0.0f);
      distortionCoefficients.ptr(0, 3).putFloat(0.0f);
      distortionCoefficients.ptr(0, 4).putFloat(0.0f);
   }

   public void setSourceImageForDetection(Mat optionalSourceColorImage)
   {
      this.optionalSourceColorImage = optionalSourceColorImage;
   }

   public void setCameraIntrinsics(CameraIntrinsics cameraIntrinsics)
   {
      opencv_core.setIdentity(cameraMatrix);
      cameraMatrix.ptr(0, 0).putDouble(cameraIntrinsics.getFx());
      cameraMatrix.ptr(1, 1).putDouble(cameraIntrinsics.getFy());
      cameraMatrix.ptr(0, 2).putDouble(cameraIntrinsics.getCx());
      cameraMatrix.ptr(1, 2).putDouble(cameraIntrinsics.getCy());
   }

   /**
    * Used to adjust the parameters used by the detector.
    */
   public DetectorParameters getDetectorParameters()
   {
      return detectorParameters;
   }

   public void update()
   {
      update(optionalSourceColorImage);
   }

   /** The user can pass in the image each time or set it once using {@link #setSourceImageForDetection}. */
   public void update(Mat sourceColorImage)
   {
      if (rgb8ImageForDetection == null)
      {
         rgb8ImageForDetection = new Mat(sourceColorImage.size(), opencv_core.CV_8UC3);
      }

      convertOrCopyToRGB(sourceColorImage, rgb8ImageForDetection);

      performDetection(rgb8ImageForDetection);
   }

   public static void convertOrCopyToRGB(Mat sourceColorImage, Mat rgb8ImageForDetection)
   {
      if (!OpenCVTools.dimensionsMatch(sourceColorImage, rgb8ImageForDetection))
         rgb8ImageForDetection = new Mat(sourceColorImage.size(), opencv_core.CV_8UC3);

      if (sourceColorImage.type() == opencv_core.CV_8UC4)
      {
         // ArUco library doesn't support alpha channel being in there
         opencv_imgproc.cvtColor(sourceColorImage, rgb8ImageForDetection, opencv_imgproc.COLOR_RGBA2RGB);
      }
      else
      {
         sourceColorImage.copyTo(rgb8ImageForDetection);
      }
   }

   /** For use if the user already has a valid image. */
   public void performDetection(Mat rgb8ImageForDetection)
   {
      arucoDetector.setDetectorParameters(detectorParameters);
      // detectMarkers is the big slow thing, so we put it on an async thread.
      stopwatch.start();
      arucoDetector.detectMarkers(rgb8ImageForDetection, corners, ids, rejectedImagePoints);
      stopwatch.suspend();
   }

   public Mat getRGB8ImageForDetection()
   {
      return rgb8ImageForDetection;
   }

   public MatVector getCorners()
   {
      return corners;
   }

   public Mat getIDs()
   {
      return ids;
   }

   public MatVector getRejectedImagePoints()
   {
      return rejectedImagePoints;
   }

   public Mat getCameraMatrix()
   {
      return cameraMatrix;
   }

   public Mat getDistortionCoefficients()
   {
      return distortionCoefficients;
   }

   public double getDetectionDuration()
   {
      return stopwatch.totalElapsed();
   }
}