package us.ihmc.perception.opencv;

import boofcv.struct.calib.CameraPinholeBrown;
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
import us.ihmc.perception.BytedecoImage;

public class OpenCVArUcoMarkerDetection
{
   public static final int DEFAULT_DICTIONARY = opencv_objdetect.DICT_4X4_100;

   private ArucoDetector arucoDetector;
   private Dictionary dictionary;
   private BytedecoImage rgb8ImageForDetection;
   private MatVector corners;
   private Mat ids;
   private MatVector rejectedImagePoints;
   private DetectorParameters detectorParameters;
   private Mat cameraMatrix;
   private Mat distortionCoefficients;
   private BytedecoImage optionalSourceColorImage;
   private final Stopwatch stopwatch = new Stopwatch();

   public void create()
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

   public void setSourceImageForDetection(BytedecoImage optionalSourceColorImage)
   {
      this.optionalSourceColorImage = optionalSourceColorImage;
   }

   public void setCameraInstrinsics(CameraPinholeBrown depthCameraIntrinsics)
   {
      opencv_core.setIdentity(cameraMatrix);
      cameraMatrix.ptr(0, 0).putDouble(depthCameraIntrinsics.getFx());
      cameraMatrix.ptr(1, 1).putDouble(depthCameraIntrinsics.getFy());
      cameraMatrix.ptr(0, 2).putDouble(depthCameraIntrinsics.getCx());
      cameraMatrix.ptr(1, 2).putDouble(depthCameraIntrinsics.getCy());
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
   public void update(BytedecoImage sourceColorImage)
   {
      if (rgb8ImageForDetection == null)
      {
         rgb8ImageForDetection = new BytedecoImage(sourceColorImage.getImageWidth(), sourceColorImage.getImageHeight(), opencv_core.CV_8UC3);
      }

      convertOrCopyToRGB(sourceColorImage, rgb8ImageForDetection);

      performDetection(rgb8ImageForDetection);
   }

   public static void convertOrCopyToRGB(BytedecoImage sourceColorImage, BytedecoImage rgb8ImageForDetection)
   {
      rgb8ImageForDetection.ensureDimensionsMatch(sourceColorImage);

      if (sourceColorImage.getBytedecoOpenCVMat().type() == opencv_core.CV_8UC4)
      {
         // ArUco library doesn't support alpha channel being in there
         opencv_imgproc.cvtColor(sourceColorImage.getBytedecoOpenCVMat(),
                                 rgb8ImageForDetection.getBytedecoOpenCVMat(),
                                 opencv_imgproc.COLOR_RGBA2RGB);
      }
      else
      {
         sourceColorImage.getBytedecoOpenCVMat().copyTo(rgb8ImageForDetection.getBytedecoOpenCVMat());
      }
   }

   /** For use if the user already has a valid image. */
   public void performDetection(BytedecoImage rgb8ImageForDetection)
   {
      arucoDetector.setDetectorParameters(detectorParameters);
      // detectMarkers is the big slow thing, so we put it on an async thread.
      stopwatch.start();
      arucoDetector.detectMarkers(rgb8ImageForDetection.getBytedecoOpenCVMat(), corners, ids, rejectedImagePoints);
      stopwatch.suspend();
   }

   public BytedecoImage getRGB8ImageForDetection()
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