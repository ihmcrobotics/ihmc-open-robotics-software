package us.ihmc.ihmcPerception.vision.shapes;

import us.ihmc.commons.PrintTools;
import us.ihmc.ihmcPerception.OpenCVTools;
import us.ihmc.tools.nativelibraries.NativeLibraryLoader;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class OpenCVColoredCircularBlobDetectorFactory
{
   static
   {
      try
      {
         NativeLibraryLoader.loadLibrary("org.opencv", OpenCVTools.OPEN_CV_LIBRARY_NAME);
      }
      catch (UnsatisfiedLinkError e)
      {
         PrintTools.error("Failed to load the OpenCV library.");
      }
   }

   private String videoFileName = null;
   private int cameraIndex = -1;
   private OpenCVColoredCircularBlobDetector.CaptureSource captureSource = null;

   public OpenCVColoredCircularBlobDetector buildBlobDetector()
   {
      if(captureSource == OpenCVColoredCircularBlobDetector.CaptureSource.CAMERA && cameraIndex == -1)
      {
         throw new RuntimeException("Cannot create detector for camera capture w/out a proper camera index (index >= 0)! Camera index: " + cameraIndex);
      }

      if(captureSource == OpenCVColoredCircularBlobDetector.CaptureSource.FILE && videoFileName == null)
      {
         throw new RuntimeException("Cannot create detector for video file processing, file name is null!");
      }

      return new OpenCVColoredCircularBlobDetector(videoFileName, cameraIndex, captureSource);
   }

   public String getVideoFileName()
   {
      return videoFileName;
   }

   public OpenCVColoredCircularBlobDetectorFactory setVideoFileName(String videoFileName)
   {
      this.videoFileName = videoFileName;
      return this;
   }

   public int getCameraIndex()
   {
      return cameraIndex;
   }

   public OpenCVColoredCircularBlobDetectorFactory setCameraIndex(int cameraIndex)
   {
      this.cameraIndex = cameraIndex;
      return this;
   }

   public OpenCVColoredCircularBlobDetector.CaptureSource getCaptureSource()
   {
      return captureSource;
   }

   public OpenCVColoredCircularBlobDetectorFactory setCaptureSource(OpenCVColoredCircularBlobDetector.CaptureSource captureSource)
   {
      this.captureSource = captureSource;
      return this;
   }
}
