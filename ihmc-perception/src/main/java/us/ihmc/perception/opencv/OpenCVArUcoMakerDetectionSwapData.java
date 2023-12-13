package us.ihmc.perception.opencv;

import gnu.trove.map.hash.TIntIntHashMap;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.MatVector;
import org.bytedeco.opencv.opencv_objdetect.DetectorParameters;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.perception.BytedecoImage;

public class OpenCVArUcoMakerDetectionSwapData
{
   private BytedecoImage rgb8ImageForDetection;
   private final MatVector corners;
   private final Mat ids;
   private final MatVector rejectedImagePoints;
   private final DetectorParameters detectorParameters;
   private final Stopwatch stopwatch = new Stopwatch().start();
   private final TIntIntHashMap markerIDToCornersIndexMap = new TIntIntHashMap();
   private boolean hasDetected = false;

   public OpenCVArUcoMakerDetectionSwapData()
   {
      corners = new MatVector();
      ids = new Mat();
      rejectedImagePoints = new MatVector();
      detectorParameters = new DetectorParameters();
   }

   public void ensureImageInitialized(int imageWidth, int imageHeight)
   {
      if (rgb8ImageForDetection == null)
      {
         rgb8ImageForDetection = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_8UC3);
      }
   }

   public BytedecoImage getRgb8ImageForDetection()
   {
      return rgb8ImageForDetection;
   }

   public MatVector getCorners()
   {
      return corners;
   }

   public Mat getIds()
   {
      return ids;
   }

   public MatVector getRejectedImagePoints()
   {
      return rejectedImagePoints;
   }

   public TIntIntHashMap getMarkerIDToCornersIndexMap()
   {
      return markerIDToCornersIndexMap;
   }

   public DetectorParameters getDetectorParameters()
   {
      return detectorParameters;
   }

   public Stopwatch getStopwatch()
   {
      return stopwatch;
   }

   public void setHasDetected(boolean hasDetected)
   {
      this.hasDetected = hasDetected;
   }

   public boolean getHasDetected()
   {
      return hasDetected;
   }
}
