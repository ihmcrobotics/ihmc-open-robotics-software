package us.ihmc.perception;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_cudafilters;
import org.bytedeco.opencv.global.opencv_cudaimgproc;
import org.bytedeco.opencv.global.opencv_cudawarping;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.MatVector;
import org.bytedeco.opencv.opencv_core.Moments;
import org.bytedeco.opencv.opencv_core.Point;
import org.bytedeco.opencv.opencv_core.Point2f;
import org.bytedeco.opencv.opencv_core.RotatedRect;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.bytedeco.opencv.opencv_core.Size;
import org.bytedeco.opencv.opencv_cudafilters.Filter;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.perception.tools.PerceptionDebugTools;

import java.util.Arrays;
import java.util.Comparator;
import java.util.Optional;

public class BallDetector
{
   private final Scalar hsvLowerBound = new Scalar(0.0, 0.0, 0.0, 0.0);
   private final Scalar hsvUpperBound = new Scalar(255.0, 255.0, 255.0, 255.0);
   private final Filter gaussianFilter = opencv_cudafilters.createGaussianFilter(opencv_core.CV_8UC3, opencv_core.CV_8UC3, new Size(9, 9), 0.0);
   private final Filter openFilter = opencv_cudafilters.createMorphologyFilter(opencv_imgproc.MORPH_OPEN,
                                                                               opencv_core.CV_8UC1,
                                                                               opencv_imgproc.getStructuringElement(opencv_imgproc.MORPH_RECT,
                                                                                                                    new Size(7, 7)),
                                                                               new Point(-1, -1), // put anchor in center of kernel
                                                                               2);

   public float detect(RawImage colorImage, Point2f centerPoint)
   {
      centerPoint.x(-1.0f);
      centerPoint.y(-1.0f);

      if (!colorImage.isAvailable())
         return -1.0f;

      colorImage.get();

      float radius;

      int halfWidth = colorImage.getImageWidth() / 2;
      int halfHeight = colorImage.getImageHeight() / 2;

      GpuMat imageMat = colorImage.getGpuImageMat();

      try (GpuMat detectionImage = new GpuMat(halfHeight, halfWidth, opencv_core.CV_8UC3);
           Mat detectionMaskCPU = new Mat(halfHeight, halfWidth, opencv_core.CV_8UC1);
           MatVector contours = new MatVector())
      {

         // Get a smaller image for faster processing (quarter size of original)
         opencv_cudawarping.resize(imageMat, detectionImage, new Size(halfWidth, halfHeight));
         // Blur the image a bit to get a smoother detection
         gaussianFilter.apply(detectionImage, detectionImage);
         // Convert into HSV
         opencv_cudaimgproc.cvtColor(detectionImage, detectionImage, opencv_imgproc.COLOR_BGR2HSV);

         // Get mask of the ball
         GpuMat detectionMask = OpenCVTools.cudaInRange(detectionImage, hsvLowerBound, hsvUpperBound);
         openFilter.apply(detectionMask, detectionMask);

         // Must use CPU Mat from this point. OpenCV doesn't support GpuMats for some functions
         detectionMask.download(detectionMaskCPU);
         PerceptionDebugTools.display("Mask", detectionMaskCPU, 5);
         detectionMask.release();

         // Find the biggest contour (probably our ball)
         opencv_imgproc.findContours(detectionMaskCPU, contours, new Mat(), opencv_imgproc.RETR_EXTERNAL, opencv_imgproc.CHAIN_APPROX_SIMPLE);
         Optional<Mat> maxContourOptional = Arrays.stream(contours.get()).max(Comparator.comparingDouble(opencv_imgproc::contourArea));
         if (maxContourOptional.isEmpty())
            return -1.0f;
         Mat maxContour = maxContourOptional.get();

         // Get the circle of the contour
         // Try fitting an ellipse to counteract motion blur
         try
         {
            RotatedRect ellipse = opencv_imgproc.fitEllipse(maxContour);

            // this is actually the diameter of the ellipse, but we need to double it to get the radius at the original size of the source image
            // i.e. diameter in detection image = radius in source image
            radius = Math.min(ellipse.size().height(), ellipse.size().width());
         }
         catch (RuntimeException e) // if there aren't enough points to fit an ellipse, fit a circle instead.
         {
            float[] radiusResult = new float[1];
            opencv_imgproc.minEnclosingCircle(maxContour, new Point2f(), radiusResult);
            radius = 2.0f * radiusResult[0];
         }

         Moments moments = opencv_imgproc.moments(maxContour);
         centerPoint.x((float) (2.0 * moments.m10() / moments.m00()));
         centerPoint.y((float) (2.0 * moments.m01() / moments.m00()));

         maxContour.close();
      }

      colorImage.release();

      return radius;
   }

   public void setHSVLowerBound(double h, double s, double v)
   {
      hsvLowerBound.put(0L, h);
      hsvLowerBound.put(1L, s);
      hsvLowerBound.put(2L, v);
   }

   public void setHSVUpperBound(double h, double s, double v)
   {
      hsvUpperBound.put(0L, h);
      hsvUpperBound.put(1L, s);
      hsvUpperBound.put(2L, v);
   }

   public void destroy()
   {
      hsvLowerBound.close();
      hsvUpperBound.close();
      gaussianFilter.close();
      openFilter.close();
   }
}
