package us.ihmc.perception.opticalFlow;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_cudaarithm;
import org.bytedeco.opencv.global.opencv_cudaimgproc;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Stream;
import org.bytedeco.opencv.opencv_cudaimgproc.CornersDetector;
import org.bytedeco.opencv.opencv_cudaoptflow.SparsePyrLKOpticalFlow;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.perception.RawImage;

public class OpenCVSparseOpticalFlowProcessor
{
   private final CornersDetector trackFeatureDetector;
   private final SparsePyrLKOpticalFlow opticalFlow;

   private GpuMat firstImageGray = new GpuMat();
   private final GpuMat secondImageGray = new GpuMat();
   private GpuMat previousPoints = new GpuMat();
   private GpuMat mask = null;
   private boolean useMask = false;

   public OpenCVSparseOpticalFlowProcessor()
   {
      trackFeatureDetector = opencv_cudaimgproc.createGoodFeaturesToTrackDetector(opencv_core.CV_8UC1);
      opticalFlow = SparsePyrLKOpticalFlow.create();
   }

   public void setNewImage(RawImage newImage, RawImage newMask)
   {
      newImage.get();

      if (newMask != null)
      {
         newMask.get();

         if (mask == null)
            mask = new GpuMat(newMask.getImageHeight(), newMask.getImageWidth(), opencv_core.CV_8UC1);
         opencv_cudaarithm.threshold(newMask.getGpuImageMat(), mask, 0.5, 255.0, opencv_core.CV_8UC1);
         useMask = true;

         newMask.release();
      }
      else
      {
         useMask = false;
      }

      firstImageGray.release();
      firstImageGray = secondImageGray.clone();
      opencv_cudaimgproc.cvtColor(newImage.getGpuImageMat(), secondImageGray, opencv_imgproc.COLOR_BGR2GRAY);

      newImage.release();
   }

   public Vector2D calculateFlow()
   {
      if (useMask)
      {
         trackFeatureDetector.detect(firstImageGray, previousPoints, mask, Stream.Null());
      }

      GpuMat newPointsGPU = new GpuMat();
      GpuMat statusMat = new GpuMat();
      opticalFlow.calc(firstImageGray, secondImageGray, previousPoints, newPointsGPU, statusMat);

      statusMat.convertTo(statusMat, newPointsGPU.type());
      opencv_cudaarithm.multiply(newPointsGPU, statusMat, newPointsGPU);

      Mat newPointsCPU = new Mat(newPointsGPU.size(), newPointsGPU.type());
      newPointsGPU.download(newPointsCPU);

      float totalX = 0.0f;
      float totalY = 0.0f;
      int numberOfDetectedPoints = 0;
      for (int i = 0; i < newPointsCPU.cols(); ++i) // TODO: maybe .rows()?
      {
         BytePointer pointer = newPointsCPU.ptr(0, i);
         float xValue = pointer.getFloat();
         float yValue = pointer.getFloat(Float.BYTES);

         if (xValue > 0.0f)
         {
            totalX += xValue;
            totalY += yValue;
            numberOfDetectedPoints++;
         }
      }

      previousPoints.close();
      previousPoints = newPointsGPU.clone();
      newPointsGPU.close();
      newPointsCPU.close();
      statusMat.close();


      return new Vector2D(totalX / numberOfDetectedPoints, totalY / numberOfDetectedPoints);
   }

   public void destroy()
   {
      trackFeatureDetector.close();
      opticalFlow.close();
   }
}
