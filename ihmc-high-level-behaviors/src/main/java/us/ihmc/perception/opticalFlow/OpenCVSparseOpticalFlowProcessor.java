package us.ihmc.perception.opticalFlow;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_cudaarithm;
import org.bytedeco.opencv.global.opencv_cudaimgproc;
import org.bytedeco.opencv.global.opencv_cudawarping;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.GpuMatVector;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.bytedeco.opencv.opencv_core.Stream;
import org.bytedeco.opencv.opencv_cudaimgproc.CornersDetector;
import org.bytedeco.opencv.opencv_cudaoptflow.SparsePyrLKOpticalFlow;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.perception.RawImage;

public class OpenCVSparseOpticalFlowProcessor
{
   private final CornersDetector goodFeatureDetector;
   private final SparsePyrLKOpticalFlow opticalFlow;

   private GpuMat firstImageGray = new GpuMat();
   private final GpuMat secondImageGray = new GpuMat();
   private GpuMat previousPoints = new GpuMat();
   private GpuMat mask = new GpuMat();
   private boolean findNewFeatures = false;

   public OpenCVSparseOpticalFlowProcessor()
   {
      goodFeatureDetector = opencv_cudaimgproc.createGoodFeaturesToTrackDetector(opencv_core.CV_8UC1);
      opticalFlow = SparsePyrLKOpticalFlow.create();
   }

   public void setNewImage(RawImage newImage, RawImage newMaskImage)
   {
      newImage.get();

      if (newMaskImage != null)
      {
         newMaskImage.get();

         mask.release();
         mask = newMaskImage.getGpuImageMat().clone();
         mask.convertTo(mask, opencv_core.CV_8UC1);
         findNewFeatures = true;

         newMaskImage.release();
      }

      firstImageGray.release();
      firstImageGray = secondImageGray.clone();
      opencv_cudaimgproc.cvtColor(newImage.getGpuImageMat(), secondImageGray, opencv_imgproc.COLOR_BGR2GRAY);
      opencv_cudawarping.resize(secondImageGray, secondImageGray, mask.size());

      newImage.release();
   }

   public Scalar calculateFlow()
   {
      Scalar averageTranslationXY = null;

      if (findNewFeatures)
      {
         goodFeatureDetector.detect(firstImageGray, previousPoints, mask, Stream.Null());
         findNewFeatures = false;
      }

      if (!previousPoints.empty())
      {
         try (GpuMat newPoints = new GpuMat();
              GpuMat statusMat = new GpuMat();
              GpuMat translations = new GpuMat())
         {
            opticalFlow.calc(firstImageGray, secondImageGray, previousPoints, newPoints, statusMat);

            opencv_cudaarithm.subtract(newPoints, previousPoints, translations);
            Scalar sumXY = opencv_cudaarithm.sum(translations, statusMat);
            int numGoodValues = opencv_cudaarithm.countNonZero(statusMat);

            averageTranslationXY = sumXY.mul(new Scalar(1.0 / numGoodValues, 1.0 / numGoodValues));
            previousPoints.release();
            previousPoints = newPoints.clone();
         }
      }

      return averageTranslationXY;
   }

   public void destroy()
   {
      goodFeatureDetector.close();
      opticalFlow.close();
   }
}
