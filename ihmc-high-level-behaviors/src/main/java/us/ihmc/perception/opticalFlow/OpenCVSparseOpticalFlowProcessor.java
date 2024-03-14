package us.ihmc.perception.opticalFlow;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_cudaarithm;
import org.bytedeco.opencv.global.opencv_cudaimgproc;
import org.bytedeco.opencv.global.opencv_cudawarping;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.SVD;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.bytedeco.opencv.opencv_core.Stream;
import org.bytedeco.opencv.opencv_cudaimgproc.CornersDetector;
import org.bytedeco.opencv.opencv_cudaoptflow.SparsePyrLKOpticalFlow;
import org.bytedeco.opencv.opencv_tracking.TrackerKCF;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.perception.RawImage;

public class OpenCVSparseOpticalFlowProcessor
{
   private final CornersDetector goodFeatureDetector;
   private final SparsePyrLKOpticalFlow opticalFlow;

   private GpuMat firstImageGray = new GpuMat();
   private final GpuMat secondImageGray = new GpuMat();
   private GpuMat oldPoints = new GpuMat();
   private GpuMat mask = new GpuMat();
   private boolean findNewFeatures = false;

   private final Stopwatch calculationTimer = new Stopwatch();
   private double lastCalculationTime = 0.0;

   public OpenCVSparseOpticalFlowProcessor()
   {
      goodFeatureDetector = opencv_cudaimgproc.createGoodFeaturesToTrackDetector(opencv_core.CV_8UC1);
      opticalFlow = SparsePyrLKOpticalFlow.create();

      calculationTimer.start();
      calculationTimer.suspend();
   }

   public void setFirstImage(RawImage firstImage, RawImage maskImage)
   {
      firstImage.get();
      maskImage.get();

      mask.release();
      mask = maskImage.getGpuImageMat().clone();
      mask.convertTo(mask, opencv_core.CV_8UC1);
      findNewFeatures = true;

      opencv_cudaimgproc.cvtColor(firstImage.getGpuImageMat(), firstImageGray, opencv_imgproc.COLOR_BGR2GRAY);
      opencv_cudawarping.resize(firstImageGray, firstImageGray, mask.size());

      firstImage.release();
      maskImage.release();
   }

   public void setSecondImage(RawImage secondImage)
   {
      secondImage.get();

      opencv_cudaimgproc.cvtColor(secondImage.getGpuImageMat(), secondImageGray, opencv_imgproc.COLOR_BGR2GRAY);
      opencv_cudawarping.resize(secondImageGray, secondImageGray, mask.size());

      secondImage.release();
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

   public Mat calculateFlow(Mat initialPoints, Mat derivedPoints)
   {
      calculationTimer.resume();

      Mat affineTransformMat = null;

      if (/*findNewFeatures*/ true)
      {
         goodFeatureDetector.detect(firstImageGray, oldPoints, mask, Stream.Null());
         findNewFeatures = false;
      }

      if (!oldPoints.empty())
      {
         try (GpuMat newPoints = new GpuMat();
              GpuMat goodOldPoints = new GpuMat();
              GpuMat goodNewPoints = new GpuMat();
              GpuMat statusMat = new GpuMat();
              GpuMat translations = new GpuMat();
              GpuMat A = new GpuMat(2, 2, oldPoints.type());
              Mat cpuA = new Mat(2, 2, oldPoints.type());
              Mat singularValues = new Mat();
              Mat U = new Mat();
              Mat VTransposed = new Mat();
              Mat rotation = new Mat())
         {
            opticalFlow.calc(firstImageGray, secondImageGray, oldPoints, newPoints, statusMat);

            // Nice values to have
            int numGoodValues = opencv_cudaarithm.countNonZero(statusMat);
            Scalar averageScalar = new Scalar(1.0 / numGoodValues, 1.0 / numGoodValues);

            // Find only the good points
            oldPoints.copyTo(goodOldPoints, statusMat);
            newPoints.copyTo(goodNewPoints, statusMat);

            // Get centroid of old points
            Scalar oldPointCentroid = opencv_cudaarithm.sum(goodOldPoints);
            oldPointCentroid = oldPointCentroid.mul(averageScalar);

            // Get 2D versions of the point lists
            GpuMat oldPoints2D = goodOldPoints.reshape(1, 2);
            opencv_cudaarithm.transpose(oldPoints2D, oldPoints2D);         // 2 cols
            GpuMat newPoints2D = goodNewPoints.reshape(1, 2);     // 2 rows

            // Get centroid subtracted points
            GpuMat oldCentroidSubtractedPoints = new GpuMat(oldPoints2D.size(), oldPoints2D.type());
            oldPoints2D.col(0).convertTo(oldCentroidSubtractedPoints.col(0), oldPoints2D.type(), 1.0, -oldPointCentroid.get(0));
            oldPoints2D.col(1).convertTo(oldCentroidSubtractedPoints.col(1), oldPoints2D.type(), 1.0, -oldPointCentroid.get(1));

            GpuMat newCentroidSubtractedPoints = new GpuMat(newPoints2D.size(), newPoints2D.type());
            newPoints2D.row(0).convertTo(newCentroidSubtractedPoints.row(0), newPoints2D.type(), 1.0, -oldPointCentroid.get(0));
            newPoints2D.row(1).convertTo(newCentroidSubtractedPoints.row(1), newPoints2D.type(), 1.0, -oldPointCentroid.get(1));

            // Multiply new and old points to get H
            opencv_cudaarithm.gemm(newPoints2D, oldPoints2D, 1.0, new GpuMat(), 0.0, A);
            // Perform SVD on H and find rotation matrix
            A.download(cpuA);
            SVD.compute(cpuA, singularValues, U, VTransposed, /*SVD.MODIFY_A + */SVD.FULL_UV);
            
            opencv_core.gemm(U, VTransposed, 1.0, new Mat(), 0.0, rotation);
            // Ensure determinant of rotation is positive
            if (opencv_core.determinant(rotation) < 0.0)
            {
               System.out.println("NEGATIVE");
               printMat(rotation, "rotation before: ");

               VTransposed.row(1).convertTo(VTransposed.row(1), VTransposed.type(), -1.0, 0.0);
               SVD.compute(cpuA, singularValues, U, VTransposed, /*SVD.MODIFY_A + */SVD.FULL_UV);
               VTransposed.row(1).convertTo(VTransposed.row(1), VTransposed.type(), -1.0, 0.0);
               opencv_core.gemm(U, VTransposed, 1.0, new Mat(), 0.0, rotation);

               /*rotation.convertTo(rotation, rotation.type(), -1.0, 0.0);*/
               printMat(rotation, "rotation after: " + opencv_core.determinant(rotation));
            }

            // Get translations
            opencv_cudaarithm.subtract(goodNewPoints, goodOldPoints, translations);
            // Sum all translations together, using status as a mask to ignore bad points
            Scalar sumXY = opencv_cudaarithm.sum(translations);
            // Find the average translation
            Scalar averageTranslationXY = sumXY.mul(averageScalar);

            float averageTranslationX = (float) averageTranslationXY.get(0);
            float averageTranslationY = (float) averageTranslationXY.get(1);
            float centroidX = (float) oldPointCentroid.get(0);
            float centroidY = (float) oldPointCentroid.get(1);
            float cosTheta = rotation.data().getFloat();
            float sinTheta = rotation.data().getFloat(Float.BYTES);
//            float dX = centroidX * (1.0f - cosTheta) - centroidY * sinTheta + averageTranslationX * cosTheta + averageTranslationY * sinTheta;
//            float dY = centroidX * sinTheta + centroidY * (1.0f - cosTheta) - averageTranslationX * sinTheta + averageTranslationY * cosTheta;
            float dX = (1.0f - cosTheta) * centroidX - sinTheta * centroidY;
            float dY = sinTheta * centroidX + (1.0f - cosTheta) * centroidY;

            affineTransformMat = new Mat(2, 3, opencv_core.CV_32FC1);
            affineTransformMat.data().putFloat(0L * Float.BYTES, rotation.data().getFloat(0L * Float.BYTES));
            affineTransformMat.data().putFloat(1L * Float.BYTES, rotation.data().getFloat(1L * Float.BYTES));
            affineTransformMat.data().putFloat(2L * Float.BYTES, dX);
            affineTransformMat.data().putFloat(3L * Float.BYTES, rotation.data().getFloat(2L * Float.BYTES));
            affineTransformMat.data().putFloat(4L * Float.BYTES, rotation.data().getFloat(3L * Float.BYTES));
            affineTransformMat.data().putFloat(5L * Float.BYTES, dY);

            // Clean up
            oldPoints2D.release();
            newPoints2D.release();
            oldCentroidSubtractedPoints.release();
            newCentroidSubtractedPoints.release();
            averageScalar.releaseReference();
            averageTranslationXY.releaseReference();

            if (initialPoints != null)
               goodOldPoints.download(initialPoints);

            if (derivedPoints != null)
               goodNewPoints.download(derivedPoints);

            // remember the new points
            oldPoints.release();
            oldPoints = goodNewPoints.clone();
         }
      }

      lastCalculationTime = calculationTimer.lap();
      calculationTimer.suspend();

      return affineTransformMat;
   }

   public double getLastCalculationTime()
   {
      return lastCalculationTime;
   }

   public double getAverageCalculationTime()
   {
      return calculationTimer.averageLap();
   }

   public void destroy()
   {
      goodFeatureDetector.close();
      opticalFlow.close();
   }

   private void printMat(Mat mat, String name)
   {
      System.out.println("Mat: " + name);
      for (int i = 0; i < mat.rows(); ++i)
      {
         if (i == 0)
            System.out.print("/");
         else if (i == mat.rows() - 1)
            System.out.print("\\");
         else
            System.out.print("|");

         for (int j = 0; j < mat.cols(); ++j)
         {
            System.out.print(" " + mat.row(i).col(j).data().getFloat());
         }

         if (i == 0)
            System.out.println(" \\");
         else if (i == mat.rows() - 1)
            System.out.println(" /");
         else
            System.out.println(" |");
      }
   }
}
