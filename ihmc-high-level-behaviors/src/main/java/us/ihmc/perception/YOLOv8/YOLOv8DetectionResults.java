package us.ihmc.perception.YOLOv8;

import org.bytedeco.javacpp.indexer.FloatIndexer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;

import java.util.EnumMap;
import java.util.List;

public class YOLOv8DetectionResults
{
   private final List<YOLOv8Detection> detections;
   private final Mat outputMasks;
   private final FloatIndexer outputMasksIndexer;
   private final EnumMap<YOLOv8DetectableObject, Mat> objectMasks = new EnumMap<>(YOLOv8DetectableObject.class);

   public YOLOv8DetectionResults(List<YOLOv8Detection> detections, Mat outputMasks)
   {
      this.detections = detections;

      this.outputMasks = outputMasks;
      this.outputMasksIndexer = outputMasks.createIndexer();
   }

   public Mat getSegmentationMatrixForObject(YOLOv8DetectableObject objectType, float maskThreshold)
   {
      if (objectMasks.containsKey(objectType))
         return objectMasks.get(objectType);

      int numberOfMasks = (int) outputMasksIndexer.size(1);
      int rowSize = (int) outputMasksIndexer.size(2);
      int columnSize = (int) outputMasksIndexer.size(3);
      Mat maskBooleanMat = new Mat(rowSize, columnSize, opencv_core.CV_8UC1, new Scalar(0.0));

      if (!detections.isEmpty())
      {
         for (YOLOv8Detection detection : detections)
         {
            // Find the detection that matches the query object type
            if (detection.objectClass() == objectType)
            {
               float[][] maskFloatMatrix = new float[rowSize][columnSize];

               for (int i = 0; i < numberOfMasks; i++)
                  for (int j = 0; j < rowSize; j++)
                     for (int k = 0; k < columnSize; k++)
                        maskFloatMatrix[j][k] += detection.maskWeights()[i] * outputMasksIndexer.get(0, i, j, k);

               // TODO: Use CV threshold here instead
               for (int j = 0; j < rowSize; j++)
               {
                  for (int k = 0; k < columnSize; k++)
                  {
                     if (maskFloatMatrix[j][k] >= maskThreshold
                         && j >= detection.y() / 4 && j <= (detection.y() + detection.height()) / 4
                         && k >= detection.x() / 4 && k <= (detection.x() + detection.width()) / 4)
                     {
                        maskBooleanMat.data().put((long) j * columnSize + k, (byte) 255);
                     }
                     else
                        maskBooleanMat.data().put((long) j * columnSize + k, (byte) 0);
                  }
               }
            }
         }
      }

      objectMasks.put(objectType, maskBooleanMat);

      return maskBooleanMat;
   }

   public void destroy()
   {
      for (Mat mat : objectMasks.values())
         mat.release();

      outputMasksIndexer.close();
      outputMasks.release();
   }
}
