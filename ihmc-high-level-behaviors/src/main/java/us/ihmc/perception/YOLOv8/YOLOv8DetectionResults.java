package us.ihmc.perception.YOLOv8;

import org.bytedeco.javacpp.indexer.FloatIndexer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;

import java.util.List;

public class YOLOv8DetectionResults
{
   private static final float MASK_THRESHOLD = 0.0f;
   private final List<YOLOv8Detection> detections;
   private final FloatIndexer outputMasks; // TODO: Find out what this represents

   public YOLOv8DetectionResults(List<YOLOv8Detection> detections, FloatIndexer outputMasks)
   {
      this.detections = detections;
      this.outputMasks = outputMasks;
   }

   public boolean[][] getSegmentationMatrixForObject(YOLOv8DetectableObject objectType)
   {
      if (!detections.isEmpty())
      {
         for (YOLOv8Detection detection : detections)
         {
            // Find the detection that matches the query object type
            if (detection.objectClass().toString().equals(objectType.toString()))
            {
               float[][] maskFloatMatrix = new float[(int) outputMasks.sizes()[2]][(int) outputMasks.sizes()[3]];
               boolean[][] maskBooleanMatrix = new boolean[(int) outputMasks.sizes()[2]][(int) outputMasks.sizes()[3]];
               Mat mask = new Mat((int) outputMasks.sizes()[2], (int) outputMasks.sizes()[3], opencv_core.CV_8U, new Scalar(0));

               for (int i = 0; i < (outputMasks.size(1)) - 1; i++) // FIXME: Why the -1???
                  for (int j = 0; j < (int) outputMasks.size(2); j++)
                     for (int k = 0; k < (int) outputMasks.size(3); k++)
                        maskFloatMatrix[j][k] += detection.maskWeights()[i] * outputMasks.get(0, i, j, k);

               // TODO: Use CV threshold here instead
               for (int j = 0; j < (int) outputMasks.size(2); j++)
               {
                  for (int k = 0; k < (int) outputMasks.size(3); k++)
                  {
                     // TODO: Finish computing the data into boolean matrix
                  }
               }
            }
         }
      }
   }

   public void destroy()
   {
      outputMasks.close();
   }
}
