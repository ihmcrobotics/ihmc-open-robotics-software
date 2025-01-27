package us.ihmc.perception.cuda;

import org.bytedeco.javacpp.BoolPointer;
import org.bytedeco.javacpp.FloatPointer;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

public class CUDANonMaximumSuppressionTest
{
   @Test
   public void testNMS() throws Exception
   {
      int boxesPerCluster = 3;
      int numberOfClusters = 3;
      int boxCount = numberOfClusters * boxesPerCluster;
      int floatsPerBox = 5;
      try (FloatPointer boxes = new FloatPointer(floatsPerBox * boxCount);
           BoolPointer inclusionVector = new BoolPointer(boxCount);
           CUDANonMaximumSuppression nms = new CUDANonMaximumSuppression())
      {
         boxes.put(0.0f, 0.0f, 10.0f, 10.0f, 0.9f,    // 0
                   1.0f, 0.0f, 10.0f, 11.0f, 0.7f,    // 1
                   1.0f, 2.0f, 9.0f, 8.0f, 0.8f,      // 2

                   20.0f, 0.0f, 7.0f, 4.0f, 0.5f,     // 3
                   22.0f, 1.0f, 6.0f, 4.0f, 0.6f,     // 4
                   19.0f, 0.0f, 8.0f, 3.0f, 0.3f,     // 5

                   20.0f, 20.0f, 12.0f, 2.0f, 0.7f,   // 6
                   20.0f, 21.0f, 10.0f, 1.0f, 0.9f,   // 7
                   18.0f, 19.0f, 13.0f, 3.0f, 0.8f);  // 8

         // Test slow NMS
         nms.runSlow(boxes, boxCount, 0.2f, inclusionVector);

         List<Integer> includedBoxIndices = new ArrayList<>();
         for (int i = 0; i < boxCount; ++i)
         {
            if (inclusionVector.get(i))
               includedBoxIndices.add(i);
         }

         assertEquals(3, includedBoxIndices.size());
         assertEquals(0, includedBoxIndices.get(0));
         assertEquals(4, includedBoxIndices.get(1));
         assertEquals(7, includedBoxIndices.get(2));

         // Test fast NMS
         nms.runFast(boxes, boxCount, 0.2f, inclusionVector);

         includedBoxIndices = new ArrayList<>();
         for (int i = 0; i < boxCount; ++i)
         {
            if (inclusionVector.get(i))
               includedBoxIndices.add(i);
         }

         assertEquals(3, includedBoxIndices.size());
         assertEquals(0, includedBoxIndices.get(0));
         assertEquals(4, includedBoxIndices.get(1));
         assertEquals(7, includedBoxIndices.get(2));

         // Test auto fast/slow NMS
         nms.run(boxes, boxCount, 0.2f, inclusionVector);

         includedBoxIndices = new ArrayList<>();
         for (int i = 0; i < boxCount; ++i)
         {
            if (inclusionVector.get(i))
               includedBoxIndices.add(i);
         }

         assertEquals(3, includedBoxIndices.size());
         assertEquals(0, includedBoxIndices.get(0));
         assertEquals(4, includedBoxIndices.get(1));
         assertEquals(7, includedBoxIndices.get(2));
      }
   }

}
