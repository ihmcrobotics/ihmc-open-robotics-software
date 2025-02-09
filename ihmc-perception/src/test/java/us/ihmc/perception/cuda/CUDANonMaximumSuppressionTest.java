package us.ihmc.perception.cuda;

import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.IntPointer;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

public class CUDANonMaximumSuppressionTest
{
   @Test
   public void testNMS()
   {
      // We will have 3 clusters of boxes, each containing 3 boxes (9 total)
      int boxesPerCluster = 3;
      int numberOfClusters = 3;
      int boxCount = numberOfClusters * boxesPerCluster;

      // Each box consists of 5 floats (x, y, width, height, score)
      int floatsPerBox = 5;

      // We'll use an IoU threshold of 0.3 for NMS
      float intersectionOverUnionThreshold = 0.3f;

      try (FloatPointer boxes = new FloatPointer(floatsPerBox * boxCount);
           IntPointer includedIndices = new IntPointer(boxCount);
           CUDANonMaximumSuppression nms = new CUDANonMaximumSuppression())
      {
         // Put the box values into a FloatPointer with varying scores and overlap.
         boxes.put(0.0f, 0.0f, 10.0f, 10.0f, 0.9f,    // box 0 (highest score of first cluster)
                   1.0f, 0.0f, 10.0f, 11.0f, 0.7f,    // box 1
                   1.0f, 2.0f, 9.0f, 8.0f, 0.8f,      // box 2

                   20.0f, 0.0f, 7.0f, 4.0f, 0.5f,     // box 3
                   22.0f, 1.0f, 6.0f, 4.0f, 0.6f,     // box 4 (highest score of second cluster)
                   19.0f, 0.0f, 8.0f, 3.0f, 0.3f,     // box 5

                   20.0f, 20.0f, 12.0f, 2.0f, 0.7f,   // box 6
                   20.0f, 21.0f, 10.0f, 0.0f, 0.9f,   // box 7 (this box is invalid, since the height is 0)
                   18.0f, 19.0f, 13.0f, 3.0f, 0.8f);  // box 8 (highest scoring valid box of third cluster)

         // Test slow NMS
         long includedCount = nms.runSlow(boxes, boxCount, intersectionOverUnionThreshold, includedIndices);

         // Put included indices into an array list
         List<Integer> includedBoxIndices = new ArrayList<>();
         for (int i = 0; i < includedCount; ++i)
            includedBoxIndices.add(includedIndices.get(i));

         assertEquals(3, includedCount);                                // We should have 3 included boxes
         assertTrue(includedBoxIndices.containsAll(List.of(0, 4, 8)));  // They should be the three highest scoring boxes of each cluster (that are valid)

         // Repeat test fast NMS
         includedCount = nms.runFast(boxes, boxCount, intersectionOverUnionThreshold, includedIndices);

         includedBoxIndices.clear();
         for (int i = 0; i < includedCount; ++i)
            includedBoxIndices.add(includedIndices.get(i));

         assertEquals(3, includedCount);
         assertTrue(includedBoxIndices.containsAll(List.of(0, 4, 8)));

         // Repeat test auto fast/slow NMS
         includedCount = nms.run(boxes, boxCount, intersectionOverUnionThreshold, includedIndices);

         includedBoxIndices.clear();
         for (int i = 0; i < includedCount; ++i)
            includedBoxIndices.add(includedIndices.get(i));

         assertEquals(3, includedCount);
         assertTrue(includedBoxIndices.containsAll(List.of(0, 4, 8)));
      }
   }

}
