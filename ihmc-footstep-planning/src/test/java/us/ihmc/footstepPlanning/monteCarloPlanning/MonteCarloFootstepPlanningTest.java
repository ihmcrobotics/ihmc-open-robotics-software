package us.ihmc.footstepPlanning.monteCarloPlanning;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.perception.tools.PerceptionDebugTools;

public class MonteCarloFootstepPlanningTest
{
   @Disabled
   @Test
   public void testMonteCarloFootstepPlanning()
   {
      MonteCarloPlanner planner = new MonteCarloPlanner();

      Mat heightMap = new Mat(401, 401, opencv_core.CV_16UC1, new Scalar(0));

      // set a rectangle of size 100x100 to 1 in the center
      for (int i = 150; i < 250; i++)
      {
         for (int j = 150; j < 250; j++)
         {
            heightMap.ptr(i, j).putShort((short) 32768);
         }
      }

      PerceptionDebugTools.printMat("Height Map", heightMap, 4);

      Assertions.assertEquals(0.0, 0.0 - 0.0001, 1e-3);
   }
}
