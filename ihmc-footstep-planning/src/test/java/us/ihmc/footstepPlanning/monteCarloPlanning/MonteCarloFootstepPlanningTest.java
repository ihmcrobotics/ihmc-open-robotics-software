package us.ihmc.footstepPlanning.monteCarloPlanning;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.MonteCarloFootstepPlannerParameters;
import us.ihmc.perception.gpuHeightMap.RapidHeightMapExtractor;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;

public class MonteCarloFootstepPlanningTest
{
   @Disabled
   @Test
   public void testMonteCarloFootstepPlanning()
   {
      OpenCLManager openCLManager = new OpenCLManager();
      RapidHeightMapExtractor heightMapExtractor = new RapidHeightMapExtractor(openCLManager);
      MonteCarloFootstepPlanner planner = new MonteCarloFootstepPlanner();

      HeightMapParameters heightMapParameters = new HeightMapParameters("GPU");
      MonteCarloFootstepPlannerParameters plannerParameters = new MonteCarloFootstepPlannerParameters();

      // height map is 8x8 meters, with a resolution of 0.02 meters, and a 100x100 patch in the center is set to 1m
      Mat heightMap = new Mat(401, 401, opencv_core.CV_16UC1, new Scalar(32768));
      Mat display = new Mat(401, 401, opencv_core.CV_8UC3);

      // set a rectangle of size 100x100 to 1 in the center
      for (int i = 150; i < 250; i++)
      {
         for (int j = 150; j < 250; j++)
         {
            heightMap.ptr(i, j).putShort((short) (32768 + 0.35 * 32768));
         }
      }

      Mat contactMap = heightMapExtractor.computeContactMap(heightMap, heightMapParameters);

      FootstepPlan footstepPlan = planner.generateFootstepPlan(heightMap, contactMap, plannerParameters);

      PerceptionDebugTools.plotFootsteps(display, null, 2);

      PerceptionDebugTools.printMat("Height Map", heightMap, 4);
      PerceptionDebugTools.printMat("Contact Map", contactMap, 4);

      Assertions.assertEquals(0.0, 0.0 - 0.0001, 1e-3);
   }
}
