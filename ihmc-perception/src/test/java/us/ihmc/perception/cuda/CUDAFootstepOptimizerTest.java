package us.ihmc.perception.cuda;

import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.perception.gpuMapping.HeightMapData;

import static org.junit.jupiter.api.Assertions.*;

public class CUDAFootstepOptimizerTest
{
   @Disabled
   public void findMinimumCost()
   {
      CUDAFootstepOptimizer footstepOptimizer = new CUDAFootstepOptimizer(0.5f, 0.25f);

      float[] costs = new float[10000];
      float[] solutions = new float[30000];
      for (int i=0; i< costs.length; i++)
      {
         costs[i] = (float) Math.random();
      }
      for (int i=0; i< solutions.length; i++)
      {
         solutions[i] = (float) Math.random();
      }
      Result result = findMinimumAndIndex(costs);

      footstepOptimizer.setGpuCosts(costs);
      footstepOptimizer.testResultKernel();

      float bestCost = footstepOptimizer.getBestCost();
      float bestIndex = footstepOptimizer.getBestIndex();

      assertEquals(result.minValue, bestCost);
      assertEquals(result.index, bestIndex);

      footstepOptimizer.close();
   }

   // Helper class to store the result
   private class Result
   {
      float minValue;
      int index;

      public Result(float minValue, int index)
      {
         this.minValue = minValue;
         this.index = index;
      }
   }

   private Result findMinimumAndIndex(float[] array)
   {
      if (array == null || array.length == 0)
      {
         throw new IllegalArgumentException("Array must not be null or empty.");
      }

      float minValue = array[0];
      int index = 0;

      for (int i = 1; i < array.length; i++)
      {
         if (array[i] < minValue)
         {
            minValue = array[i];
            index = i;
         }
      }

      return new Result(minValue, index);
   }

   @Disabled
   public void computeCosts()
   {
      CUDAFootstepOptimizer footstepOptimizer = new CUDAFootstepOptimizer(0.5f,0.25f);
      footstepOptimizer.compute(new HeightMapData(0.03, 5.0, 0.0, 0.0), new FramePose3D());
   }
}
