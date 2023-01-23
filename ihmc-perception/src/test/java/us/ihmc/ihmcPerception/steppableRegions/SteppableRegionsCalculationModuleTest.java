package us.ihmc.ihmcPerception.steppableRegions;

import org.junit.jupiter.api.Test;
import us.ihmc.log.LogTools;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapManager;

import java.util.List;

import static us.ihmc.robotics.Assert.assertEquals;

public class SteppableRegionsCalculationModuleTest
{
   private static final double gridResolution = 0.05;
   private static final double gridSizeXY = 5.0;

   @Test
   public void testSimpleFlatGround()
   {
      HeightMapData heightMap = new HeightMapData(gridResolution, gridSizeXY, 0.0, 0.0);
      double groundHeight = 0.05;

      for (double x = -gridSizeXY / 2.0; x <= gridSizeXY / 2.0; x += gridResolution)
      {
         for (double y = -gridSizeXY / 2.0; y <= gridSizeXY / 2.0; y += gridResolution)
         {
            heightMap.setHeightAt(x, y, groundHeight);
         }
      }

      SteppableRegionsCalculationModule steppableRegionsCalculationModule = new SteppableRegionsCalculationModule();
      steppableRegionsCalculationModule.compute(heightMap);
      List<List<SteppableRegion>> regions = steppableRegionsCalculationModule.getSteppableRegions();

      assertEquals(SteppableRegionsCalculationModule.yawDiscretizations, regions.size());
      for (int i = 0; i < SteppableRegionsCalculationModule.yawDiscretizations; i++)
      {
         assertEquals(1, regions.get(i).size());
      }

      LogTools.info("Tests passed!");
   }
}
