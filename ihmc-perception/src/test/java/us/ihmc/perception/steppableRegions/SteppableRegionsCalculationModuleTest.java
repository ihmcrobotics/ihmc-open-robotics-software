package us.ihmc.perception.steppableRegions;

import org.junit.jupiter.api.Test;
import us.ihmc.log.LogTools;
import us.ihmc.perception.steppableRegions.SteppableRegionsCalculationModule;
import us.ihmc.perception.steppableRegions.SteppableRegionsListCollection;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

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

      double extremumValue = gridSizeXY / 2.0 - Math.max(SteppableRegionsCalculationModule.footLength, SteppableRegionsCalculationModule.footWidth) / 2.0;

      SteppableRegionsCalculationModule steppableRegionsCalculationModule = new SteppableRegionsCalculationModule();
      steppableRegionsCalculationModule.compute(heightMap);
      SteppableRegionsListCollection regions = steppableRegionsCalculationModule.getSteppableRegionsListCollection();

      int yawDiscretizations = steppableRegionsCalculationModule.getYawDiscretizations();

      assertEquals(yawDiscretizations, regions.getDiscretizations());

      for (int i = 0; i < yawDiscretizations; i++)
      {
         assertTrue(regions.getSteppableRegions(i).getSteppableRegion(0).getConvexHullInRegionFrame().isPointInside(extremumValue, extremumValue));
         assertTrue(regions.getSteppableRegions(i).getSteppableRegion(0).getConvexHullInRegionFrame().isPointInside(extremumValue, -extremumValue));
         assertTrue(regions.getSteppableRegions(i).getSteppableRegion(0).getConvexHullInRegionFrame().isPointInside(-extremumValue, -extremumValue));
         assertTrue(regions.getSteppableRegions(i).getSteppableRegion(0).getConvexHullInRegionFrame().isPointInside(-extremumValue, extremumValue));

         assertEquals(1, regions.getSteppableRegions(i).getSteppableRegionsAsList().size());
      }

      LogTools.info("Tests passed!");
   }
}
