package us.ihmc.footstepPlanning.steppableRegions;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.SnappingTerrainManager;
import us.ihmc.log.LogTools;
import us.ihmc.perception.heightMap.HeightMapData;
import us.ihmc.perception.heightMap.HeightMapParameters;
import us.ihmc.perception.heightMap.HeightMapTools;

import static org.junit.jupiter.api.Assertions.*;

public class SteppableRegionsCalculationModuleTest
{
   // Based on this resolution, 200 / 5 = 40, so we should have 40 * 40 = 1600 points
   private static final double gridResolution = 0.05;
   private static final double gridSizeXY = 2.0;
   private static final Point3D gridCenter = new Point3D(0.0, 0.0, 0.0);
   private static final HeightMapParameters heightMapParameters = new HeightMapParameters();

   @Test
   public void testSimpleFlatGround()
   {
      int cellsPerAxis = HeightMapTools.computeCenterIndex(gridSizeXY, gridResolution);
      Mat heightMapMat = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_16UC1, new Scalar(32768));
      HeightMapData heightMapData = new HeightMapData(gridResolution, gridSizeXY, gridCenter.getX(), gridCenter.getY());
      HeightMapTools.convertToHeightMapData(heightMapMat,
                                            heightMapData,
                                            gridCenter,
                                            (float) gridSizeXY,
                                            (float) gridResolution,
                                            (float) heightMapParameters.getHeightScaleFactor(),
                                            (float) heightMapParameters.getHeightOffset());

      double extremumValue = gridSizeXY / 2.0 - Math.max(SteppableRegionsCalculationModule.footLength, SteppableRegionsCalculationModule.footWidth) / 2.0;

      SteppableRegionsCalculationModule steppableRegionsCalculationModule = new SteppableRegionsCalculationModule();


      HeightMapParameters smallerParametersForTest = new HeightMapParameters();
      smallerParametersForTest.setCellSize(gridResolution);
      smallerParametersForTest.setTerrainWidthInMeters(gridSizeXY);

      SnappingTerrainManager snappingTerrainManager = new SnappingTerrainManager(smallerParametersForTest);
      snappingTerrainManager.updateAndPublish(heightMapData);
      TerrainMapData terrainMapData = snappingTerrainManager.getTerrainMapData();
      steppableRegionsCalculationModule.compute(terrainMapData);
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

      heightMapMat.close();
      LogTools.info("Tests passed!");
   }
}
