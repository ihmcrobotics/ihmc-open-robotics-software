package us.ihmc.footstepPlanning.steppableRegions;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.SnappingTerrainExtractor;
import us.ihmc.log.LogTools;
import us.ihmc.perception.heightMap.HeightMapData;
import us.ihmc.perception.heightMap.HeightMapParameters;
import us.ihmc.perception.heightMap.HeightMapTools;
import us.ihmc.perception.tools.PerceptionDebugTools;

import static org.junit.jupiter.api.Assertions.*;

public class SteppableRegionsCalculationModuleTest
{
   @Test
   public void testSimpleFlatGround()
   {
      // Based on this resolution, 200 / 5 = 40, so we should have 40 * 40 = 1600 points
      double gridResolution = 0.1;
      double terrainWidth = 1.0;
      Point3D gridCenter = new Point3D(0.0, 0.0, 0.0);
      HeightMapParameters heightMapParameters = new HeightMapParameters();
      heightMapParameters.setCellSize(gridResolution);
      heightMapParameters.setTerrainWidthInMeters(terrainWidth);

      int centerIndex = HeightMapTools.computeCenterIndex(terrainWidth, gridResolution);
      int cellsPerAxis = 2 * centerIndex + 1;
      Mat heightMapMat = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_16UC1, new Scalar(32767));
      HeightMapData heightMapData = new HeightMapData(gridResolution, terrainWidth, gridCenter.getX(), gridCenter.getY());
      HeightMapTools.convertToHeightMapData(heightMapMat,
                                            heightMapData,
                                            gridCenter,
                                            (float) terrainWidth,
                                            (float) gridResolution,
                                            (float) heightMapParameters.getHeightScaleFactor(),
                                            (float) heightMapParameters.getHeightOffset());

      double extremumValue = terrainWidth / 2.0 - Math.max(SteppableRegionsCalculationModule.footLength, SteppableRegionsCalculationModule.footWidth) / 2.0;

      SteppableRegionsCalculationModule steppableRegionsCalculationModule = new SteppableRegionsCalculationModule();

      SnappingTerrainExtractor snappingTerrainExtractor = new SnappingTerrainExtractor(heightMapParameters);
      snappingTerrainExtractor.update(heightMapData);
      TerrainMapData terrainMapData = snappingTerrainExtractor.getTerrainMapData();

      PerceptionDebugTools.printMat("ii", terrainMapData.getSteppabilityMat(), 1);
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
