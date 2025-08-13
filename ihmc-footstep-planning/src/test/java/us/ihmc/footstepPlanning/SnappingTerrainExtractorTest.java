package us.ihmc.footstepPlanning;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.steppableRegions.TerrainMapData;
import us.ihmc.perception.heightMap.HeightMapTools;
import us.ihmc.perception.heightMap.HeightMapData;
import us.ihmc.perception.heightMap.HeightMapParameters;
import us.ihmc.perception.tools.PerceptionDebugTools;

import static org.junit.jupiter.api.Assertions.*;

public class SnappingTerrainExtractorTest
{
   public static final boolean DEBUG = false;

   /**
    * This test only proves that the kernel doesn't have a compilation issue.
    * Future tests should expand on this to test the methods themselves
    */
   @Test
   @Disabled
   public void testSnappingTerrainKernelRuns()
   {
      HeightMapParameters heightMapParameters = new HeightMapParameters();
      SnappingTerrainExtractor snappingTerrainExtractor = new SnappingTerrainExtractor(heightMapParameters);

      GpuMat fakeHeightMap = new GpuMat(401, 401, opencv_core.CV_16UC1);
      fakeHeightMap.setTo(new Scalar(100));
      Mat heightMap = new Mat(401, 401, opencv_core.CV_8UC1);
      fakeHeightMap.download(heightMap);

      Point3D heightMapCenter = new Point3D();
      heightMapCenter.set(new Point3D(0.0, 0.0, 0.0));

      HeightMapData heightMapData = new HeightMapData((float) heightMapParameters.getCellSize(), (float) heightMapParameters.getTerrainWidthInMeters(), 0, 0);

      HeightMapTools.convertToHeightMapData(heightMap, heightMapData, new Point3D(0.0, 0.0, 0.0), (float) 4.0, 0.02F, 10000, 3.2768f);

      snappingTerrainExtractor.update(heightMapData);
      snappingTerrainExtractor.close();
   }

   @Test
   @Disabled
   public void testSteppabilityMatIsFull()
   {
      // Based on this resolution, 200 / 5 = 40, so we should have 40 * 40 = 1600 points
      double gridResolution = 0.05;
      double terrainWidthXY = 2.0;

      Point3D gridCenter = new Point3D(0.0, 0.0, 0.0);
      HeightMapParameters heightMapParameters = new HeightMapParameters();
      heightMapParameters.setCellSize(gridResolution);
      heightMapParameters.setTerrainWidthInMeters(terrainWidthXY);
      int centerIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getTerrainWidthInMeters(), gridResolution);
      int cellsPerAxis = (centerIndex * 2) + 1;

      SnappingTerrainExtractor snappingTerrainExtractor = new SnappingTerrainExtractor(heightMapParameters);

      GpuMat fakeHeightMap = new GpuMat(cellsPerAxis, cellsPerAxis, opencv_core.CV_16UC1);
      fakeHeightMap.setTo(new Scalar(32767));
      Mat inputDataShorts = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_8UC1);
      fakeHeightMap.download(inputDataShorts);

      HeightMapData heightMapData = new HeightMapData((float) heightMapParameters.getCellSize(),
                                                      (float) heightMapParameters.getTerrainWidthInMeters(),
                                                      gridCenter.getX(),
                                                      gridCenter.getY());

      HeightMapTools.convertToHeightMapData(inputDataShorts,
                                            heightMapData,
                                            gridCenter,
                                            (float) heightMapParameters.getTerrainWidthInMeters(),
                                            (float) heightMapParameters.getCellSize(),
                                            (float) heightMapParameters.getHeightScaleFactor(),
                                            (float) heightMapParameters.getHeightOffset());

      snappingTerrainExtractor.update(heightMapData);

      TerrainMapData terrainMapData = snappingTerrainExtractor.getTerrainMapData();
      Mat actualResult = terrainMapData.getSteppabilityMat();

      for (int i = 0; i < actualResult.rows(); i++)
      {
         for (int j = 0; j < actualResult.cols(); j++)
         {
            assertEquals(4, actualResult.row(i).col(j).ptr().get());
         }
      }

      if (DEBUG)
      {
         PerceptionDebugTools.printMat("result", actualResult, 1);
      }

      snappingTerrainExtractor.close();
   }
}
