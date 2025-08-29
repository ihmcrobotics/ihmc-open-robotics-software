package us.ihmc.footstepPlanning;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.steppableRegions.TerrainMapData;
import us.ihmc.perception.heightMap.HeightMapTools;
import us.ihmc.perception.heightMap.HeightMapData;
import us.ihmc.perception.heightMap.HeightMapParameters;

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
      DefaultFootstepPlannerParameters  footstepPlannerParameters = new DefaultFootstepPlannerParameters();
      SnappingTerrainExtractor snappingTerrainExtractor = new SnappingTerrainExtractor(heightMapParameters, footstepPlannerParameters);

      GpuMat fakeHeightMap = new GpuMat(401, 401, opencv_core.CV_16UC1);
      fakeHeightMap.setTo(new Scalar(100));
      Mat heightMap = new Mat(401, 401, opencv_core.CV_8UC1);
      fakeHeightMap.download(heightMap);

      Point3D heightMapCenter = new Point3D();
      heightMapCenter.set(new Point3D(0.0, 0.0, 0.0));

      HeightMapData heightMapData = new HeightMapData((float) heightMapParameters.getCellSize(), (float) heightMapParameters.getTerrainWidthInMeters(), 0, 0);

      HeightMapTools.convertToHeightMapData(heightMap, heightMapData, new Point3D(0.0, 0.0, 0.0), (float) 4.0, 0.02F);

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

      DefaultFootstepPlannerParameters  footstepPlannerParameters = new DefaultFootstepPlannerParameters();
      SnappingTerrainExtractor snappingTerrainExtractor = new SnappingTerrainExtractor(heightMapParameters, footstepPlannerParameters);

      GpuMat fakeHeightMap = new GpuMat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1);
      fakeHeightMap.setTo(new Scalar(32767));
      Mat inputDataFloats = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1);
      fakeHeightMap.download(inputDataFloats);

      HeightMapData heightMapData = new HeightMapData((float) heightMapParameters.getCellSize(),
                                                      (float) heightMapParameters.getTerrainWidthInMeters(),
                                                      gridCenter.getX(),
                                                      gridCenter.getY());

      HeightMapTools.convertToHeightMapData(inputDataFloats,
                                            heightMapData,
                                            gridCenter,
                                            (float) heightMapParameters.getTerrainWidthInMeters(),
                                            (float) heightMapParameters.getCellSize());

      snappingTerrainExtractor.update(heightMapData);

      TerrainMapData terrainMapData = snappingTerrainExtractor.getTerrainMapData();
      byte[] actualResult = terrainMapData.getSteppabilityMap();

      for (int i = 0; i < terrainMapData.getCellsPerAxis(); i++)
      {
         for (int j = 0; j < terrainMapData.getCellsPerAxis(); j++)
         {
            assertEquals(4, actualResult[i * cellsPerAxis + j]);
         }
      }

      snappingTerrainExtractor.close();
   }

   @Test
   public void testSteppableConnectionsIsCorrect()
   {
      // Based on this resolution, 200 / 5 = 40, so we should have 40 * 40 = 1600 points
      double gridResolution = 0.1;
      double terrainWidthXY = 1.0;

      Point3D gridCenter = new Point3D(0.0, 0.0, 0.0);
      HeightMapParameters heightMapParameters = new HeightMapParameters();
      heightMapParameters.setCellSize(gridResolution);
      heightMapParameters.setTerrainWidthInMeters(terrainWidthXY);
      int centerIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getTerrainWidthInMeters(), gridResolution);
      int cellsPerAxis = (centerIndex * 2) + 1;

      DefaultFootstepPlannerParameters  footstepPlannerParameters = new DefaultFootstepPlannerParameters();
      SnappingTerrainExtractor snappingTerrainExtractor = new SnappingTerrainExtractor(heightMapParameters, footstepPlannerParameters);

      GpuMat fakeHeightMap = new GpuMat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1);
      fakeHeightMap.setTo(new Scalar(32767));
      Mat inputDataShorts = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1);
      fakeHeightMap.download(inputDataShorts);

      HeightMapData heightMapData = new HeightMapData((float) heightMapParameters.getCellSize(),
                                                      (float) heightMapParameters.getTerrainWidthInMeters(),
                                                      gridCenter.getX(),
                                                      gridCenter.getY());

      HeightMapTools.convertToHeightMapData(inputDataShorts,
                                            heightMapData,
                                            gridCenter,
                                            (float) heightMapParameters.getTerrainWidthInMeters(),
                                            (float) heightMapParameters.getCellSize());

      snappingTerrainExtractor.update(heightMapData);

      TerrainMapData terrainMapData = snappingTerrainExtractor.getTerrainMapData();
      byte[] actualResult= terrainMapData.getSteppabilityConnectionsMap();

      // We expect all the middle since its fully connected, to be filled with 255, all the bits are set
      for (int i = 1; i < terrainMapData.getCellsPerAxis() - 1; i++)
      {
         for (int j = 1; j < terrainMapData.getCellsPerAxis() - 1; j++)
         {
            assertEquals(255, actualResult[i * cellsPerAxis + j] & 0xFF);
         }
      }

      // The corners will all be different because they will have different bits set
      // These values are the expected bits to be set based on there surrounding neighbors
      // We keep the 0's lying around because it represents the x,y index
      assertEquals(208, actualResult[0] & 0xFF);
      assertEquals(11, actualResult[(cellsPerAxis * cellsPerAxis + cellsPerAxis) - 1] & 0xFF);
      assertEquals(22, actualResult[cellsPerAxis * cellsPerAxis + 0] & 0xFF);
      assertEquals(104, actualResult[0 + cellsPerAxis] & 0xFF);

      snappingTerrainExtractor.close();
   }
}
