package us.ihmc.footstepPlanning;

import com.esotericsoftware.minlog.Log;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.steppableRegions.SnapResult;
import us.ihmc.footstepPlanning.steppableRegions.SteppableRegionCalculatorParameters;
import us.ihmc.footstepPlanning.steppableRegions.TerrainMapData;
import us.ihmc.footstepPlanning.steppableRegions.TerrainMapTools;
import us.ihmc.log.LogTools;
import us.ihmc.perception.heightMap.HeightMapTools;
import us.ihmc.perception.heightMap.HeightMapData;
import us.ihmc.perception.heightMap.HeightMapParameters;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.robotDataLogger.LogIndex;

import java.nio.Buffer;
import java.nio.FloatBuffer;
import java.util.Random;

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
      SteppableRegionCalculatorParameters steppableRegionCalculatorParameters = new SteppableRegionCalculatorParameters();
      SnappingTerrainExtractor snappingTerrainExtractor = new SnappingTerrainExtractor(heightMapParameters, steppableRegionCalculatorParameters);

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
   public void testFullTraversabilityOnFlatTerrain()
   {
      HeightMapParameters heightMapParameters = new HeightMapParameters();
      SteppableRegionCalculatorParameters  steppableRegionParameters = new SteppableRegionCalculatorParameters();
      double gridResolution = heightMapParameters.getCellSize();
      double terrainWidthXY = heightMapParameters.getTerrainWidthInMeters();

      Point3D gridCenter = new Point3D(0.0, 0.0, 0.0);
      heightMapParameters.setCellSize(gridResolution);
      heightMapParameters.setTerrainWidthInMeters(terrainWidthXY);
      int centerIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getTerrainWidthInMeters(), gridResolution);
      int cellsPerAxis = (centerIndex * 2) + 1;

      SnappingTerrainExtractor snappingTerrainExtractor = new SnappingTerrainExtractor(heightMapParameters, steppableRegionParameters);

      double randomHeight = 2.0;
      GpuMat fakeHeightMap = new GpuMat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1, new Scalar(randomHeight));

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
      float[] traversabilityScoreMap = terrainMapData.getTraversabilityScoreMap();
      byte[] traversabilityClassMap = terrainMapData.getTraversabilityClassMap();

      for (int i = 0; i < heightMapData.getCellsPerAxis(); i++)
      {
         for (int j = 0; j < heightMapData.getCellsPerAxis(); j++)
         {
            assertTrue(traversabilityScoreMap[i * cellsPerAxis + j] > 0.99f);
            assertEquals(traversabilityClassMap[i * cellsPerAxis + j], SnapResult.VALID.ordinal());
         }
      }

      snappingTerrainExtractor.close();
   }
}
