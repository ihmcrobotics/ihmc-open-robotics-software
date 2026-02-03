package us.ihmc.perception.gpuMapping;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;
import us.ihmc.log.LogTools;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.perception.gpuMapping.TerrainMapData.*;

public class TerrainMapExtractorTest
{
   static private final int iterations = 1000;

   /**
    * This test measures the average runtime of the terrain update kernel over 100 iterations.
    * This test was vibe coded ;)
    */
   @Test
   public void testSnappingTerrainKernelAverageRuntime()
   {
      HeightMapParameters heightMapParameters = new HeightMapParameters();
      TerrainMapParameters terrainMapParameters = new TerrainMapParameters();
      TerrainMapExtractor terrainMapExtractor = new TerrainMapExtractor(heightMapParameters, terrainMapParameters);

      GpuMat heightMap = new GpuMat(401, 401, opencv_core.CV_32FC1);
      heightMap.setTo(new Scalar(100));

      long totalTimeNano = 0;

      for (int i = 0; i < iterations; i++)
      {
         long startTime = System.nanoTime();
         terrainMapExtractor.update(heightMap, new Point3D(0.0, 0.0, 0.0));
         long endTime = System.nanoTime();

         totalTimeNano += (endTime - startTime);
      }

      double averageMillis = (totalTimeNano / (double) iterations) / 1_000_000.0;
      System.out.println("Average terrain update runtime over " + iterations + " iterations: " + averageMillis + " ms");

      TerrainMapData terrainMapData = terrainMapExtractor.getTerrainMapData();
      assertNotNull(terrainMapData);

      terrainMapExtractor.destroy();
   }

   @Test
   public void testFullTraversabilityOnFlatTerrain()
   {
      HeightMapParameters heightMapParameters = new HeightMapParameters();
      double gridResolution = 0.02;
      double terrainWidthXY = 1.0;
      heightMapParameters.setCellSize(gridResolution);
      heightMapParameters.setGlobalWidthInMeters(terrainWidthXY);

      int centerIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getGlobalWidthInMeters(), gridResolution);
      int cellsPerAxis = (centerIndex * 2) + 1;

      TerrainMapParameters steppableRegionParameters = new TerrainMapParameters();
      TerrainMapExtractor terrainMapExtractor = new TerrainMapExtractor(heightMapParameters, steppableRegionParameters);

      double randomHeight = 2.0;
      GpuMat heightMap = new GpuMat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1, new Scalar(randomHeight));

      Mat what = new Mat();
      heightMap.download(what);

      terrainMapExtractor.update(heightMap, new Point3D(0.0, 0.0, 0.0));

      TerrainMapData terrainMapData = terrainMapExtractor.getTerrainMapData();
      float[] traversabilityScoreMap = terrainMapData.getTraversabilityScoreMap();

      byte[] traversabilityClassMap = terrainMapData.getTraversabilityClassMap();

      for (int i = 0; i < heightMap.rows(); i++)
      {
         for (int j = 0; j < heightMap.cols(); j++)
         {
            assertEquals(1.0f, traversabilityScoreMap[i * cellsPerAxis + j], "The values was: (" + traversabilityScoreMap[i * cellsPerAxis + j] + ")");
            Assertions.assertEquals(traversabilityClassMap[i * cellsPerAxis + j], SnapResult.VALID.ordinal());
         }
      }

      terrainMapExtractor.destroy();
   }

   @Test
   public void testNormalCalculationForFlatInclinedTerrain()
   {
      HeightMapParameters heightMapParameters = new HeightMapParameters();
      TerrainMapParameters steppableRegionParameters = new TerrainMapParameters();
      double gridResolution = 0.05;
      double terrainWidthXY = 4.0;

      Point3D gridCenter = new Point3D(0.0, 0.0, 0.0);
      Vector3D normal = new Vector3D(-0.15, 0.0, 1.0);
      normal.normalize();
      Plane3D plane = new Plane3D(gridCenter, normal);

      heightMapParameters.setCellSize(gridResolution);
      heightMapParameters.setGlobalWidthInMeters(terrainWidthXY);
      int centerIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getGlobalWidthInMeters(), gridResolution);
      int cellsPerAxis = 2 * centerIndex + 1;

      Mat heightMat = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1);

      for (int row = 0; row < cellsPerAxis; row++)
      {
         for (int col = 0; col < cellsPerAxis; col++)
         {
            int key = HeightMapTools.indicesToKey(row, col, centerIndex);
            double x = HeightMapTools.keyToXCoordinate(key, gridCenter.getX(), gridResolution, centerIndex);
            double y = HeightMapTools.keyToYCoordinate(key, gridCenter.getY(), gridResolution, centerIndex);

            double z = plane.getZOnPlane(x, y);

            heightMat.ptr(row, col).putFloat((float) z);
         }
      }

      GpuMat gpuHeightMap = new GpuMat(heightMat);

      TerrainMapExtractor terrainMapExtractor = new TerrainMapExtractor(heightMapParameters, steppableRegionParameters);

      terrainMapExtractor.update(gpuHeightMap, new Point3D(0.0, 0.0, 0.0));

      TerrainMapData terrainMapData = terrainMapExtractor.getTerrainMapData();

      byte[] snapNormalXMap = terrainMapData.getSnapNormalXMap();
      byte[] snapNormalYMap = terrainMapData.getSnapNormalYMap();
      byte[] snapNormalZMap = terrainMapData.getSnapNormalZMap();

      byte expectedNormalX = TerrainMapData.packFloatAsByte(normal.getX32(), -NORMAL_MIN_MAX_XY, NORMAL_MIN_MAX_XY);
      byte expectedNormalY = TerrainMapData.packFloatAsByte(normal.getY32(), -NORMAL_MIN_MAX_XY, NORMAL_MIN_MAX_XY);
      byte expectedNormalZ = TerrainMapData.packFloatAsByte(normal.getZ32(), NORMAL_MIN_Z, NORMAL_MAX_Z);

      for (int row = 0; row < cellsPerAxis; row++)
      {
         for (int col = 0; col < cellsPerAxis; col++)
         {
            int key = HeightMapTools.indicesToKey(row, col, centerIndex);

            assertEquals(expectedNormalX, snapNormalXMap[key], "Normal x value is: (" + snapNormalXMap[key] + ")");
            assertEquals(expectedNormalY, snapNormalYMap[key], "Normal y value is: (" + snapNormalYMap[key] + ")");
            assertEquals(expectedNormalZ, snapNormalZMap[key], "Normal z value is: (" + snapNormalZMap[key] + ")");
         }
      }

      terrainMapExtractor.destroy();
   }
}
