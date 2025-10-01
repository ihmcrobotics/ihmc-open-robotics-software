package us.ihmc.perception.gpuHeightMap;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.log.LogTools;
import us.ihmc.perception.heightMap.HeightMapParameters;
import us.ihmc.perception.heightMap.HeightMapTools;
import us.ihmc.perception.heightMap.SnapResult;
import us.ihmc.perception.heightMap.TerrainMapParameters;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.perception.heightMap.TerrainMapExtractor;

import static org.junit.jupiter.api.Assertions.*;

public class TerrainMapExtractorTest
{
   /**
    * This test only proves that the kernel doesn't have a compilation issue.
    * Future tests should expand on this to test the methods themselves
    */
   @Test
   public void testSnappingTerrainKernelRuns()
   {
      HeightMapParameters heightMapParameters = new HeightMapParameters();
      TerrainMapParameters terrainMapParameters = new TerrainMapParameters();
      TerrainMapExtractor terrainMapExtractor = new TerrainMapExtractor(heightMapParameters, terrainMapParameters);

      GpuMat heightMap = new GpuMat(401, 401, opencv_core.CV_32FC1);
      heightMap.setTo(new Scalar(100));

      terrainMapExtractor.update(heightMap);

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
      heightMapParameters.setTerrainWidthInMeters(terrainWidthXY);

      int centerIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getTerrainWidthInMeters(), gridResolution);
      int cellsPerAxis = (centerIndex * 2) + 1;

      TerrainMapParameters steppableRegionParameters = new TerrainMapParameters();
      TerrainMapExtractor terrainMapExtractor = new TerrainMapExtractor(heightMapParameters, steppableRegionParameters);

      double randomHeight = 2.0;
      GpuMat heightMap = new GpuMat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1, new Scalar(randomHeight));

      Mat what = new Mat();
      heightMap.download(what);

      terrainMapExtractor.update(heightMap);

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

   // WIP -- test normal given flat, inclined plane as input
   @Test
   public void testNormalCalculationForFlatInclinedTerrain()
   {
      HeightMapParameters heightMapParameters = new HeightMapParameters();
      TerrainMapParameters steppableRegionParameters = new TerrainMapParameters();
      double gridResolution = heightMapParameters.getCellSize();
      double terrainWidthXY = heightMapParameters.getTerrainWidthInMeters();

      Point3D gridCenter = new Point3D(0.0, 0.0, 0.0);
      Vector3D normal = new Vector3D(-0.15, 0.2, 1.0);
      normal.normalize();
      Plane3D plane = new Plane3D(gridCenter, normal);

      heightMapParameters.setCellSize(gridResolution);
      heightMapParameters.setTerrainWidthInMeters(terrainWidthXY);
      int centerIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getTerrainWidthInMeters(), gridResolution);
      int cellsPerAxis = 2 * centerIndex + 1;

      Mat heightMat = new Mat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1);

      for (int row = 0; row < cellsPerAxis; row++)
      {
         for (int col = 0; col < cellsPerAxis; col++)
         {
            int key = row * cellsPerAxis + col;
            double x = HeightMapTools.keyToXCoordinate(key, gridCenter.getX(), gridResolution, centerIndex);
            double y = HeightMapTools.keyToYCoordinate(key, gridCenter.getY(), gridResolution, centerIndex);

            double z = plane.getZOnPlane(x, y);

            heightMat.ptr(row, col).putFloat((float) z);
         }
      }

      GpuMat gpuHeightMap = new GpuMat(heightMat);

      TerrainMapExtractor terrainMapExtractor = new TerrainMapExtractor(heightMapParameters, steppableRegionParameters);

      terrainMapExtractor.update(gpuHeightMap);

      TerrainMapData terrainMapData = terrainMapExtractor.getTerrainMapData();

      byte[] snapNormalXMap = terrainMapData.getSnapNormalXMap();
      byte[] snapNormalYMap = terrainMapData.getSnapNormalYMap();
      byte[] snapNormalZMap = terrainMapData.getSnapNormalZMap();

      LogTools.info("normal: " + terrainMapData.getNormal(0.3, 0.0));
      System.out.println(terrainMapData.getTraversabilityClass(0.3, 0.0));

      for (int i = 0; i < cellsPerAxis; i++)
      {
         for (int j = 0; j < cellsPerAxis; j++)
         {
            assertEquals(114, snapNormalXMap[i * cellsPerAxis + j], "Normal x value is: (" + snapNormalXMap[i * cellsPerAxis + j] + ")");
            assertEquals(-113, snapNormalYMap[i * cellsPerAxis + j], "Normal y value is: (" + snapNormalYMap[i * cellsPerAxis + j] + ")");
            assertEquals(-5, snapNormalZMap[i * cellsPerAxis + j], "Normal z value is: (" + snapNormalZMap[i * cellsPerAxis + j] + ")");
         }
      }

      terrainMapExtractor.destroy();
   }
}
