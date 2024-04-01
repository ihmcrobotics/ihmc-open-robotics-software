package us.ihmc.perception.gpuHeightMap;

import org.bytedeco.opencv.opencv_core.Mat;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.perception.heightMap.TerrainMapDebugger;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.tools.PerceptionDataTools;
import us.ihmc.perception.tools.PerceptionDebugTools;

public class TerrainMapDataTest
{

   private OpenCLManager openCLManager = new OpenCLManager();
   private RapidHeightMapExtractor heightMapExtractor = new RapidHeightMapExtractor(openCLManager);

   int size = 100; // 2 m x 2 m
   int scaleFactor = 8;
   private TerrainMapData terrainMapData = new TerrainMapData(size, size);
   private TerrainMapDebugger terrainMapDebugger = new TerrainMapDebugger(size, size, scaleFactor);

   public void initialize()
   {
      LogTools.info("Initializing");

      RapidHeightMapExtractor.getHeightMapParameters().setInternalGlobalWidthInMeters(4.0);
      RapidHeightMapExtractor.getHeightMapParameters().setInternalGlobalCellSizeInMeters(0.02);
      heightMapExtractor.initialize();
      heightMapExtractor.reset();

      LogTools.info("Initialized");
   }

   @Test
   public void testTerrainMapSurfaceNormals()
   {
      // set the middle 20x20 cells to ramp (0.4 m x 0.4 m)
      for (int i = size / 2 - 10; i < size / 2 + 10; i++)
      {
         for (int j = size / 2 - 10; j < size / 2 + 10; j++)
         {
            terrainMapData.setHeightLocal(i * (1 / 50.0f), i,j);
         }
      }

      //PerceptionDebugTools.printMat("Height Map", terrainMapData.getHeightMap(), 1);

      LogTools.info("Normal: {}", terrainMapData.computeSurfaceNormalInWorld(0.3f, 0.3f, 1));
   }

   @Test
   public void testTraversabilityGraphKernel()
   {
      initialize();

      Mat heightMap = heightMapExtractor.getInternalGlobalHeightMapImage().getBytedecoOpenCVMat();
      PerceptionDataTools.fillWithStep(heightMap, new Point2D(0.0f, 0.0f), new Point2D(1.0f, 1.0f), 1.5f, false);
      heightMapExtractor.getInternalGlobalHeightMapImage().writeOpenCLImage(openCLManager);
      heightMapExtractor.populateParameterBuffers(RapidHeightMapExtractor.getHeightMapParameters(), new CameraIntrinsics(), new Point3D());
      heightMapExtractor.computeContactMap();
      heightMapExtractor.readContactMapImage();

      Mat contactMap = heightMapExtractor.getGlobalContactImage();
      Mat traversabilityGraph = heightMapExtractor.getTraversabilityGraphImage().getBytedecoOpenCVMat();
      TerrainMapData terrainMapData = new TerrainMapData(heightMap, contactMap, null, traversabilityGraph);

      PerceptionDebugTools.printMat("Traversability Graph", traversabilityGraph, 1);

      terrainMapDebugger.refresh(terrainMapData);
      terrainMapDebugger.display(0);

      heightMapExtractor.destroy();
      openCLManager.destroy();
   }
}
