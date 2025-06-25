package us.ihmc.footstepPlanning;

import com.jme3.terrain.heightmap.HeightMap;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.perception.heightMap.HeightMapMessageTools;
import us.ihmc.perception.heightMap.HeightMapTools;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.perception.heightMap.HeightMapData;
import us.ihmc.perception.heightMap.HeightMapParameters;

public class SnappingTerrainExtractorTest
{
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

      HeightMapData heightMapData = new HeightMapData((float) heightMapParameters.getCellSizeInMeters(),
                                                      (float) heightMapParameters.getTerrainWidthInMeters(),
                                                      0,
                                                      0);

      HeightMapTools.convertToHeightMapData(heightMap, heightMapData, new Point3D(0.0, 0.0, 0.0), (float) 4.0, 0.02F, heightMapParameters);

      snappingTerrainExtractor.update(heightMapData);
      snappingTerrainExtractor.close();
   }
}
