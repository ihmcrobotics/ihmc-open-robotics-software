package us.ihmc.perception.gpuHeightMap;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;

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

      GpuMat fakeHeightMap = new GpuMat(500, 500, opencv_core.CV_16UC1);
      fakeHeightMap.setTo(new Scalar(100));

      Point3D heightMapCenter = new Point3D();
      heightMapCenter.set(new Point3D(0.0, 0.0, 0.0));

      snappingTerrainExtractor.update(fakeHeightMap, heightMapCenter);
      snappingTerrainExtractor.close();
   }
}
