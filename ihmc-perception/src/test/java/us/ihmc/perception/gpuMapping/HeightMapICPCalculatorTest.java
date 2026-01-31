package us.ihmc.perception.gpuMapping;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.perception.cuda.CUDAStreamManager;
import us.ihmc.perception.cuda.CUDATools;

import static org.bytedeco.cuda.global.cudart.cudaFreeAsync;
import static org.junit.jupiter.api.Assertions.*;

public class HeightMapICPCalculatorTest
{
   @Test
   public void testICPRunsWithZeroOffset()
   {
      HeightMapParameters heightMapParameters = new HeightMapParameters();
      CUstream_st stream = CUDAStreamManager.getStream();
      HeightMapICPCalculator heightMapICPCalculator = new HeightMapICPCalculator(heightMapParameters, stream);

      int centerIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getWidthInMeters(), heightMapParameters.getCellSize());
      int cellsPerAxis = 2 * centerIndex + 1;

      // 1. Create identical maps
      GpuMat localMap = new GpuMat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1); // Ensure type matches kernel expectation
      localMap.setTo(new Scalar(1.0));

      GpuMat globalMap = new GpuMat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1);
      globalMap.setTo(new Scalar(1.0));

      // 2. Set Transform to Identity (Zero translation, Zero rotation)
      // This ensures the local coordinates project exactly onto the global coordinates
      RigidBodyTransform identityTransform = new RigidBodyTransform();
      float[] transformArray = new float[16];
      identityTransform.get(transformArray);

      // 3. Prepare Device Pointers
      FloatPointer hostPtr = new FloatPointer(16);
      hostPtr.put(transformArray);
      FloatPointer devicePtr = new FloatPointer();
      CUDATools.mallocAsync(devicePtr, 16, stream);
      CUDATools.memcpyAsync(devicePtr, hostPtr, 16, stream);

      // 4. Run Kernel (Center at 0,0 for both)
      Point3D mapCenter = new Point3D(0.0, 0.0, 0.0);
      heightMapICPCalculator.update(localMap, globalMap, devicePtr, mapCenter);

      // 5. Download results and Verify
      //      GpuMat vectorMapGpu = heightMapICPCalculator.getVectorMap();
      //      Mat vectorMapCpu = new Mat();
      //      vectorMapGpu.download(vectorMapCpu);
      //
      //      Scalar meanVector = opencv_core.mean(vectorMapCpu);

      Vector3D meanVector = heightMapICPCalculator.getVectorMap();

      System.out.println("Mean X: " + meanVector.getX());
      System.out.println("Mean Y: " + meanVector.getY());
      System.out.println("Mean Z: " + meanVector.getZ());

      // Assertions: Should be effectively zero
      assertEquals(0.0, meanVector.getX(), 1e-5);
      assertEquals(0.0, meanVector.getY(), 1e-5);
      assertEquals(0.0, meanVector.getZ(), 1e-5);

      // Cleanup
      hostPtr.close();
      cudaFreeAsync(devicePtr, stream);
   }

   @Test
   public void testICPWithOneMeterZOffset()
   {
      HeightMapParameters heightMapParameters = new HeightMapParameters();
      CUstream_st stream = CUDAStreamManager.getStream();
      HeightMapICPCalculator heightMapICPCalculator = new HeightMapICPCalculator(heightMapParameters, stream);

      int centerIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getWidthInMeters(), heightMapParameters.getCellSize());
      int cellsPerAxis = 2 * centerIndex + 1;

      // 1. Create maps with 1 meter Z offset
      // Local map at height 1.0m
      GpuMat localMap = new GpuMat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1);
      localMap.setTo(new Scalar(1));

      // Global map at height 2.0m (1 meter higher)
      GpuMat globalMap = new GpuMat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1);
      globalMap.setTo(new Scalar(2));

      // 2. Identity transform (same origins in X,Y)
      RigidBodyTransform identityTransform = new RigidBodyTransform();
      float[] transformArray = new float[16];
      identityTransform.get(transformArray);

      // 3. Prepare Device Pointers
      FloatPointer hostPtr = new FloatPointer(16);
      hostPtr.put(transformArray);
      FloatPointer devicePtr = new FloatPointer();
      CUDATools.mallocAsync(devicePtr, 16, stream);
      CUDATools.memcpyAsync(devicePtr, hostPtr, 16, stream);

      // 4. Run Kernel (Center at 0,0 for both)
      Point3D mapCenter = new Point3D(0.0, 0.0, 0.0);
      heightMapICPCalculator.update(localMap, globalMap, devicePtr, mapCenter);

      // 5. Download results and Verify
      Vector3D meanVector = heightMapICPCalculator.getVectorMap();

      System.out.println("1m Z-offset test:");
      System.out.println("Mean X: " + meanVector.getX());
      System.out.println("Mean Y: " + meanVector.getY());
      System.out.println("Mean Z: " + meanVector.getZ());

      // Assertions: X,Y should be zero, Z should be 1.0 (pointing from local to global)
      assertEquals(0.0, meanVector.getX(), 1e-5);
      assertEquals(0.0, meanVector.getY(), 1e-5);
      assertEquals(1.0, meanVector.getZ(), 1e-5);

      // Cleanup
      hostPtr.close();
      cudaFreeAsync(devicePtr, stream);
   }

   @Test
   public void testICPWithOneMeterXYOriginOffset()
   {
      HeightMapParameters heightMapParameters = new HeightMapParameters();
      CUstream_st stream = CUDAStreamManager.getStream();
      HeightMapICPCalculator heightMapICPCalculator = new HeightMapICPCalculator(heightMapParameters, stream);

      int centerIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getWidthInMeters(), heightMapParameters.getCellSize());
      int cellsPerAxis = 2 * centerIndex + 1;

      // 1. Create identical height maps (same Z values)
      GpuMat localMap = new GpuMat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1);
      localMap.setTo(new Scalar(1.0));

      GpuMat globalMap = new GpuMat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC1);
      globalMap.setTo(new Scalar(1.0));

      Point3D mapCenter = new Point3D(1.0, 0.0, 0.0); // +1m in X

      // 2. Transform 1.0 meter in x
      RigidBodyTransform translation = new RigidBodyTransform();
      translation.getTranslation().set(new Vector3D(mapCenter));
      float[] transformArray = new float[16];
      translation.get(transformArray);

      // 3. Prepare Device Pointers
      FloatPointer hostPtr = new FloatPointer(16);
      hostPtr.put(transformArray);
      FloatPointer devicePtr = new FloatPointer();
      CUDATools.mallocAsync(devicePtr, 16, stream);
      CUDATools.memcpyAsync(devicePtr, hostPtr, 16, stream);

      // 4. Run Kernel with 1 meter offset in map centers
      // Local map centered at origin (0,0,0)
      // Global map centered at (1,0,0) - 1 meter offset in X
      heightMapICPCalculator.update(localMap, globalMap, devicePtr, mapCenter);

      // 5. Download results and Verify
      Vector3D meanVector = heightMapICPCalculator.getVectorMap();

      System.out.println("1m origin offset test:");
      System.out.println("Mean X: " + meanVector.getX());
      System.out.println("Mean Y: " + meanVector.getY());
      System.out.println("Mean Z: " + meanVector.getZ());

      // Assertions: Should show 1.0m offset in X direction, zero in Y and Z
      assertEquals(1.0, meanVector.getX(), 1e-5);
      assertEquals(0.0, meanVector.getY(), 1e-5);
      assertEquals(0.0, meanVector.getZ(), 1e-5);

      // Cleanup
      hostPtr.close();
      cudaFreeAsync(devicePtr, stream);
   }

   Need to create a test that puts a pillar in the middle of the global map, and a pillar in the middle of the local map
   And ICP shsould be able to find the transform between them if I say the transform is 0.8 but the local center is 0.0, and the global center is 1.0
   So there is the transform saying 0.8 but the map origins saying 1.0. The ICP should be able to find the difference.

      In order to do this the global map may need to be larger then the local map. Going to look into that. I could increase it to 5 meters I guess
}
