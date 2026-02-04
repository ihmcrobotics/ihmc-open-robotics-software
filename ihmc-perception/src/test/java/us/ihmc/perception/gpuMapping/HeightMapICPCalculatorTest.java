package us.ihmc.perception.gpuMapping;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.perception.cuda.CUDAStreamManager;
import us.ihmc.perception.cuda.CUDATools;
import us.ihmc.perception.tools.PerceptionDebugTools;

import static org.bytedeco.cuda.global.cudart.cudaFreeAsync;
import static org.bytedeco.cuda.global.cudart.cudaStreamSynchronize;
import static org.junit.jupiter.api.Assertions.*;

public class HeightMapICPCalculatorTest
{
   private final HeightMapParameters heightMapParameters = new HeightMapParameters();
   private int localCellsPerAxis;
   private int localCenterIndex;
   private int globalCellsPerAxis;
   private int globalCenterIndex;
   private CUstream_st stream;

   // We don't want to depend on the default values of the parameters in case they change
   private static final double LOCAL_WIDTH_IN_METERS = 4.0;
   private static final double GLOBAL_WIDTH_IN_METERS = 4.0;

   @BeforeEach
   public void setup()
   {
      stream = CUDAStreamManager.getStream();

      heightMapParameters.setLocalWidthInMeters(LOCAL_WIDTH_IN_METERS);
      heightMapParameters.setGlobalWidthInMeters(GLOBAL_WIDTH_IN_METERS);

      localCenterIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getLocalWidthInMeters(), heightMapParameters.getCellSize());
      localCellsPerAxis = 2 * localCenterIndex + 1;
      globalCenterIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getGlobalWidthInMeters(), heightMapParameters.getCellSize());
      globalCellsPerAxis = 2 * globalCenterIndex + 1;
   }

   @AfterEach
   public void shutdown()
   {
      cudaStreamSynchronize(stream);
      CUDAStreamManager.releaseStream(stream);
   }

   /**
    * This test ensures that two maps (the local and the global) can run the ICP kernel even if there is no error.
    * This ensures any strange edge cases where the maps are on top of each other get captured and would cause the test to fail.
    * Examples of that could be some corrected transform that has some values when it should have zero, hence the test.
    */
   @Test
   public void testICPWithIdenticalMaps()
   {
      HeightMapICPCalculator heightMapICPCalculator = new HeightMapICPCalculator(heightMapParameters, stream);
      GpuMat localMap = new GpuMat(localCellsPerAxis, localCellsPerAxis, opencv_core.CV_32FC1);
      localMap.setTo(new Scalar(0.0));
      GpuMat globalMap = new GpuMat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_32FC1);
      globalMap.setTo(new Scalar(0.0));

      // Set Transform to Identity (Zero translation, Zero rotation)
      // This ensures the local coordinates project exactly onto the global coordinates
      RigidBodyTransform identityTransform = new RigidBodyTransform();
      float[] transformArray = new float[16];
      identityTransform.get(transformArray);

      // Prepare Device Pointers
      FloatPointer hostPtr = new FloatPointer(16);
      hostPtr.put(transformArray);
      FloatPointer devicePtr = new FloatPointer();
      CUDATools.mallocAsync(devicePtr, 16, stream);
      CUDATools.memcpyAsync(devicePtr, hostPtr, 16, stream);

      // Note: You would have to look in the kernel, but the local map is defined at (0, 0, 0), so we don't need to pass that in
      Point3D globalMapCenter = new Point3D(0, 0, 0);

      // Run Kernel (Centered at [0, 0] for both)
      heightMapICPCalculator.update(localMap, globalMap, devicePtr, globalMapCenter);

      // Let's get the result and see how we did...
      Vector3D correctedTransform = heightMapICPCalculator.getCorrectedTransform();

      System.out.println("Translation X: " + correctedTransform.getX());
      System.out.println("Translation Y: " + correctedTransform.getY());
      System.out.println("Translation Z: " + correctedTransform.getZ());

      assertEquals(0.0, correctedTransform.getX(), 1e-5);
      assertEquals(0.0, correctedTransform.getY(), 1e-5);
      assertEquals(0.0, correctedTransform.getZ(), 1e-5);

      // Gotta make sure everything shuts closes properly
      hostPtr.close();
      devicePtr.close();
      cudaFreeAsync(devicePtr, stream);
      localMap.close();
      globalMap.close();
   }

   /**
    * Similar test to above, only difference is the values of the maps aren't zero. This checks another edge case of what the result would be if the transform
    * was zero. But the values of the maps were non-zero.
    */
   @Test
   public void testICPWithIdenticalMapsRealHeight()
   {
      HeightMapICPCalculator heightMapICPCalculator = new HeightMapICPCalculator(heightMapParameters, stream);

      GpuMat localMap = new GpuMat(localCellsPerAxis, localCellsPerAxis, opencv_core.CV_32FC1);
      localMap.setTo(new Scalar(1.0));
      GpuMat globalMap = new GpuMat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_32FC1);
      globalMap.setTo(new Scalar(1.0));

      // Set Transform to Identity (Zero translation, Zero rotation)
      // This ensures the local coordinates project exactly onto the global coordinates
      RigidBodyTransform identityTransform = new RigidBodyTransform();
      float[] transformArray = new float[16];
      identityTransform.get(transformArray);

      // Prepare Device Pointers
      FloatPointer hostPtr = new FloatPointer(16);
      hostPtr.put(transformArray);
      FloatPointer devicePtr = new FloatPointer();
      CUDATools.mallocAsync(devicePtr, 16, stream);
      CUDATools.memcpyAsync(devicePtr, hostPtr, 16, stream);

      // Note: You would have to look in the kernel, but the local map is defined at (0, 0, 0), so we don't need to pass that in
      Point3D globalMapCenter = new Point3D(0.0, 0.0, 0.0);

      // Run Kernel (Center at [0, 0] for both)
      heightMapICPCalculator.update(localMap, globalMap, devicePtr, globalMapCenter);

      // Let's get the result and see how we did...
      Vector3D correctedTransform = heightMapICPCalculator.getCorrectedTransform();

      System.out.println("Mean X: " + correctedTransform.getX());
      System.out.println("Mean Y: " + correctedTransform.getY());
      System.out.println("Mean Z: " + correctedTransform.getZ());

      assertEquals(0.0, correctedTransform.getX(), 1e-5);
      assertEquals(0.0, correctedTransform.getY(), 1e-5);
      assertEquals(0.0, correctedTransform.getZ(), 1e-5);

      // Gotta make sure everything shuts closes properly
      hostPtr.close();
      devicePtr.close();
      cudaFreeAsync(devicePtr, stream);
      localMap.close();
      globalMap.close();
   }

   /**
    * Makes sure the expected transform captures a Z change if there is no transform in X and Y
    */
   @Test
   public void testICPSameOriginDifferentHeights()
   {
      HeightMapICPCalculator heightMapICPCalculator = new HeightMapICPCalculator(heightMapParameters, stream);

      // The global map is higher than the local map
      GpuMat localMap = new GpuMat(localCellsPerAxis, localCellsPerAxis, opencv_core.CV_32FC1, new Scalar(1.0));
      localMap.setTo(new Scalar(1.0));
      GpuMat globalMap = new GpuMat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_32FC1);
      globalMap.setTo(new Scalar(2.0));

      // Identity transform (same origins in X,Y)
      RigidBodyTransform identityTransform = new RigidBodyTransform();
      float[] transformArray = new float[16];
      identityTransform.get(transformArray);

      // Prepare Device Pointers
      FloatPointer hostPtr = new FloatPointer(16);
      hostPtr.put(transformArray);
      FloatPointer devicePtr = new FloatPointer();
      CUDATools.mallocAsync(devicePtr, 16, stream);
      CUDATools.memcpyAsync(devicePtr, hostPtr, 16, stream);

      // Run Kernel (Center at [0, 0] for both)
      Point3D mapCenter = new Point3D(0.0, 0.0, 0.0);
      heightMapICPCalculator.update(localMap, globalMap, devicePtr, mapCenter);

      // Let's get the result and see how we did...
      Vector3D correctedTransform = heightMapICPCalculator.getCorrectedTransform();

      System.out.println("Mean X: " + correctedTransform.getX());
      System.out.println("Mean Y: " + correctedTransform.getY());
      System.out.println("Mean Z: " + correctedTransform.getZ());

      // Assertions: X,Y should be zero, Z should be 1.0 (pointing from local to global)
      final double EPSILON = 0.1;
      assertEquals(0.0, correctedTransform.getX(), EPSILON);
      assertEquals(0.0, correctedTransform.getY(), EPSILON);
      assertEquals(1.0, correctedTransform.getZ(), EPSILON);

      // Gotta make sure everything shuts closes properly
      hostPtr.close();
      cudaFreeAsync(devicePtr, stream);
      devicePtr.close();
      localMap.close();
      globalMap.close();
   }

   /**
    * Make sure the expected X change is captured in ICP. The height values of the two maps are the same
    */
   @Test
   public void testICPWithOneMeterXYOriginOffset()
   {
      HeightMapICPCalculator heightMapICPCalculator = new HeightMapICPCalculator(heightMapParameters, stream);

      GpuMat localMap = new GpuMat(localCellsPerAxis, localCellsPerAxis, opencv_core.CV_32FC1, new Scalar(1.0));
      localMap.setTo(new Scalar(1.0));
      GpuMat globalMap = new GpuMat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_32FC1);
      globalMap.setTo(new Scalar(1.0));

      // Set the global map center to be 1 meter away from the local map
      Point3D mapCenter = new Point3D(1.0, 0.0, 0.0);

      // Transform 1.0 meter in X
      RigidBodyTransform translation = new RigidBodyTransform();
      translation.getTranslation().set(new Vector3D(mapCenter));
      float[] transformArray = new float[16];
      translation.get(transformArray);

      // Prepare Device Pointers
      FloatPointer hostPtr = new FloatPointer(16);
      hostPtr.put(transformArray);
      FloatPointer devicePtr = new FloatPointer();
      CUDATools.mallocAsync(devicePtr, 16, stream);
      CUDATools.memcpyAsync(devicePtr, hostPtr, 16, stream);

      // Run Kernel with 1 meter offset in map centers
      // Local map centered at origin (0,0,0)
      // Global map centered at (1,0,0) - 1 meter offset in X
      heightMapICPCalculator.update(localMap, globalMap, devicePtr, mapCenter);

      // Let's get the result and see how we did...
      Vector3D correctedTransform = heightMapICPCalculator.getCorrectedTransform();

      System.out.println("Mean X: " + correctedTransform.getX());
      System.out.println("Mean Y: " + correctedTransform.getY());
      System.out.println("Mean Z: " + correctedTransform.getZ());

      // Should show 1.0m offset in X direction, zero in Y and Z
      assertEquals(1.0, correctedTransform.getX(), 1e-5);
      assertEquals(0.0, correctedTransform.getY(), 1e-5);
      assertEquals(0.0, correctedTransform.getZ(), 1e-5);

      // Cleanup
      hostPtr.close();
      cudaFreeAsync(devicePtr, stream);
      devicePtr.close();
      localMap.close();
      globalMap.close();
   }

   @Test
   public void testICPWithSlopedHeightMap()
   {
      heightMapParameters.setLocalWidthInMeters(1.0);
      heightMapParameters.setGlobalWidthInMeters(2.0);
      heightMapParameters.setCellSize(0.1);
      HeightMapICPCalculator heightMapICPCalculator = new HeightMapICPCalculator(heightMapParameters, stream);

      int localCenterIndex  = HeightMapTools.computeCenterIndex(heightMapParameters.getLocalWidthInMeters(), heightMapParameters.getCellSize());
      int globalCenterIndex  = HeightMapTools.computeCenterIndex(heightMapParameters.getGlobalWidthInMeters(), heightMapParameters.getCellSize());
      int localCellsPerAxis = 2 * localCenterIndex + 1;
      int globalCellsPerAxis = 2 * globalCenterIndex + 1;

      int borderSize = globalCenterIndex - localCenterIndex; // space around local map in global map

      // Create local and global CPU mats
      Mat localMatCPU  = new Mat(localCellsPerAxis, localCellsPerAxis, opencv_core.CV_32FC1);
      Mat globalMatCPU = new Mat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_32FC1, new Scalar(2.0f));

      // ---------------- Local map: linear slope in X ----------------
      for (int y = 0; y < localCellsPerAxis; y++)
      {
         for (int x = 0; x < localCellsPerAxis; x++)
         {
            float height = 5.0f + x + 2; // slope: 5..(5+localCellsPerAxis-1) left→right
            localMatCPU.ptr(y, x).putFloat(height);
         }
      }

      // ---------------- Global map: copy local map into center, border = 2.0 ----------------
      for (int y = 0; y < localCellsPerAxis; y++)
      {
         for (int x = 0; x < localCellsPerAxis; x++)
         {
            float height = localMatCPU.ptr(y, x).getFloat();
            int globalX = borderSize + x;
            int globalY = borderSize + y;
            globalMatCPU.ptr(globalY, globalX).putFloat(height);
         }
      }

      // Upload to GPU
      GpuMat localMap  = new GpuMat();
      GpuMat globalMap = new GpuMat();
      localMap.upload(localMatCPU);
      globalMap.upload(globalMatCPU);

      // Slight disagreement in map centers
      Point3D globalMapCenter = new Point3D(0.0, 0.0, 0.0);

      // Slightly wrong transform (ICP should correct +0.1 in X)
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.getTranslation().set(0.1, 0.0, 0.0);

      float[] transformArray = new float[16];
      transform.get(transformArray);

      FloatPointer hostPtr = new FloatPointer(16);
      hostPtr.put(transformArray);

      FloatPointer devicePtr = new FloatPointer();
      CUDATools.mallocAsync(devicePtr, 16, stream);
      CUDATools.memcpyAsync(devicePtr, hostPtr, 16, stream);

      // Run ICP
      heightMapICPCalculator.update(localMap, globalMap, devicePtr, globalMapCenter);

      Vector3D correctedTransform = heightMapICPCalculator.getCorrectedTransform();

      System.out.println("Corrected X: " + correctedTransform.getX());
      System.out.println("Corrected Y: " + correctedTransform.getY());
      System.out.println("Corrected Z: " + correctedTransform.getZ());

      // Expect ICP to recover the X offset only
      final double EPSILON = 0.01;
      assertEquals(0.1, correctedTransform.getX(), EPSILON);
      assertEquals(0.0, correctedTransform.getY(), EPSILON);
      assertEquals(0.0, correctedTransform.getZ(), EPSILON);

      // Cleanup
      hostPtr.close();
      cudaFreeAsync(devicePtr, stream);
      devicePtr.close();
      localMap.close();
      globalMap.close();
   }

   @Test
   public void testICPWithSlopedHeightMapHalfZeros()
   {
      heightMapParameters.setLocalWidthInMeters(1.0);
      heightMapParameters.setGlobalWidthInMeters(2.0);
      heightMapParameters.setCellSize(0.1);
      HeightMapICPCalculator heightMapICPCalculator = new HeightMapICPCalculator(heightMapParameters, stream);

      int localCenterIndex  = HeightMapTools.computeCenterIndex(heightMapParameters.getLocalWidthInMeters(), heightMapParameters.getCellSize());
      int globalCenterIndex  = HeightMapTools.computeCenterIndex(heightMapParameters.getGlobalWidthInMeters(), heightMapParameters.getCellSize());
      int localCellsPerAxis = 2 * localCenterIndex + 1;
      int globalCellsPerAxis = 2 * globalCenterIndex + 1;

      int borderSize = globalCenterIndex - localCenterIndex; // space around local map in global map

      // Create local and global CPU mats
      Mat localMatCPU  = new Mat(localCellsPerAxis, localCellsPerAxis, opencv_core.CV_32FC1, new Scalar(0.0f));
      Mat globalMatCPU = new Mat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_32FC1, new Scalar(2.0f));

      int halfLocalCellsPerAxis = localCellsPerAxis / 2;

      // ---------------- Local map: linear slope in X ----------------
      for (int y = halfLocalCellsPerAxis; y < localCellsPerAxis; y++)
      {
         for (int x = halfLocalCellsPerAxis; x < localCellsPerAxis; x++)
         {
            float height = 5.0f + x + 2; // slope: 5..(5+localCellsPerAxis-1) left→right
            localMatCPU.ptr(y, x).putFloat(height);
         }
      }

      // ---------------- Global map: copy local map into center, border = 2.0 ----------------
      for (int y = halfLocalCellsPerAxis; y < localCellsPerAxis; y++)
      {
         for (int x = halfLocalCellsPerAxis; x < localCellsPerAxis; x++)
         {
            float height = localMatCPU.ptr(y, x).getFloat();
            int globalX = borderSize + x;
            int globalY = borderSize + y;
            globalMatCPU.ptr(globalY, globalX).putFloat(height);
         }
      }

      // Upload to GPU
      GpuMat localMap  = new GpuMat();
      GpuMat globalMap = new GpuMat();
      localMap.upload(localMatCPU);
      globalMap.upload(globalMatCPU);

      // Slight disagreement in map centers
      Point3D globalMapCenter = new Point3D(0.0, 0.0, 0.0);

      // Slightly wrong transform (ICP should correct +0.1 in X)
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.getTranslation().set(0.1, 0.0, 0.0);

      float[] transformArray = new float[16];
      transform.get(transformArray);

      FloatPointer hostPtr = new FloatPointer(16);
      hostPtr.put(transformArray);

      FloatPointer devicePtr = new FloatPointer();
      CUDATools.mallocAsync(devicePtr, 16, stream);
      CUDATools.memcpyAsync(devicePtr, hostPtr, 16, stream);

      // Run ICP
      heightMapICPCalculator.update(localMap, globalMap, devicePtr, globalMapCenter);

      Vector3D correctedTransform = heightMapICPCalculator.getCorrectedTransform();

      System.out.println("Corrected X: " + correctedTransform.getX());
      System.out.println("Corrected Y: " + correctedTransform.getY());
      System.out.println("Corrected Z: " + correctedTransform.getZ());

      // Expect ICP to recover the X offset only
      final double EPSILON = 0.01;
      assertEquals(0.1, correctedTransform.getX(), EPSILON);
      assertEquals(0.0, correctedTransform.getY(), EPSILON);
      assertEquals(0.0, correctedTransform.getZ(), EPSILON);

      // Cleanup
      hostPtr.close();
      cudaFreeAsync(devicePtr, stream);
      devicePtr.close();
      localMap.close();
      globalMap.close();
   }
}
