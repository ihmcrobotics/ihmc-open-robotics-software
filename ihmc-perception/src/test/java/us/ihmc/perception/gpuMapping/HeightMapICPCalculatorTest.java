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
   public void testICPWithPillarAndConflictingTransformAndOrigin()
   {
      HeightMapICPCalculator heightMapICPCalculator = new HeightMapICPCalculator(heightMapParameters, stream);

      Mat localMatCPU = new Mat(localCellsPerAxis, localCellsPerAxis, opencv_core.CV_32FC1, new Scalar(0.0));
      Mat globalMatCPU = new Mat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_32FC1, new Scalar(0.0));

      // ----- Define Pillar Parameters -----
      // I want a pillar that's not in the center of the map, because I'm trying to make two pillars
      // In order for the pillars to be in the same location on both maps, we only use the local indexing to create the pillars.
      // So whatever index we use on the local map, we use the same index on the global map. That will result in different places on the maps relativly, but the same propersions.
      float pillarHeight = 2.0f;
      int startX = localCenterIndex / 2;
      int startY = localCenterIndex / 2;
      int size = 10;

      for (int y = startY; y < startY + size; y++)
      {
         for (int x = startX; x < startX + size; x++)
         {
            localMatCPU.ptr(x, y).putFloat(pillarHeight);
            globalMatCPU.ptr(y, x).putFloat(pillarHeight);
         }
      }

      pillarHeight = 4.0f;
      startX = localCenterIndex;
      startY = localCenterIndex;

      for (int y = startY; y < startY + size; y++)
      {
         for (int x = startX; x < startX + size; x++)
         {
            localMatCPU.ptr(x, y).putFloat(pillarHeight);
            globalMatCPU.ptr(y, x).putFloat(pillarHeight);
         }
      }

      // Create GpuMats and Upload
      GpuMat localMap = new GpuMat();
      GpuMat globalMap = new GpuMat();
      localMap.upload(localMatCPU);
      globalMap.upload(globalMatCPU);

      // Map centers disagree
      // Local center = 0.0
      // Global center = 0.9
      Point3D globalMapCenter = new Point3D(0.9, 0.0, 0.0);

      // Transform claims only 0.8m in X
      // ICP should recover the remaining +0.2m
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.getTranslation().set(0.85, 0.0, 0.0);

      float[] transformArray = new float[16];
      transform.get(transformArray);
      FloatPointer hostPtr = new FloatPointer(16);
      hostPtr.put(transformArray);
      FloatPointer devicePtr = new FloatPointer();
      CUDATools.mallocAsync(devicePtr, 16, stream);
      CUDATools.memcpyAsync(devicePtr, hostPtr, 16, stream);

      // Run Kernel with a transform not matching the origins of hte map
      heightMapICPCalculator.update(localMap, globalMap, devicePtr, globalMapCenter);

      // Let's get the result and see how we did...
      Vector3D correctedTransform = heightMapICPCalculator.getCorrectedTransform();

      System.out.println("Mean X: " + correctedTransform.getX());
      System.out.println("Mean Y: " + correctedTransform.getY());
      System.out.println("Mean Z: " + correctedTransform.getZ());

      // Global center (1.0) - transform (0.8) = 0.2
      assertEquals(0.2, correctedTransform.getX(), 1e-4);
      assertEquals(0.0, correctedTransform.getY(), 1e-5);
      assertEquals(0.0, correctedTransform.getZ(), 1e-5);

      // Cleanup
      hostPtr.close();
      devicePtr.close();
      cudaFreeAsync(devicePtr, stream);
      localMap.close();
      globalMap.close();
   }
}
