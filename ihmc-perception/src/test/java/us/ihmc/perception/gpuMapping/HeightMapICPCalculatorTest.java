package us.ihmc.perception.gpuMapping;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.junit.jupiter.api.BeforeEach;
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
   private final HeightMapParameters heightMapParameters = new HeightMapParameters();
   private int localCellsPerAxis;
   private int localCenterIndex;
   private int globalCellsPerAxis;
   private int globalCenterIndex;

   @BeforeEach
   public void setup()
   {
      localCenterIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getLocalWidthInMeters(), heightMapParameters.getCellSize());
      localCellsPerAxis = 2 * localCenterIndex + 1;
      globalCenterIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getGlobalWidthInMeters(), heightMapParameters.getCellSize());
      globalCellsPerAxis = 2 * globalCenterIndex + 1;
   }

   @Test
   public void testICPRunsWithZeroOffsetAtZeroHeight()
   {
      CUstream_st stream = CUDAStreamManager.getStream();
      HeightMapICPCalculator heightMapICPCalculator = new HeightMapICPCalculator(heightMapParameters, stream);

      // 1. Create identical maps
      GpuMat localMap = new GpuMat(localCellsPerAxis, localCellsPerAxis, opencv_core.CV_32FC1); // Ensure type matches kernel expectation
      localMap.setTo(new Scalar(0.0));

      GpuMat globalMap = new GpuMat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_32FC1);
      globalMap.setTo(new Scalar(0.0));

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

      Vector3D meanVector = heightMapICPCalculator.getVectorMapGPU();

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
   public void testICPRunsWithZeroOffset()
   {
      CUstream_st stream = CUDAStreamManager.getStream();
      HeightMapICPCalculator heightMapICPCalculator = new HeightMapICPCalculator(heightMapParameters, stream);

      // 1. Create identical maps
      GpuMat localMap = new GpuMat(localCellsPerAxis, localCellsPerAxis, opencv_core.CV_32FC1); // Ensure type matches kernel expectation
      localMap.setTo(new Scalar(1.0));

      GpuMat globalMap = new GpuMat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_32FC1);
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

      Vector3D meanVector = heightMapICPCalculator.getVectorMapGPU();

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
      CUstream_st stream = CUDAStreamManager.getStream();
      HeightMapICPCalculator heightMapICPCalculator = new HeightMapICPCalculator(heightMapParameters, stream);

      // 1. Create maps with 1 meter Z offset
      // Local map at height 1.0m
      GpuMat localMap = new GpuMat(localCellsPerAxis, localCellsPerAxis, opencv_core.CV_32FC1);
      localMap.setTo(new Scalar(1));

      // Global map at height 2.0m (1 meter higher)
      GpuMat globalMap = new GpuMat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_32FC1);
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
      Vector3D meanVector = heightMapICPCalculator.getVectorMapGPU();

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

      // 1. Create identical height maps (same Z values)
      GpuMat localMap = new GpuMat(localCellsPerAxis, localCellsPerAxis, opencv_core.CV_32FC1);
      localMap.setTo(new Scalar(1.0));

      GpuMat globalMap = new GpuMat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_32FC1);
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
      Vector3D meanVector = heightMapICPCalculator.getVectorMapGPU();

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

   @Test
   public void testICPWithPillarAndConflictingTransformAndOrigin()
   {
      CUstream_st stream = CUDAStreamManager.getStream();
      HeightMapICPCalculator heightMapICPCalculator = new HeightMapICPCalculator(heightMapParameters, stream);

      // 1. Create CPU Mats (Empty)
      Mat localMatCPU = new Mat(localCellsPerAxis, localCellsPerAxis, opencv_core.CV_32FC1, new Scalar(0.0));
      Mat globalMatCPU = new Mat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_32FC1, new Scalar(0.0));

      // 2. Define Pillar Parameters
      float pillarHeight = 2.0f;
      int startX = localCenterIndex / 2; // Centering a 5x5 pillar around the centerIndex
      int startY = localCenterIndex / 2;
      int size = 10;

      for (int y = startY; y < startY + size; y++)
      {
         for (int x = startX; x < startX + size; x++)
         {
            localMatCPU.ptr(x, y).putFloat(pillarHeight);
         }
      }

      startX = globalCenterIndex / 2; // Centering a 5x5 pillar around the centerIndex
      startY = globalCenterIndex / 2;

      for (int y = startY; y < startY + size; y++)
      {
         for (int x = startX; x < startX + size; x++)
         {
            globalMatCPU.ptr(y, x).putFloat(pillarHeight);
         }
      }

      pillarHeight = 4.0f;
      startX = localCenterIndex; // Centering a 5x5 pillar around the centerIndex
      startY = localCenterIndex;
      size = 10;

      for (int y = startY; y < startY + size; y++)
      {
         for (int x = startX; x < startX + size; x++)
         {
            localMatCPU.ptr(x, y).putFloat(pillarHeight);
         }
      }

      // 4. Create GpuMats and Upload
      GpuMat localMap = new GpuMat();
      GpuMat globalMap = new GpuMat();

      localMap.upload(localMatCPU);
      globalMap.upload(globalMatCPU);

      // 3. Map centers disagree
      // Local center = 0.0
      // Global center = 1.0
      Point3D globalMapCenter = new Point3D(0.9, 0.0, 0.0);

      // 4. Transform claims only 0.8m in X
      // ICP should recover the remaining +0.2m
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.getTranslation().set(0.85, 0.0, 0.0);

      float[] transformArray = new float[16];
      transform.get(transformArray);

      // 5. Upload transform
      FloatPointer hostPtr = new FloatPointer(16);
      hostPtr.put(transformArray);

      FloatPointer devicePtr = new FloatPointer();
      CUDATools.mallocAsync(devicePtr, 16, stream);
      CUDATools.memcpyAsync(devicePtr, hostPtr, 16, stream);

      // 6. Run ICP
      heightMapICPCalculator.update(localMap, globalMap, devicePtr, globalMapCenter);

      // 7. Verify result
      Vector3D meanVector = heightMapICPCalculator.getVectorMapGPU();

      System.out.println("Pillar + conflicting transform test:");
      System.out.println("Mean X: " + meanVector.getX());
      System.out.println("Mean Y: " + meanVector.getY());
      System.out.println("Mean Z: " + meanVector.getZ());

      // Expected:
      // Global center (1.0) - transform (0.8) = 0.2
      assertEquals(0.2, meanVector.getX(), 1e-4);
      assertEquals(0.0, meanVector.getY(), 1e-5);
      assertEquals(0.0, meanVector.getZ(), 1e-5);

      // Cleanup
      hostPtr.close();
      cudaFreeAsync(devicePtr, stream);
   }
}
