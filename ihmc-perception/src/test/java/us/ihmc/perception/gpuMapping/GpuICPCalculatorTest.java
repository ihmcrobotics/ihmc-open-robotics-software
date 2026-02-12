package us.ihmc.perception.gpuMapping;

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
import us.ihmc.perception.cuda.CUDATools;

import static org.junit.jupiter.api.Assertions.*;

public class GpuICPCalculatorTest
{
   // We don't want to depend on the default values of the parameters in case they change
   private static final double LOCAL_WIDTH_IN_METERS = 4.0;
   private static final double GLOBAL_WIDTH_IN_METERS = 4.0;
   private final HeightMapParameters heightMapParameters = new HeightMapParameters();
   private int localCellsPerAxis;
   private int globalCellsPerAxis;
   private int localCenterIndex;
   private int globalCenterIndex;

   @BeforeEach
   public void setup()
   {
      heightMapParameters.setLocalWidthInMeters(LOCAL_WIDTH_IN_METERS);
      heightMapParameters.setGlobalWidthInMeters(GLOBAL_WIDTH_IN_METERS);

      localCenterIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getLocalWidthInMeters(), heightMapParameters.getCellSize());
      localCellsPerAxis = 2 * localCenterIndex + 1;
      globalCenterIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getGlobalWidthInMeters(), heightMapParameters.getCellSize());
      globalCellsPerAxis = 2 * globalCenterIndex + 1;
   }

   /**
    * This test ensures that two maps (the local and the global) can run the ICP kernel with only drift in Z.
    * This ensures any strange edge cases where the maps are on top of each other get captured and would cause the test to fail.
    */
   @Test
   public void testICPWithIdenticalMapsSmallOffsetInZ()
   {
      // ----------------- Parameters -----------------
      heightMapParameters.setLocalWidthInMeters(2.0);
      heightMapParameters.setGlobalWidthInMeters(2.0);
      heightMapParameters.setCellSize(0.1);
      heightMapParameters.setIcpMaxIterations(6);
      heightMapParameters.setIcpConvergence(0.001);
      heightMapParameters.setIcpSearchRadius(4.0);

      int localCenterIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getLocalWidthInMeters(), heightMapParameters.getCellSize());
      int globalCenterIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getGlobalWidthInMeters(), heightMapParameters.getCellSize());

      int localCellsPerAxis = 2 * localCenterIndex + 1;
      int globalCellsPerAxis = 2 * globalCenterIndex + 1;

      // ----------------- Create GPU maps -----------------
      GpuICPCalculator heightMapICPCalculator = new GpuICPCalculator(heightMapParameters);

      GpuMat localMap = new GpuMat(localCellsPerAxis, localCellsPerAxis, opencv_core.CV_32FC1);
      localMap.setTo(new Scalar(0.4)); // Local map Z = 0.4

      GpuMat globalMap = new GpuMat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_32FC1);
      globalMap.setTo(new Scalar(0.1)); // Global map Z = 0.1

      // ----------------- Initialize localToGlobalTransform -----------------
      RigidBodyTransform identity = new RigidBodyTransform();
      float[] identityMatrix = new float[16];
      identity.get(identityMatrix);
      FloatPointer cpuLocalToGlobalTransform = new FloatPointer(16);
      cpuLocalToGlobalTransform.put(identityMatrix);
      FloatPointer gpuLocalToGlobalTransform = new FloatPointer();

      CUDATools.mallocAsync(gpuLocalToGlobalTransform, cpuLocalToGlobalTransform.limit(), heightMapICPCalculator.getStream());
      CUDATools.memcpyAsync(gpuLocalToGlobalTransform, cpuLocalToGlobalTransform, cpuLocalToGlobalTransform.limit(), heightMapICPCalculator.getStream());

      // ----------------- Map centers -----------------
      Point3D localCenter = new Point3D(0.0, 0.0, 0.0);
      Point3D globalCenter = new Point3D(0.0, 0.0, 0.0);

      // ----------------- Run ICP -----------------
      heightMapICPCalculator.computeICPErrorTransform(localMap, globalMap, localCenter, globalCenter, gpuLocalToGlobalTransform);

      // ----------------- Get result -----------------
      Vector3D correctedTransform = heightMapICPCalculator.getLatestPointCloudErrorTransform();

      System.out.println("Translation X: " + correctedTransform.getX());
      System.out.println("Translation Y: " + correctedTransform.getY());
      System.out.println("Translation Z: " + correctedTransform.getZ());

      // Z difference between maps is 0.3
      assertEquals(0.0, correctedTransform.getX(), 1e-4);
      assertEquals(0.0, correctedTransform.getY(), 1e-4);
      assertEquals(-0.3, correctedTransform.getZ(), 1e-4);

      // ----------------- Cleanup -----------------
      localMap.close();
      globalMap.close();
      heightMapICPCalculator.close();
   }

   /**
    * This test ensures that two maps (the local and the global) can run the ICP kernel even if there is no error.
    * This ensures any strange edge cases where the maps are on top of each other get captured and would cause the test to fail.
    * Examples of that could be some corrected transform that has some values when it should have zero, hence the test.
    */
   @Test
   public void testICPWithIdenticalMaps()
   {
      // ----------------- Parameters -----------------
      heightMapParameters.setLocalWidthInMeters(2.0);
      heightMapParameters.setGlobalWidthInMeters(2.0);
      heightMapParameters.setCellSize(0.1);
      heightMapParameters.setIcpMaxIterations(6);
      heightMapParameters.setIcpConvergence(0.001);
      heightMapParameters.setIcpSearchRadius(4.0);

      int localCenterIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getLocalWidthInMeters(), heightMapParameters.getCellSize());
      int globalCenterIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getGlobalWidthInMeters(), heightMapParameters.getCellSize());

      int localCellsPerAxis = 2 * localCenterIndex + 1;
      int globalCellsPerAxis = 2 * globalCenterIndex + 1;

      // ----------------- Create GPU maps -----------------
      GpuICPCalculator heightMapICPCalculator = new GpuICPCalculator(heightMapParameters);

      GpuMat localMap = new GpuMat(localCellsPerAxis, localCellsPerAxis, opencv_core.CV_32FC1);
      localMap.setTo(new Scalar(0.0)); // Local map Z = 0.0

      GpuMat globalMap = new GpuMat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_32FC1);
      globalMap.setTo(new Scalar(0.0)); // Global map Z = 0.0

      // ----------------- Initialize localToGlobalTransform -----------------
      RigidBodyTransform identity = new RigidBodyTransform();
      float[] identityMatrix = new float[16];
      identity.get(identityMatrix);
      FloatPointer cpuLocalToGlobalTransform = new FloatPointer(16);
      cpuLocalToGlobalTransform.put(identityMatrix);
      FloatPointer gpuLocalToGlobalTransform = new FloatPointer();

      CUDATools.mallocAsync(gpuLocalToGlobalTransform, cpuLocalToGlobalTransform.limit(), heightMapICPCalculator.getStream());
      CUDATools.memcpyAsync(gpuLocalToGlobalTransform, cpuLocalToGlobalTransform, cpuLocalToGlobalTransform.limit(), heightMapICPCalculator.getStream());

      // ----------------- Map centers -----------------
      Point3D localCenter = new Point3D(0.0, 0.0, 0.0);
      Point3D globalCenter = new Point3D(0.0, 0.0, 0.0);

      // ----------------- Run ICP -----------------
      heightMapICPCalculator.computeICPErrorTransform(localMap, globalMap, localCenter, globalCenter, gpuLocalToGlobalTransform);

      // ----------------- Get result -----------------
      Vector3D correctedTransform = heightMapICPCalculator.getLatestPointCloudErrorTransform();

      System.out.println("Translation X: " + correctedTransform.getX());
      System.out.println("Translation Y: " + correctedTransform.getY());
      System.out.println("Translation Z: " + correctedTransform.getZ());

      // Z difference is zero
      assertEquals(0.0, correctedTransform.getX(), 1e-4);
      assertEquals(0.0, correctedTransform.getY(), 1e-4);
      assertEquals(0.0, correctedTransform.getZ(), 1e-4);

      // ----------------- Cleanup -----------------
      localMap.close();
      globalMap.close();
      heightMapICPCalculator.close();
   }

   /**
    * Similar test to above, only difference is the values of the maps aren't zero. This checks another edge case of what the result would be if the transform
    * was zero. But the values of the maps were non-zero.
    */
   @Test
   public void testICPWithIdenticalMapsRealHeight()
   {
      // ----------------- Parameters -----------------
      heightMapParameters.setLocalWidthInMeters(2.0);
      heightMapParameters.setGlobalWidthInMeters(2.0);
      heightMapParameters.setCellSize(0.1);
      heightMapParameters.setIcpMaxIterations(6);
      heightMapParameters.setIcpConvergence(0.001);
      heightMapParameters.setIcpSearchRadius(4.0);

      int localCenterIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getLocalWidthInMeters(), heightMapParameters.getCellSize());
      int globalCenterIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getGlobalWidthInMeters(), heightMapParameters.getCellSize());

      int localCellsPerAxis = 2 * localCenterIndex + 1;
      int globalCellsPerAxis = 2 * globalCenterIndex + 1;

      // ----------------- Create GPU maps -----------------
      GpuICPCalculator heightMapICPCalculator = new GpuICPCalculator(heightMapParameters);

      GpuMat localMap = new GpuMat(localCellsPerAxis, localCellsPerAxis, opencv_core.CV_32FC1);
      localMap.setTo(new Scalar(1.0)); // Local map Z = 1.0

      GpuMat globalMap = new GpuMat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_32FC1);
      globalMap.setTo(new Scalar(1.0)); // Global map Z = 1.0

      // ----------------- Initialize localToGlobalTransform -----------------
      RigidBodyTransform identity = new RigidBodyTransform();
      float[] identityMatrix = new float[16];
      identity.get(identityMatrix);
      FloatPointer cpuLocalToGlobalTransform = new FloatPointer(16);
      cpuLocalToGlobalTransform.put(identityMatrix);
      FloatPointer gpuLocalToGlobalTransform = new FloatPointer();

      CUDATools.mallocAsync(gpuLocalToGlobalTransform, cpuLocalToGlobalTransform.limit(), heightMapICPCalculator.getStream());
      CUDATools.memcpyAsync(gpuLocalToGlobalTransform, cpuLocalToGlobalTransform, cpuLocalToGlobalTransform.limit(), heightMapICPCalculator.getStream());

      // ----------------- Map centers -----------------
      Point3D localCenter = new Point3D(0.0, 0.0, 0.0);
      Point3D globalCenter = new Point3D(0.0, 0.0, 0.0);

      // ----------------- Run ICP -----------------
      heightMapICPCalculator.computeICPErrorTransform(localMap, globalMap, localCenter, globalCenter, gpuLocalToGlobalTransform);

      // ----------------- Get result -----------------
      Vector3D correctedTransform = heightMapICPCalculator.getLatestPointCloudErrorTransform();

      System.out.println("Translation X: " + correctedTransform.getX());
      System.out.println("Translation Y: " + correctedTransform.getY());
      System.out.println("Translation Z: " + correctedTransform.getZ());

      // Z difference is zero (maps are identical)
      assertEquals(0.0, correctedTransform.getX(), 1e-4);
      assertEquals(0.0, correctedTransform.getY(), 1e-4);
      assertEquals(0.0, correctedTransform.getZ(), 1e-4);

      // ----------------- Cleanup -----------------
      localMap.close();
      globalMap.close();
      heightMapICPCalculator.close();
   }

   /**
    * Makes sure the expected transform captures a Z change if there is no transform in X and Y
    */
   @Test
   public void testICPSameOriginDifferentHeights()
   {
      // ----------------- Parameters -----------------
      heightMapParameters.setLocalWidthInMeters(2.0);
      heightMapParameters.setGlobalWidthInMeters(2.0);
      heightMapParameters.setCellSize(0.1);
      heightMapParameters.setIcpMaxIterations(6);
      heightMapParameters.setIcpConvergence(0.001);
      heightMapParameters.setIcpSearchRadius(4.0);
      heightMapParameters.setIcpMaxDistance(0.5);

      int localCenterIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getLocalWidthInMeters(), heightMapParameters.getCellSize());
      int globalCenterIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getGlobalWidthInMeters(), heightMapParameters.getCellSize());

      int localCellsPerAxis = 2 * localCenterIndex + 1;
      int globalCellsPerAxis = 2 * globalCenterIndex + 1;

      // ----------------- Create GPU maps -----------------
      GpuICPCalculator heightMapICPCalculator = new GpuICPCalculator(heightMapParameters);

      GpuMat localMap = new GpuMat(localCellsPerAxis, localCellsPerAxis, opencv_core.CV_32FC1);
      localMap.setTo(new Scalar(1.4)); // Local map Z = 1.4

      GpuMat globalMap = new GpuMat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_32FC1);
      globalMap.setTo(new Scalar(1.0)); // Global map Z = 1.0

      // ----------------- Initialize localToGlobalTransform -----------------
      RigidBodyTransform identity = new RigidBodyTransform();
      float[] identityMatrix = new float[16];
      identity.get(identityMatrix);
      FloatPointer cpuLocalToGlobalTransform = new FloatPointer(16);
      cpuLocalToGlobalTransform.put(identityMatrix);
      FloatPointer gpuLocalToGlobalTransform = new FloatPointer();

      CUDATools.mallocAsync(gpuLocalToGlobalTransform, cpuLocalToGlobalTransform.limit(), heightMapICPCalculator.getStream());
      CUDATools.memcpyAsync(gpuLocalToGlobalTransform, cpuLocalToGlobalTransform, cpuLocalToGlobalTransform.limit(), heightMapICPCalculator.getStream());

      // ----------------- Map centers -----------------
      Point3D localCenter = new Point3D(0.0, 0.0, 0.0);
      Point3D globalCenter = new Point3D(0.0, 0.0, 0.0);

      // ----------------- Run ICP -----------------
      heightMapICPCalculator.computeICPErrorTransform(localMap, globalMap, localCenter, globalCenter, gpuLocalToGlobalTransform);

      // ----------------- Get result -----------------
      Vector3D correctedTransform = heightMapICPCalculator.getLatestPointCloudErrorTransform();

      System.out.println("Translation X: " + correctedTransform.getX());
      System.out.println("Translation Y: " + correctedTransform.getY());
      System.out.println("Translation Z: " + correctedTransform.getZ());

      // Assertions: X,Y should be zero, Z should correct local -> global difference
      assertEquals(0.0, correctedTransform.getX(), 1e-4);
      assertEquals(0.0, correctedTransform.getY(), 1e-4);
      assertEquals(-0.4, correctedTransform.getZ(), 1e-4);

      // ----------------- Cleanup -----------------
      localMap.close();
      globalMap.close();
      heightMapICPCalculator.close();
   }

   @Test
   public void testICPWithSlopedHeightMapOffsetInX()
   {
      // ----------------- Parameters -----------------
      heightMapParameters.setLocalWidthInMeters(1.0);
      heightMapParameters.setGlobalWidthInMeters(2.0);
      heightMapParameters.setCellSize(0.1);
      heightMapParameters.setIcpMaxIterations(6);
      heightMapParameters.setIcpConvergence(0.001);
      heightMapParameters.setIcpSearchRadius(4.0);

      GpuICPCalculator heightMapICPCalculator = new GpuICPCalculator(heightMapParameters);

      int localCenterIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getLocalWidthInMeters(), heightMapParameters.getCellSize());
      int globalCenterIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getGlobalWidthInMeters(), heightMapParameters.getCellSize());
      int localCellsPerAxis = 2 * localCenterIndex + 1;
      int globalCellsPerAxis = 2 * globalCenterIndex + 1;
      int borderSize = globalCenterIndex - localCenterIndex;

      // ----------------- Create CPU Mats -----------------
      Mat localMatCPU = new Mat(localCellsPerAxis, localCellsPerAxis, opencv_core.CV_32FC1);
      Mat globalMatCPU = new Mat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_32FC1, new Scalar(2.0f));

      // Local map: linear slope in X
      for (int y = 0; y < localCellsPerAxis; y++)
      {
         for (int x = 0; x < localCellsPerAxis; x++)
         {
            float height = 5.0f + x + 2; // slope: 5..(5+localCellsPerAxis-1) left→right
            localMatCPU.ptr(y, x).putFloat(height);
         }
      }

      // Global map: place local map in center, border = 2.0
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

      // ----------------- Upload to GPU -----------------
      GpuMat localMap = new GpuMat();
      GpuMat globalMap = new GpuMat();
      localMap.upload(localMatCPU);
      globalMap.upload(globalMatCPU);

      // ----------------- Run ICP -----------------
      Point3D localMapCenter = new Point3D(0.0, 0.0, 0.0);
      Point3D globalMapCenter = new Point3D(0.1, 0.0, 0.0); // Slight offset in X

      // ----------------- Initialize localToGlobalTransform -----------------
      RigidBodyTransform identity = new RigidBodyTransform();
      float[] identityMatrix = new float[16];
      identity.get(identityMatrix);
      FloatPointer cpuLocalToGlobalTransform = new FloatPointer(16);
      cpuLocalToGlobalTransform.put(identityMatrix);
      FloatPointer gpuLocalToGlobalTransform = new FloatPointer();

      CUDATools.mallocAsync(gpuLocalToGlobalTransform, cpuLocalToGlobalTransform.limit(), heightMapICPCalculator.getStream());
      CUDATools.memcpyAsync(gpuLocalToGlobalTransform, cpuLocalToGlobalTransform, cpuLocalToGlobalTransform.limit(), heightMapICPCalculator.getStream());

      heightMapICPCalculator.computeICPErrorTransform(localMap, globalMap, localMapCenter, globalMapCenter, gpuLocalToGlobalTransform);

      // ----------------- Get result -----------------
      Vector3D correctedTransform = heightMapICPCalculator.getLatestPointCloudErrorTransform();

      System.out.println("Corrected X: " + correctedTransform.getX());
      System.out.println("Corrected Y: " + correctedTransform.getY());
      System.out.println("Corrected Z: " + correctedTransform.getZ());

      // ----------------- Assertions -----------------
      final double EPSILON = 1e-4;
      assertEquals(0.1, correctedTransform.getX(), EPSILON);
      assertEquals(0.0, correctedTransform.getY(), EPSILON);
      assertEquals(0.0, correctedTransform.getZ(), EPSILON);

      // ----------------- Cleanup -----------------
      localMap.close();
      globalMap.close();
      heightMapICPCalculator.close();
   }

   @Test
   public void testICP_FailsWithNaNInvalidation_WhenLocalMapHasZeros()
   {
      heightMapParameters.setLocalWidthInMeters(1.0);
      heightMapParameters.setGlobalWidthInMeters(2.0);
      heightMapParameters.setCellSize(0.1);
      heightMapParameters.setIcpMaxIterations(5);
      heightMapParameters.setIcpConvergence(1e-6);

      GpuICPCalculator heightMapICPCalculator = new GpuICPCalculator(heightMapParameters);

      int localCenterIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getLocalWidthInMeters(), heightMapParameters.getCellSize());
      int globalCenterIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getGlobalWidthInMeters(), heightMapParameters.getCellSize());

      int localCellsPerAxis = 2 * localCenterIndex + 1;
      int globalCellsPerAxis = 2 * globalCenterIndex + 1;

      int borderSize = globalCenterIndex - localCenterIndex;

      // ---------------- Local map: half zeros, half sloped ----------------
      Mat localMatCPU = new Mat(localCellsPerAxis, localCellsPerAxis, opencv_core.CV_32FC1, new Scalar(0.0f));

      for (int y = localCellsPerAxis / 2; y < localCellsPerAxis; y++)
      {
         for (int x = localCellsPerAxis / 2; x < localCellsPerAxis; x++)
         {
            float height = 5.0f + x + y; // simple slope
            localMatCPU.ptr(y, x).putFloat(height);
         }
      }

      // ---------------- Global map: perfect match ----------------
      Mat globalMatCPU = new Mat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_32FC1, new Scalar(2.0f));

      for (int y = 0; y < localCellsPerAxis; y++)
      {
         for (int x = 0; x < localCellsPerAxis; x++)
         {
            float h = localMatCPU.ptr(y, x).getFloat();
            if (h != 0.0f)
            {
               globalMatCPU.ptr(borderSize + y, borderSize + x).putFloat(h);
            }
         }
      }

      // Upload to GPU
      GpuMat localMap = new GpuMat();
      GpuMat globalMap = new GpuMat();
      localMap.upload(localMatCPU);
      globalMap.upload(globalMatCPU);

      // ----------------- Initialize localToGlobalTransform -----------------
      RigidBodyTransform identity = new RigidBodyTransform();
      float[] identityMatrix = new float[16];
      identity.get(identityMatrix);
      FloatPointer cpuLocalToGlobalTransform = new FloatPointer(16);
      cpuLocalToGlobalTransform.put(identityMatrix);
      FloatPointer gpuLocalToGlobalTransform = new FloatPointer();

      CUDATools.mallocAsync(gpuLocalToGlobalTransform, cpuLocalToGlobalTransform.limit(), heightMapICPCalculator.getStream());
      CUDATools.memcpyAsync(gpuLocalToGlobalTransform, cpuLocalToGlobalTransform, cpuLocalToGlobalTransform.limit(), heightMapICPCalculator.getStream());

      // ---------------- Run ICP ----------------
      heightMapICPCalculator.computeICPErrorTransform(localMap, globalMap, new Point3D(), new Point3D(0.1, 0.1, 0.0), gpuLocalToGlobalTransform);

      Vector3D corrected = heightMapICPCalculator.getLatestPointCloudErrorTransform();

      System.out.println("Corrected = " + corrected);

      // ---------------- EXPECTED FAILURE ----------------
      // With NaN-based invalidation, ICP does NOT recover the true shift.
      // This assertion SHOULD FAIL.
      assertEquals(0.1, corrected.getX(), 1e-4);
      assertEquals(0.1, corrected.getY(), 1e-4);
      assertEquals(0.0, corrected.getZ(), 1e-4);

      // Cleanup
      localMap.close();
      globalMap.close();
      heightMapICPCalculator.close();
   }

      @Test
      public void testICP_SimulationScenario_LocalMostlyZero_NoDrift()
      {
         // ----------------- Parameters -----------------
         heightMapParameters.setLocalWidthInMeters(1.0);
         heightMapParameters.setGlobalWidthInMeters(1.0);
         heightMapParameters.setCellSize(0.1);
         heightMapParameters.setIcpMaxIterations(6);
         heightMapParameters.setIcpConvergence(0.001);

         GpuICPCalculator heightMapICPCalculator = new GpuICPCalculator(heightMapParameters);

         int localCenterIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getLocalWidthInMeters(), heightMapParameters.getCellSize());
         int globalCenterIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getGlobalWidthInMeters(), heightMapParameters.getCellSize());

         int localCellsPerAxis = 2 * localCenterIndex + 1;
         int globalCellsPerAxis = 2 * globalCenterIndex + 1;

         // ----------------- Local map (mostly zeros) -----------------
         float[][] localData = new float[][] {
               {0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f},
               {0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f},
               {0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f},
               {0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f},
               {0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f},
               {0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f},
               {0.000000f, 0.000000f, 0.000000f, 0.000433f, 0.000434f, 0.000434f, 0.000434f, 0.000434f, 0.000000f, 0.000000f, 0.000000f},
               {0.000000f, 0.000000f, 0.000000f, 0.000508f, 0.000510f, 0.000508f, 0.000510f, 0.000508f, 0.000000f, 0.000000f, 0.000000f},
               {0.000000f, 0.000000f, 0.000534f, 0.000533f, 0.000535f, 0.000534f, 0.000535f, 0.000533f, 0.000534f, 0.000000f, 0.000000f},
               {0.000000f, 0.000000f, 0.000802f, 0.000803f, 0.076585f, 0.085295f, 0.091000f, 0.066076f, 0.085295f, 0.000000f, 0.000000f},
               {0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f}
         };

         // 1. Flatten the 2D array into a 1D float array
         float[] flatLocalData = new float[localCellsPerAxis * localCellsPerAxis];
         for (int y = 0; y < localCellsPerAxis; y++)
         {
            System.arraycopy(localData[y], 0, flatLocalData, y * localCellsPerAxis, localCellsPerAxis);
         }

         Mat localMatCPU = new Mat(localCellsPerAxis, localCellsPerAxis, opencv_core.CV_32FC1);
         FloatPointer localFloatArrayPointer = new FloatPointer(localMatCPU.data());
         localFloatArrayPointer.put(flatLocalData);

         // ----------------- Global map (matches local) -----------------
         float[][] globalData = new float[][] {
               {0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f},
               {0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f},
               {0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f},
               {0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f},
               {0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f},
               {0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f},
               {0.000000f, 0.000000f, 0.000000f, 0.000433f, 0.000434f, 0.000434f, 0.000434f, 0.000434f, 0.000000f, 0.000000f, 0.000000f},
               {0.000000f, 0.000000f, 0.000000f, 0.000508f, 0.000510f, 0.000508f, 0.000510f, 0.000508f, 0.000000f, 0.000000f, 0.000000f},
               {0.000000f, 0.000000f, 0.000534f, 0.000533f, 0.000535f, 0.000534f, 0.000535f, 0.000533f, 0.000534f, 0.000000f, 0.000000f},
               {0.000000f, 0.000000f, 0.000802f, 0.000803f, 0.076585f, 0.085295f, 0.091000f, 0.066076f, 0.085295f, 0.000000f, 0.000000f},
               {0.000000f, 0.000815f, 0.000814f, 0.000815f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f}
         };

         // 1. Flatten the 2D array into a 1D float array
         float[] flattenedGlobalData = new float[globalCellsPerAxis * globalCellsPerAxis];
         for (int y = 0; y < globalCellsPerAxis; y++)
         {
            System.arraycopy(globalData[y], 0, flattenedGlobalData, y * globalCellsPerAxis, globalCellsPerAxis);
         }

         Mat globalMatCPU = new Mat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_32FC1);
         FloatPointer globalFloatArrayPointer = new FloatPointer(globalMatCPU.data());
         globalFloatArrayPointer.put(flattenedGlobalData);

         // ----------------- Upload to GPU -----------------
         GpuMat localMap = new GpuMat(localCellsPerAxis, localCellsPerAxis, opencv_core.CV_32FC1);
         GpuMat globalMap = new GpuMat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_32FC1);
         localMap.upload(localMatCPU);
         globalMap.upload(globalMatCPU);

         // ----------------- Initialize localToGlobalTransform -----------------
         RigidBodyTransform identity = new RigidBodyTransform();
         float[] identityMatrix = new float[16];
         identity.get(identityMatrix);
         FloatPointer cpuLocalToGlobalTransform = new FloatPointer(16);
         cpuLocalToGlobalTransform.put(identityMatrix);
         FloatPointer gpuLocalToGlobalTransform = new FloatPointer();

         CUDATools.mallocAsync(gpuLocalToGlobalTransform, cpuLocalToGlobalTransform.limit(), heightMapICPCalculator.getStream());
         CUDATools.memcpyAsync(gpuLocalToGlobalTransform, cpuLocalToGlobalTransform, cpuLocalToGlobalTransform.limit(), heightMapICPCalculator.getStream());


         // ----------------- Run ICP -----------------
         heightMapICPCalculator.computeICPErrorTransform(localMap,
                                                         globalMap,
                                                         new Point3D(),
                                                         new Point3D(),
                                                         gpuLocalToGlobalTransform);
         Vector3D corrected = heightMapICPCalculator.getLatestPointCloudErrorTransform();
         System.out.println("Corrected transform = " + corrected);

         // ----------------- EXPECTATION -----------------
         // In simulation, no real drift exists, so corrected transform should be zero
         assertEquals(0.0, corrected.getX(), 1e-5);
         assertEquals(0.0, corrected.getY(), 1e-5);
         assertEquals(0.0, corrected.getZ(), 1e-5);

         // ----------------- Cleanup -----------------
         localMap.close();
         globalMap.close();
         heightMapICPCalculator.close();
      }
}