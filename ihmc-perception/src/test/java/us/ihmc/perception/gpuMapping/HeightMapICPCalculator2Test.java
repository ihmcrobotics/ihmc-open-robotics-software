package us.ihmc.perception.gpuMapping;

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

import static org.junit.jupiter.api.Assertions.*;

public class HeightMapICPCalculator2Test
{
   private final HeightMapParameters heightMapParameters = new HeightMapParameters();
   private int localCellsPerAxis;
   private int localCenterIndex;
   private int globalCellsPerAxis;
   private int globalCenterIndex;

   // We don't want to depend on the default values of the parameters in case they change
   private static final double LOCAL_WIDTH_IN_METERS = 4.0;
   private static final double GLOBAL_WIDTH_IN_METERS = 4.0;

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

   @AfterEach
   public void shutdown()
   {
   }

   /**
    * This test ensures that two maps (the local and the global) can run the ICP kernel even if there is no error.
    * This ensures any strange edge cases where the maps are on top of each other get captured and would cause the test to fail.
    * Examples of that could be some corrected transform that has some values when it should have zero, hence the test.
    */
   @Test
   public void testICPWithIdenticalMapsSmall()
   {
      // ----------------- Parameters -----------------
      heightMapParameters.setLocalWidthInMeters(2.0);
      heightMapParameters.setGlobalWidthInMeters(2.0);
      heightMapParameters.setCellSize(0.1);
      heightMapParameters.setIcpMaxIterations(6);
      heightMapParameters.setIcpConvergenceThreshold(0.001);

      int localCenterIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getLocalWidthInMeters(), heightMapParameters.getCellSize());
      int globalCenterIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getGlobalWidthInMeters(), heightMapParameters.getCellSize());

      int localCellsPerAxis = 2 * localCenterIndex + 1;
      int globalCellsPerAxis = 2 * globalCenterIndex + 1;

      HeightMapICPCalculator2 heightMapICPCalculator = new HeightMapICPCalculator2(heightMapParameters);
      GpuMat localMap = new GpuMat(localCellsPerAxis, localCellsPerAxis, opencv_core.CV_32FC1);
      localMap.setTo(new Scalar(0.4));
      GpuMat globalMap = new GpuMat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_32FC1);
      globalMap.setTo(new Scalar(0.1));

      // Run Kernel (Centered at [0, 0] for both)
      heightMapICPCalculator.computeICPErrorTransform(localMap, globalMap, new Point3D(), new Point3D(), new RigidBodyTransform());

      // Let's get the result and see how we did...
      Vector3D correctedTransform = heightMapICPCalculator.getLatestPointCloudErrorTransform();

      System.out.println("Translation X: " + correctedTransform.getX());
      System.out.println("Translation Y: " + correctedTransform.getY());
      System.out.println("Translation Z: " + correctedTransform.getZ());

      assertEquals(0.0, correctedTransform.getX(), 1e-5);
      assertEquals(0.0, correctedTransform.getY(), 1e-5);
      assertEquals(-0.3, correctedTransform.getZ(), 1e-3);

      // Gotta make sure everything shuts closes properly
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
      HeightMapICPCalculator2 heightMapICPCalculator = new HeightMapICPCalculator2(heightMapParameters);
      GpuMat localMap = new GpuMat(localCellsPerAxis, localCellsPerAxis, opencv_core.CV_32FC1);
      localMap.setTo(new Scalar(0.0));
      GpuMat globalMap = new GpuMat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_32FC1);
      globalMap.setTo(new Scalar(0.0));

      // Set Transform to Identity (Zero translation, Zero rotation)
      // This ensures the local coordinates project exactly onto the global coordinates
      RigidBodyTransform identityTransform = new RigidBodyTransform();
      float[] transformArray = new float[16];
      identityTransform.get(transformArray);

      // Note: You would have to look in the kernel, but the local map is defined at (0, 0, 0), so we don't need to pass that in
      Point3D globalMapCenter = new Point3D(0, 0, 0);

      // Run Kernel (Centered at [0, 0] for both)
      heightMapICPCalculator.computeICPErrorTransform(localMap, globalMap, globalMapCenter, globalMapCenter, new RigidBodyTransform());

      // Let's get the result and see how we did...
      Vector3D correctedTransform = heightMapICPCalculator.getLatestPointCloudErrorTransform();

      System.out.println("Translation X: " + correctedTransform.getX());
      System.out.println("Translation Y: " + correctedTransform.getY());
      System.out.println("Translation Z: " + correctedTransform.getZ());

      assertEquals(0.0, correctedTransform.getX(), 1e-5);
      assertEquals(0.0, correctedTransform.getY(), 1e-5);
      assertEquals(0.0, correctedTransform.getZ(), 1e-5);

      // Gotta make sure everything shuts closes properly
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
      HeightMapICPCalculator2 heightMapICPCalculator = new HeightMapICPCalculator2(heightMapParameters);

      GpuMat localMap = new GpuMat(localCellsPerAxis, localCellsPerAxis, opencv_core.CV_32FC1);
      localMap.setTo(new Scalar(1.0));
      GpuMat globalMap = new GpuMat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_32FC1);
      globalMap.setTo(new Scalar(1.0));

      // Set Transform to Identity (Zero translation, Zero rotation)
      // This ensures the local coordinates project exactly onto the global coordinates
      RigidBodyTransform identityTransform = new RigidBodyTransform();
      float[] transformArray = new float[16];
      identityTransform.get(transformArray);

      // Run Kernel (Center at [0, 0] for both)
      heightMapICPCalculator.computeICPErrorTransform(localMap, globalMap, new Point3D(), new Point3D(), new RigidBodyTransform());

      // Let's get the result and see how we did...
      Vector3D correctedTransform = heightMapICPCalculator.getLatestPointCloudErrorTransform();

      System.out.println("Mean X: " + correctedTransform.getX());
      System.out.println("Mean Y: " + correctedTransform.getY());
      System.out.println("Mean Z: " + correctedTransform.getZ());

      assertEquals(0.0, correctedTransform.getX(), 1e-5);
      assertEquals(0.0, correctedTransform.getY(), 1e-5);
      assertEquals(0.0, correctedTransform.getZ(), 1e-5);

      // Gotta make sure everything shuts closes properly
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
      HeightMapICPCalculator2 heightMapICPCalculator = new HeightMapICPCalculator2(heightMapParameters);

      // The global map is higher than the local map
      GpuMat localMap = new GpuMat(localCellsPerAxis, localCellsPerAxis, opencv_core.CV_32FC1);
      localMap.setTo(new Scalar(1.4));
      GpuMat globalMap = new GpuMat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_32FC1);
      globalMap.setTo(new Scalar(1.0));

      // Identity transform (same origins in X,Y)
      RigidBodyTransform identityTransform = new RigidBodyTransform();
      float[] transformArray = new float[16];
      identityTransform.get(transformArray);

      // Run Kernel (Center at [0, 0] for both)
      heightMapICPCalculator.computeICPErrorTransform(localMap, globalMap, new Point3D(), new Point3D(), new RigidBodyTransform());

      // Let's get the result and see how we did...
      Vector3D correctedTransform = heightMapICPCalculator.getLatestPointCloudErrorTransform();

      System.out.println("Mean X: " + correctedTransform.getX());
      System.out.println("Mean Y: " + correctedTransform.getY());
      System.out.println("Mean Z: " + correctedTransform.getZ());

      // Assertions: X,Y should be zero, Z should be 1.0 (pointing from local to global)
      final double EPSILON = 0.1;
      assertEquals(0.0, correctedTransform.getX(), EPSILON);
      assertEquals(0.0, correctedTransform.getY(), EPSILON);
      assertEquals(-0.4, correctedTransform.getZ(), EPSILON);

      // Gotta make sure everything shuts closes properly
      localMap.close();
      globalMap.close();
      heightMapICPCalculator.close();
   }

   /**
    * Make sure the expected X change is captured in ICP. The height values of the two maps are the same
    */
   @Test
   public void testICPWithDirectPointClouds()
   {
      HeightMapICPCalculator2 heightMapICPCalculator = new HeightMapICPCalculator2(heightMapParameters);

      int numPoints = 10;

      // Create local point cloud with a distinct 3D pattern
      // Pattern: points arranged in a line with varying heights
      FloatPointer localPoints = new FloatPointer(numPoints * 3);

      for (int i = 0; i < numPoints; i++)
      {
         float x = i * 0.1f;           // 0.0, 0.1, 0.2, ... 0.9
         float y = i * 0.05f;          // 0.0, 0.05, 0.1, ... 0.45
         float z = (float) Math.sin(i * 0.5);  // Varying height pattern

         localPoints.put(i * 3 + 0, x);
         localPoints.put(i * 3 + 1, y);
         localPoints.put(i * 3 + 2, z);
      }

      // Create global point cloud - SAME PATTERN but offset by (0.1, 0.1, 0.0)
      FloatPointer globalPoints = new FloatPointer(numPoints * 3);

      for (int i = 0; i < numPoints; i++)
      {
         float x = i * 0.1f + 0.1f;     // Offset by 0.1 in X
         float y = i * 0.05f + 0.1f;    // Offset by 0.1 in Y
         float z = (float) Math.sin(i * 0.5);  // Same height pattern

         globalPoints.put(i * 3 + 0, x);
         globalPoints.put(i * 3 + 1, y);
         globalPoints.put(i * 3 + 2, z);
      }

      // Run ICP
      heightMapICPCalculator.computeICPFromPointClouds(localPoints, numPoints, globalPoints, numPoints);

      // Get the result
      Vector3D correctedTransform = heightMapICPCalculator.getLatestPointCloudErrorTransform();

      System.out.println("Corrected X: " + correctedTransform.getX());
      System.out.println("Corrected Y: " + correctedTransform.getY());
      System.out.println("Corrected Z: " + correctedTransform.getZ());

      // Should recover the (0.1, 0.1, 0.0) offset
      assertEquals(0.1, correctedTransform.getX(), 0.01);
      assertEquals(0.1, correctedTransform.getY(), 0.01);
      assertEquals(0.0, correctedTransform.getZ(), 0.001);

      // Cleanup
      localPoints.close();
      globalPoints.close();
      heightMapICPCalculator.close();
   }

   @Test
   public void testICPWithSlopedHeightMap()
   {
      heightMapParameters.setLocalWidthInMeters(1.0);
      heightMapParameters.setGlobalWidthInMeters(2.0);
      heightMapParameters.setCellSize(0.1);
      HeightMapICPCalculator2 heightMapICPCalculator = new HeightMapICPCalculator2(heightMapParameters);

      int localCenterIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getLocalWidthInMeters(), heightMapParameters.getCellSize());
      int globalCenterIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getGlobalWidthInMeters(), heightMapParameters.getCellSize());
      int localCellsPerAxis = 2 * localCenterIndex + 1;
      int globalCellsPerAxis = 2 * globalCenterIndex + 1;

      int borderSize = globalCenterIndex - localCenterIndex; // space around local map in global map

      // Create local and global CPU mats
      Mat localMatCPU = new Mat(localCellsPerAxis, localCellsPerAxis, opencv_core.CV_32FC1);
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
      GpuMat localMap = new GpuMat();
      GpuMat globalMap = new GpuMat();
      localMap.upload(localMatCPU);
      globalMap.upload(globalMatCPU);

      // Slight disagreement in map centers
      Point3D localMapCenter = new Point3D(0.0, 0.0, 0.0);

      // Run ICP
      heightMapICPCalculator.computeICPErrorTransform(localMap, globalMap, localMapCenter, new Point3D(0.1, 0.0, 0.0), new RigidBodyTransform());

      Vector3D correctedTransform = heightMapICPCalculator.getLatestPointCloudErrorTransform();

      System.out.println("Corrected X: " + correctedTransform.getX());
      System.out.println("Corrected Y: " + correctedTransform.getY());
      System.out.println("Corrected Z: " + correctedTransform.getZ());

      // Expect ICP to recover the X offset only
      final double EPSILON = 0.01;
      assertEquals(0.1, correctedTransform.getX(), EPSILON);
      assertEquals(0.0, correctedTransform.getY(), EPSILON);
      assertEquals(0.0, correctedTransform.getZ(), EPSILON);

      // Cleanup
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
      heightMapParameters.setIcpConvergenceThreshold(1e-6);

      HeightMapICPCalculator2 heightMapICPCalculator = new HeightMapICPCalculator2(heightMapParameters);

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

      // ---------------- Run ICP ----------------
      heightMapICPCalculator.computeICPErrorTransform(localMap, globalMap, new Point3D(), new Point3D(0.1, 0.1, 0.0), new RigidBodyTransform());

      Vector3D corrected = heightMapICPCalculator.getLatestPointCloudErrorTransform();

      System.out.println("Corrected = " + corrected);

      // ---------------- EXPECTED FAILURE ----------------
      // With NaN-based invalidation, ICP does NOT recover the true shift.
      // This assertion SHOULD FAIL.
      assertEquals(0.1, corrected.getX(), 1e-2);
      assertEquals(0.1, corrected.getY(), 1e-2);
      assertEquals(0.0, corrected.getZ(), 1e-3);

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
      heightMapParameters.setIcpConvergenceThreshold(0.001);

      HeightMapICPCalculator2 heightMapICPCalculator = new HeightMapICPCalculator2(heightMapParameters);

      int localCenterIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getLocalWidthInMeters(), heightMapParameters.getCellSize());
      int globalCenterIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getGlobalWidthInMeters(), heightMapParameters.getCellSize());

      int localCellsPerAxis = 2 * localCenterIndex + 1;
      int globalCellsPerAxis = 2 * globalCenterIndex + 1;

      // ----------------- Local map (mostly zeros) -----------------
      float[][] localData = new float[][] {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                           {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                           {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                           {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                           {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                           {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                           {0, 0, 0, 0.000873f, 0.001251f, 0.001077f, 0.000690f, 0.000308f, 0, 0, 0},
                                           {0, 0, 0, 0.000201f, 0.000809f, 0.000206f, -0.000195f, 0.000819f, 0, 0, 0},
                                           {0, 0, 0, 0.001001f, -0.000031f, 0.001301f, 0.001041f, 0.001480f, 0.000571f, 0, 0},
                                           {0, 0, -0.000618f, 0.001075f, 0.076491f, 0.085750f, 0.089970f, 0.067645f, 0.087720f, 0, 0},
                                           {0, -0.006122f, -0.006347f, -0.004396f, 0, 0, 0, 0, 0, 0, 0}};

      // 1. Flatten the 2D array into a 1D float array
      float[] flatLocalData = new float[localCellsPerAxis * localCellsPerAxis];
      for (int y = 0; y < localCellsPerAxis; y++)
      {
         for (int x = 0; x < localCellsPerAxis; x++)
         {
            flatLocalData[y * localCellsPerAxis + x] = localData[y][x];
         }
      }

      Mat localMatCPU = new Mat(localCellsPerAxis, localCellsPerAxis, opencv_core.CV_32FC1);
      FloatPointer localFloatArrayPointer = new FloatPointer(localMatCPU.data());
      localFloatArrayPointer.put(flatLocalData);

      // ----------------- Global map (matches local) -----------------
      float[][] globalData = new float[][] {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                            {0, 0, 0, 0.000523f, 0.000482f, 0.000336f, 0.000484f, 0.000418f, 0, 0, 0},
                                            {0, 0, 0, 0.000593f, 0.000586f, 0.000557f, 0.000568f, 0.000534f, 0, 0, 0},
                                            {0, 0, 0, 0.000502f, 0.000502f, 0.000477f, 0.000716f, 0.000628f, 0.000564f, 0.000489f, 0},
                                            {0, 0, 0, 0.000894f, 0.000889f, 0.002925f, 0.078416f, 0.085725f, 0.082217f, 0.081014f, 0},
                                            {0, -0.007268f, -0.007584f, -0.007432f, 0.013890f, 0, 0, 0, 0, 0, 0}};

      // 1. Flatten the 2D array into a 1D float array
      float[] flattenedGlobalData = new float[globalCellsPerAxis * globalCellsPerAxis];
      for (int y = 0; y < globalCellsPerAxis; y++)
      {
         for (int x = 0; x < globalCellsPerAxis; x++)
         {
            flattenedGlobalData[y * globalCellsPerAxis + x] = globalData[y][x];
         }
      }

      Mat globalMatCPU = new Mat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_32FC1);
      FloatPointer globalFloatArrayPointer = new FloatPointer(globalMatCPU.data());
      globalFloatArrayPointer.put(flattenedGlobalData);

      // ----------------- Upload to GPU -----------------
      GpuMat localMap = new GpuMat(localCellsPerAxis, localCellsPerAxis, opencv_core.CV_32FC1);
      GpuMat globalMap = new GpuMat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_32FC1);
      localMap.upload(localMatCPU);
      globalMap.upload(globalMatCPU);

      // ----------------- Apply known transform -----------------
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.getTranslation().set(0.10512145848794968, 0.03088995940675374, 0.9680018086657742); // actual transform from simulation

      float[] transformArray = new float[16];
      transform.get(transformArray);
      FloatPointer hostPtr = new FloatPointer(16);
      hostPtr.put(transformArray);

      // ----------------- Run ICP -----------------
      new Point3D(0.10512145566304529, -6.100405932460462E-4, 0.9680018063204976);
      heightMapICPCalculator.computeICPErrorTransform(localMap, globalMap, new Point3D(), new Point3D(), new RigidBodyTransform());
      Vector3D corrected = heightMapICPCalculator.getLatestPointCloudErrorTransform();
      System.out.println("Corrected transform = " + corrected);

      // ----------------- EXPECTATION -----------------
      // In simulation, no real drift exists, so corrected transform should be near zero
      assertEquals(0.0, corrected.getX(), 0.01);
      assertEquals(0.0, corrected.getY(), 0.01);
      assertEquals(0.0, corrected.getZ(), 0.01);

      // ----------------- Cleanup -----------------
      localMap.close();
      globalMap.close();
      heightMapICPCalculator.close();
   }
}