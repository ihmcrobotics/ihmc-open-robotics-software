package us.ihmc.perception.voxelMap;

import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.perception.RawImage;
import us.ihmc.sensors.CameraIntrinsics;

import java.time.Instant;
import java.util.HashSet;
import java.util.Set;

import static org.bytedeco.cuda.global.cudart.cudaMemcpy;
import static org.bytedeco.cuda.global.cudart.cudaMemcpyDefault;
import static org.bytedeco.opencv.global.opencv_core.CV_16UC1;
import static org.junit.jupiter.api.Assertions.*;

public class VoxelMapExtractorTest
{
   private static final int MAP_SIZE = 32;
   private static final float VOXEL_SIZE = 0.1f;
   private static final float DEPTH_DISCRETIZATION = 0.001f;

   @Test
   public void testFlatDepthImageWithIdentityTransforms()
   {
      int height = 720;
      int width = 1280;
      float fx = 2000.0f;
      float fy = 2000.0f;
      float cx = 0.5f * width;
      float cy = 0.5f * height;
      int depthValue = 1030; // 1.03 m

      Mat depthMat = new Mat(height, width, CV_16UC1, new Scalar(depthValue));
      CameraIntrinsics intrinsics = new CameraIntrinsics(height, width, fx, fy, cx, cy);
      RawImage depthImage = RawImage.createWith16BitDepth(depthMat, intrinsics, new RigidBodyTransform(), Instant.now(), 0, DEPTH_DISCRETIZATION);

      try (VoxelMapExtractor extractor = new VoxelMapExtractor(MAP_SIZE, VOXEL_SIZE))
      {
         float[] voxelMap = downloadVoxelMap(extractor.getVoxelMap(new Pose3D(), depthImage).getGpuData());

         // CPU reference mirroring the kernel's float math (identity transforms, so no FMA contraction differences)
         float[] expectedMap = new float[MAP_SIZE * MAP_SIZE * MAP_SIZE];
         float depthInMeters = DEPTH_DISCRETIZATION * depthValue;
         for (int pixelY = 0; pixelY < height; pixelY++)
         {
            for (int pixelX = 0; pixelX < width; pixelX++)
            {
               float imageX = (pixelX - cx) / fx * depthInMeters;
               float imageY = (pixelY - cy) / fy * depthInMeters;
               // pixelDepthToPoint3D: sensor frame is X forward, Y left, Z up
               int voxelX = voxelIndex(depthInMeters);
               int voxelY = voxelIndex(-imageX);
               int voxelZ = voxelIndex(-imageY);
               if (inBounds(voxelX) && inBounds(voxelY) && inBounds(voxelZ))
                  expectedMap[flatIndex(voxelX, voxelY, voxelZ)] = 1.0f;
            }
         }

         assertArrayEquals(expectedMap, voxelMap);
      }

      depthImage.release();
   }

   @Test
   public void testSparseDepthWithTransformedCameraAndOrigin()
   {
      int height = 64;
      int width = 64;
      float fx = 100.0f;
      float fy = 100.0f;
      float cx = 0.5f * width;
      float cy = 0.5f * height;

      int[][] pixelsAndDepthValues = {{32, 32, 1000}, {10, 20, 800}, {50, 40, 1200}, {0, 0, 65000}}; // last one lands far outside the map

      Mat depthMat = new Mat(height, width, CV_16UC1, new Scalar(0.0));
      for (int[] pixelAndDepthValue : pixelsAndDepthValues)
         depthMat.ptr(pixelAndDepthValue[1], pixelAndDepthValue[0]).putShort((short) pixelAndDepthValue[2]);

      RigidBodyTransform sensorToWorldTransform = new RigidBodyTransform(new YawPitchRoll(0.7, 0.2, -0.1), new Vector3D(0.2, -0.1, 0.1));
      CameraIntrinsics intrinsics = new CameraIntrinsics(height, width, fx, fy, cx, cy);
      RawImage depthImage = RawImage.createWith16BitDepth(depthMat, intrinsics, sensorToWorldTransform, Instant.now(), 0, DEPTH_DISCRETIZATION);

      Pose3D origin = new Pose3D(0.3, 0.1, -0.2, 0.4, 0.0, 0.0);
      RigidBodyTransform worldToMapTransform = new RigidBodyTransform();
      origin.get(worldToMapTransform);
      worldToMapTransform.invert();

      Set<Integer> expectedOccupiedIndices = new HashSet<>();
      for (int[] pixelAndDepthValue : pixelsAndDepthValues)
      {
         double depthInMeters = DEPTH_DISCRETIZATION * pixelAndDepthValue[2];
         Point3D point = new Point3D(depthInMeters,
                                     -(pixelAndDepthValue[0] - cx) / fx * depthInMeters,
                                     -(pixelAndDepthValue[1] - cy) / fy * depthInMeters);
         sensorToWorldTransform.transform(point);
         worldToMapTransform.transform(point);

         int voxelX = (int) Math.rint(point.getX() / VOXEL_SIZE + 0.5 * (MAP_SIZE - 1));
         int voxelY = (int) Math.rint(point.getY() / VOXEL_SIZE + 0.5 * (MAP_SIZE - 1));
         int voxelZ = (int) Math.rint(point.getZ() / VOXEL_SIZE + 0.5 * (MAP_SIZE - 1));
         if (inBounds(voxelX) && inBounds(voxelY) && inBounds(voxelZ))
            expectedOccupiedIndices.add(flatIndex(voxelX, voxelY, voxelZ));
      }
      assertEquals(3, expectedOccupiedIndices.size()); // sanity check that the out-of-map point got dropped

      try (VoxelMapExtractor extractor = new VoxelMapExtractor(MAP_SIZE, VOXEL_SIZE))
      {
         float[] voxelMap = downloadVoxelMap(extractor.getVoxelMap(origin, depthImage).getGpuData());

         float occupancySum = 0.0f;
         for (float voxel : voxelMap)
            occupancySum += voxel;
         assertEquals(expectedOccupiedIndices.size(), (int) occupancySum);

         for (int expectedOccupiedIndex : expectedOccupiedIndices)
            assertEquals(1.0f, voxelMap[expectedOccupiedIndex]);
      }

      depthImage.release();
   }

   @Test
   public void testMapIsClearedBetweenCalls()
   {
      Mat depthMat = new Mat(64, 64, CV_16UC1, new Scalar(1000.0));
      CameraIntrinsics intrinsics = new CameraIntrinsics(64, 64, 100.0, 100.0, 32.0, 32.0);
      RawImage depthImage = RawImage.createWith16BitDepth(depthMat, intrinsics, new RigidBodyTransform(), Instant.now(), 0, DEPTH_DISCRETIZATION);

      try (VoxelMapExtractor extractor = new VoxelMapExtractor(MAP_SIZE, VOXEL_SIZE))
      {
         float[] voxelMap = downloadVoxelMap(extractor.getVoxelMap(new Pose3D(), depthImage).getGpuData());
         float occupancySum = 0.0f;
         for (float voxel : voxelMap)
            occupancySum += voxel;
         assertTrue(occupancySum > 0.0f);

         voxelMap = downloadVoxelMap(extractor.getVoxelMap(new Pose3D()).getGpuData());
         for (float voxel : voxelMap)
            assertEquals(0.0f, voxel);
      }

      depthImage.release();
   }

   private static float[] downloadVoxelMap(FloatPointer deviceVoxelMap)
   {
      int numberOfVoxels = (int) deviceVoxelMap.limit();
      try (FloatPointer hostVoxelMap = new FloatPointer(numberOfVoxels))
      {
         cudaMemcpy(hostVoxelMap, deviceVoxelMap, (long) Float.BYTES * numberOfVoxels, cudaMemcpyDefault);
         float[] voxelMap = new float[numberOfVoxels];
         hostVoxelMap.get(voxelMap);
         return voxelMap;
      }
   }

   private static int voxelIndex(float mapFrameCoordinate)
   {
      float index = mapFrameCoordinate / VOXEL_SIZE + 0.5f * (MAP_SIZE - 1);
      return (int) Math.rint(index);
   }

   private static boolean inBounds(int voxelIndex)
   {
      return voxelIndex >= 0 && voxelIndex < MAP_SIZE;
   }

   private static int flatIndex(int voxelX, int voxelY, int voxelZ)
   {
      return (voxelX * MAP_SIZE + voxelY) * MAP_SIZE + voxelZ;
   }
}
