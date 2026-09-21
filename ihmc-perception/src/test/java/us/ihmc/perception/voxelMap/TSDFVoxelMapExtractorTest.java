package us.ihmc.perception.voxelMap;

import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.cuda.CUDATools;
import us.ihmc.sensors.CameraIntrinsics;

import java.time.Instant;
import java.util.Arrays;

import static org.bytedeco.cuda.global.cudart.cudaMemcpy;
import static org.bytedeco.cuda.global.cudart.cudaMemcpyDefault;
import static org.bytedeco.opencv.global.opencv_core.CV_16UC1;
import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.perception.voxelMap.TSDFVoxelMapExtractor.FREE_SPACE_VALUE;

/**
 * Tests for {@link TSDFVoxelMapExtractor} and {@link TSDFVoxelMappingThread#warpTSDFGrid}.
 *
 * TSDF convention (matching depth_voxel_tsdf.py):
 *   tsdf = clamp((||surface_map|| - ||voxel_center_map||) / truncationDistance, -1, 1)
 * positive = voxel is closer to pelvis than surface (free space side);
 * negative = voxel is farther than surface (occluded side).
 */
public class TSDFVoxelMapExtractorTest
{
   private static final int MAP_SIZE = 8;
   private static final float VOXEL_SIZE = 0.1f;
   private static final float TRUNCATION = 0.2f;
   private static final float DEPTH_DISC = 0.001f;

   // ---- helpers ------------------------------------------------------------

   private static int flat(int x, int y, int z)
   {
      return (x * MAP_SIZE + y) * MAP_SIZE + z;
   }

   /** Half-voxel-centred coordinate: (i - (N-1)/2) * voxelSize */
   private static float coord(int i)
   {
      return (i - (MAP_SIZE - 1) * 0.5f) * VOXEL_SIZE;
   }

   private static float norm(float x, float y, float z)
   {
      return (float) Math.sqrt(x * x + y * y + z * z);
   }

   private static float expectedTSDF(float vx, float vy, float vz,
                                     float sx, float sy, float sz)
   {
      float signed = norm(sx, sy, sz) - norm(vx, vy, vz);
      return Math.max(-1.0f, Math.min(1.0f, signed / TRUNCATION));
   }

   private static float[] downloadGpu(FloatPointer gpuPointer)
   {
      int n = (int) gpuPointer.limit();
      try (FloatPointer host = new FloatPointer(n))
      {
         cudaMemcpy(host, gpuPointer, (long) Float.BYTES * n, cudaMemcpyDefault);
         float[] arr = new float[n];
         host.get(arr);
         return arr;
      }
   }

   private static RawImage flatDepthImage(int imgH, int imgW,
                                          float fx, float fy, float cx, float cy,
                                          int depthRaw,
                                          RigidBodyTransform cameraToWorld)
   {
      Mat mat = new Mat(imgH, imgW, CV_16UC1, new Scalar(depthRaw));
      CameraIntrinsics intr = new CameraIntrinsics(imgH, imgW, fx, fy, cx, cy);
      return RawImage.createWith16BitDepth(mat, intr, cameraToWorld, Instant.now(), 0, DEPTH_DISC);
   }

   // =========================================================================
   // GPU tests (skipped when no CUDA device)
   // =========================================================================

   /**
    * No depth images → all voxels stay at FREE_SPACE_VALUE (+1.0).
    */
   @Test
   public void testNoDepthImagesFreeSpace()
   {
      if (!CUDATools.hasCUDADevice())
         return;

      try (TSDFVoxelMapExtractor ext = new TSDFVoxelMapExtractor(MAP_SIZE, VOXEL_SIZE, TRUNCATION))
      {
         TSDFVoxelMap map = ext.getTSDFVoxelMap(new Pose3D());
         float[] tsdf = downloadGpu(map.getGpuData());
         map.close();

         for (int i = 0; i < tsdf.length; i++)
            assertEquals(FREE_SPACE_VALUE, tsdf[i], 1e-6f, "voxel " + i);
      }
   }

   /**
    * Zero-depth pixels (no measurement) → all voxels stay at FREE_SPACE_VALUE, none marked valid.
    */
   @Test
   public void testZeroDepthIgnored()
   {
      if (!CUDATools.hasCUDADevice())
         return;

      RawImage img = flatDepthImage(40, 64, 100f, 100f, 32f, 20f, 0, new RigidBodyTransform());
      try (TSDFVoxelMapExtractor ext = new TSDFVoxelMapExtractor(MAP_SIZE, VOXEL_SIZE, TRUNCATION))
      {
         TSDFVoxelMap map = ext.getTSDFVoxelMap(new Pose3D(), img);
         float[] tsdf = downloadGpu(map.getGpuData());
         byte[] valid = map.getValidCpuData();
         map.close();

         for (int i = 0; i < tsdf.length; i++)
         {
            assertEquals(FREE_SPACE_VALUE, tsdf[i], 1e-6f, "tsdf[" + i + "]");
            assertEquals(0, valid[i], "valid[" + i + "]");
         }
      }
      img.release();
   }

   /**
    * Identity transforms, uniform depth: voxels projecting into the image get TSDF values matching
    * the CPU pelvis-anchored formula; out-of-image voxels remain FREE_SPACE_VALUE.
    */
   @Test
   public void testIdentityTransformTSDFMath()
   {
      if (!CUDATools.hasCUDADevice())
         return;

      int imgW = 64, imgH = 40;
      float fx = 100f, fy = 100f, cx = imgW / 2.0f, cy = imgH / 2.0f;
      float depthM = 1.0f;
      int depthRaw = (int) (depthM / DEPTH_DISC);

      RawImage img = flatDepthImage(imgH, imgW, fx, fy, cx, cy, depthRaw, new RigidBodyTransform());
      try (TSDFVoxelMapExtractor ext = new TSDFVoxelMapExtractor(MAP_SIZE, VOXEL_SIZE, TRUNCATION))
      {
         TSDFVoxelMap map = ext.getTSDFVoxelMap(new Pose3D(), img);
         float[] tsdf = downloadGpu(map.getGpuData());
         byte[] valid = map.getValidCpuData();
         map.close();

         for (int ix = 0; ix < MAP_SIZE; ix++)
         {
            for (int iy = 0; iy < MAP_SIZE; iy++)
            {
               for (int iz = 0; iz < MAP_SIZE; iz++)
               {
                  float vx = coord(ix); // camera +X = forward in IHMC sensor frame
                  float vy = coord(iy);
                  float vz = coord(iz);

                  if (vx <= 0.05f) // behind or at MIN_CAMERA_DEPTH_M
                     continue;

                  // Project voxel: px = -(vy/vx)*fx + cx,  py = -(vz/vx)*fy + cy
                  int px = Math.round(-vy / vx * fx + cx);
                  int py = Math.round(-vz / vx * fy + cy);
                  if (px < 0 || px >= imgW || py < 0 || py >= imgH)
                     continue;

                  // Observed voxel must be marked valid
                  assertEquals(1, valid[flat(ix, iy, iz)], "valid at (" + ix + "," + iy + "," + iz + ")");

                  // Surface point: pixelDepthToPoint3D(px,py,depthM) in IHMC convention
                  // = (depthM, -(px-cx)/fx*depthM, -(py-cy)/fy*depthM)
                  float sx = depthM;
                  float sy = -(px - cx) / fx * depthM;
                  float sz = -(py - cy) / fy * depthM;

                  float expected = expectedTSDF(vx, vy, vz, sx, sy, sz);
                  assertEquals(expected, tsdf[flat(ix, iy, iz)], 0.01f,
                               "tsdf at (" + ix + "," + iy + "," + iz + ")");
               }
            }
         }
      }
      img.release();
   }

   /**
    * Voxels in front of the surface (closer to pelvis) → positive TSDF.
    * MAP_SIZE=8, surface at 0.4 m: voxels ix=5..7 are at 0.15..0.35 m → in free space.
    */
   @Test
   public void testSignConvention()
   {
      if (!CUDATools.hasCUDADevice())
         return;

      float surfaceM = 0.4f;
      int depthRaw = (int) (surfaceM / DEPTH_DISC);
      int imgW = 64, imgH = 40;

      RawImage img = flatDepthImage(imgH, imgW, 50f, 50f, imgW / 2.0f, imgH / 2.0f, depthRaw, new RigidBodyTransform());
      try (TSDFVoxelMapExtractor ext = new TSDFVoxelMapExtractor(MAP_SIZE, VOXEL_SIZE, TRUNCATION))
      {
         TSDFVoxelMap map = ext.getTSDFVoxelMap(new Pose3D(), img);
         float[] tsdf = downloadGpu(map.getGpuData());
         map.close();

         // Centre Y/Z (index 4, coord = +0.05 m)
         for (int ix = 5; ix <= 7; ix++)
         {
            float vx = coord(ix);
            float val = tsdf[flat(ix, 4, 4)];
            assertTrue(val > 0.0f,
                       "Voxel ix=" + ix + " (vx=" + vx + " < surface=" + surfaceM + ") should be positive, got " + val);
         }
      }
      img.release();
   }

   /**
    * All TSDF values must stay in [-1, 1].
    */
   @Test
   public void testTSDFValueRange()
   {
      if (!CUDATools.hasCUDADevice())
         return;

      int depthRaw = (int) (0.1f / DEPTH_DISC);
      RawImage img = flatDepthImage(40, 64, 100f, 100f, 32f, 20f, depthRaw, new RigidBodyTransform());
      try (TSDFVoxelMapExtractor ext = new TSDFVoxelMapExtractor(MAP_SIZE, VOXEL_SIZE, TRUNCATION))
      {
         TSDFVoxelMap map = ext.getTSDFVoxelMap(new Pose3D(), img);
         float[] tsdf = downloadGpu(map.getGpuData());
         map.close();

         for (int i = 0; i < tsdf.length; i++)
            assertTrue(tsdf[i] >= -1.0f - 1e-5f && tsdf[i] <= 1.0f + 1e-5f,
                       "TSDF out of [-1,1] at " + i + ": " + tsdf[i]);
      }
      img.release();
   }

   /**
    * Multi-camera fusion: minimum TSDF wins.
    * Camera A sees surface at 0.3 m, camera B at 0.1 m (same identity pose).
    * Fused TSDF for each observed voxel must equal min(tsdfA, tsdfB).
    */
   @Test
   public void testMultiCameraMinFusion()
   {
      if (!CUDATools.hasCUDADevice())
         return;

      int imgW = 64, imgH = 40;
      float fx = 50f, fy = 50f, cx = imgW / 2.0f, cy = imgH / 2.0f;
      float surfaceA = 0.3f, surfaceB = 0.1f;

      RawImage imgA = flatDepthImage(imgH, imgW, fx, fy, cx, cy,
                                     (int) (surfaceA / DEPTH_DISC), new RigidBodyTransform());
      RawImage imgB = flatDepthImage(imgH, imgW, fx, fy, cx, cy,
                                     (int) (surfaceB / DEPTH_DISC), new RigidBodyTransform());

      try (TSDFVoxelMapExtractor ext = new TSDFVoxelMapExtractor(MAP_SIZE, VOXEL_SIZE, TRUNCATION))
      {
         TSDFVoxelMap map = ext.getTSDFVoxelMap(new Pose3D(), imgA, imgB);
         float[] fused = downloadGpu(map.getGpuData());
         map.close();

         for (int ix = 1; ix < MAP_SIZE; ix++)
         {
            for (int iy = 0; iy < MAP_SIZE; iy++)
            {
               for (int iz = 0; iz < MAP_SIZE; iz++)
               {
                  float vx = coord(ix), vy = coord(iy), vz = coord(iz);
                  if (vx <= 0.05f) continue;

                  int px = Math.round(-vy / vx * fx + cx);
                  int py = Math.round(-vz / vx * fy + cy);
                  if (px < 0 || px >= imgW || py < 0 || py >= imgH) continue;

                  float syA = -(px - cx) / fx * surfaceA, szA = -(py - cy) / fy * surfaceA;
                  float syB = -(px - cx) / fx * surfaceB, szB = -(py - cy) / fy * surfaceB;
                  float expected = Math.min(expectedTSDF(vx, vy, vz, surfaceA, syA, szA),
                                            expectedTSDF(vx, vy, vz, surfaceB, syB, szB));

                  assertEquals(expected, fused[flat(ix, iy, iz)], 0.02f,
                               "min-fusion at (" + ix + "," + iy + "," + iz + ")");
               }
            }
         }
      }
      imgA.release();
      imgB.release();
   }

   /**
    * GPU buffer re-initialises to FREE_SPACE_VALUE between calls (no stale accumulation).
    */
   @Test
   public void testBufferClearedBetweenCalls()
   {
      if (!CUDATools.hasCUDADevice())
         return;

      RawImage img = flatDepthImage(40, 64, 100f, 100f, 32f, 20f,
                                    (int) (0.5f / DEPTH_DISC), new RigidBodyTransform());
      try (TSDFVoxelMapExtractor ext = new TSDFVoxelMapExtractor(MAP_SIZE, VOXEL_SIZE, TRUNCATION))
      {
         TSDFVoxelMap map1 = ext.getTSDFVoxelMap(new Pose3D(), img);
         float[] first = downloadGpu(map1.getGpuData());
         map1.close();

         TSDFVoxelMap map2 = ext.getTSDFVoxelMap(new Pose3D());
         float[] second = downloadGpu(map2.getGpuData());
         map2.close();

         boolean anyNonFree = false;
         for (float v : first)
            if (Math.abs(v - FREE_SPACE_VALUE) > 0.01f) { anyNonFree = true; break; }
         assertTrue(anyNonFree, "First call should produce non-free TSDF values");

         for (int i = 0; i < second.length; i++)
            assertEquals(FREE_SPACE_VALUE, second[i], 1e-6f, "second call voxel " + i);
      }
      img.release();
   }

   /**
    * Non-identity camera and map origin: TSDF values still in [-1, 1].
    */
   @Test
   public void testNonIdentityTransforms()
   {
      if (!CUDATools.hasCUDADevice())
         return;

      RigidBodyTransform cameraToWorld = new RigidBodyTransform(
            new YawPitchRoll(Math.PI / 2.0, 0.0, 0.0), new Vector3D(0.5, 0.0, 0.0));
      RawImage img = flatDepthImage(40, 64, 100f, 100f, 32f, 20f,
                                    (int) (1.0f / DEPTH_DISC), cameraToWorld);
      Pose3D mapOrigin = new Pose3D(0.3, -0.1, 0.2, 0.4, 0.0, 0.0);

      try (TSDFVoxelMapExtractor ext = new TSDFVoxelMapExtractor(MAP_SIZE, VOXEL_SIZE, TRUNCATION))
      {
         TSDFVoxelMap map = ext.getTSDFVoxelMap(mapOrigin, img);
         float[] tsdf = downloadGpu(map.getGpuData());
         map.close();

         for (int i = 0; i < tsdf.length; i++)
            assertTrue(tsdf[i] >= -1.0f - 1e-5f && tsdf[i] <= 1.0f + 1e-5f,
                       "TSDF out of range at " + i + ": " + tsdf[i]);
      }
      img.release();
   }

   // =========================================================================
   // CPU tests for warpTSDFGrid (no GPU required)
   // =========================================================================

   /**
    * Identity warp (prevOrigin == currOrigin) → grid passes through unchanged.
    */
   @Test
   public void testWarpIdentity()
   {
      int n = MAP_SIZE;
      float[] prev = new float[n * n * n];
      for (int i = 0; i < prev.length; i++)
         prev[i] = (i % 3 == 0) ? -1.0f : (i % 3 == 1) ? 0.0f : 1.0f;

      float[] out = new float[prev.length];
      Pose3D origin = new Pose3D();
      TSDFVoxelMappingThread.warpTSDFGrid(prev, origin, origin, out, MAP_SIZE, VOXEL_SIZE);

      for (int i = 0; i < prev.length; i++)
         assertEquals(prev[i], out[i], 0.001f, "identity warp voxel " + i);
   }

   /**
    * Large translation → all voxels fall outside the previous grid → reset to FREE_SPACE_VALUE.
    */
   @Test
   public void testWarpOutOfBoundsResetToFreeSpace()
   {
      float[] prev = new float[MAP_SIZE * MAP_SIZE * MAP_SIZE];
      Arrays.fill(prev, -0.5f);

      float[] out = new float[prev.length];
      Pose3D prevOrigin = new Pose3D();
      Pose3D currOrigin = new Pose3D(100.0, 100.0, 100.0, 0.0, 0.0, 0.0);
      TSDFVoxelMappingThread.warpTSDFGrid(prev, prevOrigin, currOrigin, out, MAP_SIZE, VOXEL_SIZE);

      for (int i = 0; i < out.length; i++)
         assertEquals(FREE_SPACE_VALUE, out[i], 1e-6f, "OOB voxel " + i);
   }

   /**
    * Translation by exactly one voxel in X: warped[ix,iy,iz] ≈ prev[ix+1,iy,iz].
    * currOrigin is one voxel ahead, so each voxel in the current frame samples one step
    * further forward from the previous frame.
    */
   @Test
   public void testWarpPureTranslationOneVoxel()
   {
      int n = MAP_SIZE;
      float[] prev = new float[n * n * n];
      // Value = ix * 0.1 (varies only in X so interpolation is exact for pure X shift)
      for (int ix = 0; ix < n; ix++)
         for (int iy = 0; iy < n; iy++)
            for (int iz = 0; iz < n; iz++)
               prev[(ix * n + iy) * n + iz] = ix * 0.1f;

      float[] out = new float[prev.length];
      Pose3D prevOrigin = new Pose3D();
      Pose3D currOrigin = new Pose3D(VOXEL_SIZE, 0.0, 0.0, 0.0, 0.0, 0.0);
      TSDFVoxelMappingThread.warpTSDFGrid(prev, prevOrigin, currOrigin, out, MAP_SIZE, VOXEL_SIZE);

      // ix < n-1: warped[ix] = prev[ix+1] = (ix+1)*0.1; ix=n-1 maps OOB → FREE_SPACE_VALUE
      for (int ix = 0; ix < n - 1; ix++)
         for (int iy = 1; iy < n - 1; iy++) // avoid interpolation edge effects
            for (int iz = 1; iz < n - 1; iz++)
               assertEquals((ix + 1) * 0.1f, out[(ix * n + iy) * n + iz], 0.01f,
                            "warp at (" + ix + "," + iy + "," + iz + ")");
   }

   /**
    * No-motion merge: observed voxels use instant TSDF; unobserved voxels retain warped memory.
    */
   @Test
   public void testWarpAndMergeNoMotion()
   {
      int n = MAP_SIZE;
      int count = n * n * n;

      float[] prev = new float[count];
      Arrays.fill(prev, FREE_SPACE_VALUE);

      float[] instant = new float[count];
      byte[] valid = new byte[count];
      int[] obsIdx = {10, 50, 200, 400};
      float[] obsVal = {-0.8f, -0.3f, 0.5f, 0.9f};
      for (int k = 0; k < obsIdx.length; k++)
      {
         instant[obsIdx[k]] = obsVal[k];
         valid[obsIdx[k]] = 1;
      }

      float[] warped = new float[count];
      Pose3D origin = new Pose3D();
      TSDFVoxelMappingThread.warpTSDFGrid(prev, origin, origin, warped, MAP_SIZE, VOXEL_SIZE);

      float[] merged = new float[count];
      for (int i = 0; i < count; i++)
         merged[i] = (valid[i] != 0) ? instant[i] : warped[i];

      for (int k = 0; k < obsIdx.length; k++)
         assertEquals(obsVal[k], merged[obsIdx[k]], 1e-6f, "observed voxel " + obsIdx[k]);

      for (int i = 0; i < count; i++)
      {
         boolean isObs = false;
         for (int idx : obsIdx) if (idx == i) { isObs = true; break; }
         if (!isObs)
            assertEquals(FREE_SPACE_VALUE, merged[i], 0.001f, "unobserved voxel " + i);
      }
   }
}
