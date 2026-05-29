package us.ihmc.perception.cuda;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencv.opencv_core.GpuMat;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.perception.RawImage;

import java.net.URL;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import static org.bytedeco.cuda.global.cudart.*;

/**
 * Extracts world-frame 3D point clouds and sampled RGB colours from both ZED X Mini depth cameras
 * in a single CUDA kernel launch. Depth and left-colour images are assumed to be aligned
 * (same resolution and pixel correspondence), which is the ZED SDK default.
 *
 * <p>Camera 0 = stepping camera (belly-mounted, 4 mm lens).
 * Camera 1 = experimental camera (head-mounted, 2.2 mm lens).
 */
public class CUDAFusedVoxelMapExtractor implements AutoCloseable
{
   /** All data returned for one frame pair. */
   public record DualCameraPoints(
         List<Point3D32> steppingPoints,
         List<float[]>   steppingColors,       // parallel RGB [0,1] per point
         List<Point3D32> experimentalPoints,
         List<float[]>   experimentalColors,    // parallel RGB [0,1] per point
         float steppingOriginX, float steppingOriginY, float steppingOriginZ,
         float experimentalOriginX, float experimentalOriginY, float experimentalOriginZ)
   {
      public static final DualCameraPoints EMPTY =
            new DualCameraPoints(List.of(), List.of(), List.of(), List.of(), 0, 0, 0, 0, 0, 0);
   }

   private static final int BLOCK_SIZE = 16;

   // ── CUDA infrastructure ────────────────────────────────────────────────────
   private final CUDAProgram program;
   private final CUDAKernel  kernel;
   private final CUstream_st stream;

   // ── Host-pinned memory for transforms and atomic counters ─────────────────
   private final FloatPointer transform0Ptr = new FloatPointer();
   private final FloatPointer transform1Ptr = new FloatPointer();
   private final IntPointer   size0Ptr      = new IntPointer();
   private final IntPointer   size1Ptr      = new IntPointer();

   // ── Transform scratch ─────────────────────────────────────────────────────
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final float[] transformArray0 = new float[16];
   private final float[] transformArray1 = new float[16];

   // ── GPU output: world points (float3) ─────────────────────────────────────
   private final FloatPointer gpuPointCloud0 = new FloatPointer();
   private final FloatPointer gpuPointCloud1 = new FloatPointer();

   // ── GPU output: colours (float3, normalised RGB) ──────────────────────────
   private final FloatPointer gpuColorCloud0 = new FloatPointer();
   private final FloatPointer gpuColorCloud1 = new FloatPointer();

   public CUDAFusedVoxelMapExtractor()
   {
      URL kernelURL   = getClass().getResource("FusedVoxelMap.cu");
      URL mathUtilsURL = getClass().getResource("MathUtils.cuh");

      try
      {
         program = new CUDAProgram(kernelURL,
                                   CUDATools.getUtilsFile(),
                                   CUDATools.getPerceptionUtilsFile(),
                                   mathUtilsURL);
         kernel  = program.loadKernel("extractFusedPointCloud");
      }
      catch (Exception e)
      {
         throw new RuntimeException("Failed to compile FusedVoxelMap CUDA kernel", e);
      }

      stream = CUDAStreamManager.getStream();

      CUDATools.checkCUDAError(cudaMallocHost(transform0Ptr, 16L * Float.BYTES));
      CUDATools.checkCUDAError(cudaMallocHost(transform1Ptr, 16L * Float.BYTES));
      CUDATools.checkCUDAError(cudaMallocHost(size0Ptr, Integer.BYTES));
      CUDATools.checkCUDAError(cudaMallocHost(size1Ptr, Integer.BYTES));
   }

   /**
    * Extracts coloured world-frame point clouds from both depth cameras with range limits.
    *
    * @param steppingDepth        Depth image from the stepping camera (belly, 4 mm).
    * @param steppingColor        Left colour image aligned with {@code steppingDepth}.
    * @param experimentalDepth    Depth image from the experimental camera (head, 2.2 mm).
    * @param experimentalColor    Left colour image aligned with {@code experimentalDepth}.
    * @param minRangeStepping     Minimum valid depth in metres for the stepping camera.
    * @param maxRangeStepping     Maximum valid depth in metres for the stepping camera.
    * @param minRangeExperimental Minimum valid depth in metres for the experimental camera.
    * @param maxRangeExperimental Maximum valid depth in metres for the experimental camera.
    */
   public DualCameraPoints extractPointClouds(RawImage steppingDepth,
                                              RawImage steppingColor,
                                              RawImage experimentalDepth,
                                              RawImage experimentalColor,
                                              float minRangeStepping,
                                              float maxRangeStepping,
                                              float minRangeExperimental,
                                              float maxRangeExperimental)
   {
      if (steppingDepth.get() == null || steppingColor.get() == null
            || experimentalDepth.get() == null || experimentalColor.get() == null)
         return DualCameraPoints.EMPTY;

      // ── Ensure GPU point + colour buffers are large enough ────────────────
      long max0 = 3L * steppingDepth.getWidth()     * steppingDepth.getHeight();
      long max1 = 3L * experimentalDepth.getWidth() * experimentalDepth.getHeight();

      ensureCapacity(gpuPointCloud0, max0);
      ensureCapacity(gpuColorCloud0, max0);
      ensureCapacity(gpuPointCloud1, max1);
      ensureCapacity(gpuColorCloud1, max1);

      // ── Pack transforms, extract camera origins ───────────────────────────
      tempTransform.set(steppingDepth.getTransformToWorld());
      tempTransform.get(transformArray0);
      transform0Ptr.put(transformArray0);
      float ox0 = transformArray0[3], oy0 = transformArray0[7], oz0 = transformArray0[11];

      tempTransform.set(experimentalDepth.getTransformToWorld());
      tempTransform.get(transformArray1);
      transform1Ptr.put(transformArray1);
      float ox1 = transformArray1[3], oy1 = transformArray1[7], oz1 = transformArray1[11];

      // ── Reset atomic counters ─────────────────────────────────────────────
      size0Ptr.put(0);
      size1Ptr.put(0);

      // ── Colour GpuMats (upload from CPU if not already on GPU) ────────────
      GpuMat colorGpu0 = steppingColor.getGpuImageMat();
      GpuMat colorGpu1 = experimentalColor.getGpuImageMat();

      // ── Grid: z=2 for the two cameras ────────────────────────────────────
      int maxW = Math.max(steppingDepth.getWidth(), experimentalDepth.getWidth());
      int maxH = Math.max(steppingDepth.getHeight(), experimentalDepth.getHeight());
      dim3 blockSize = new dim3(BLOCK_SIZE, BLOCK_SIZE, 1);
      int gx = Math.max(1, (maxW + BLOCK_SIZE - 1) / (BLOCK_SIZE * 2));
      int gy = Math.max(1, (maxH + BLOCK_SIZE - 1) / (BLOCK_SIZE * 2));
      dim3 gridSize = new dim3(gx, gy, 2);

      // ── Launch kernel ─────────────────────────────────────────────────────
      kernel
            // Camera 0 depth
            .withPointer(steppingDepth.getCUDADataPointer())
            .withLong(steppingDepth.getGpuImageMat().step())
            .withInt(steppingDepth.getWidth())
            .withInt(steppingDepth.getHeight())
            .withFloat(steppingDepth.getFocalLengthX())
            .withFloat(steppingDepth.getFocalLengthY())
            .withFloat(steppingDepth.getPrincipalPointX())
            .withFloat(steppingDepth.getPrincipalPointY())
            .withFloat(steppingDepth.getDepthDiscretization())
            .withPointer(transform0Ptr)
            .withFloat(minRangeStepping)
            .withFloat(maxRangeStepping)
            // Camera 0 colour
            .withPointer(colorGpu0.data())
            .withLong(colorGpu0.step())
            .withInt(colorGpu0.channels())
            // Camera 0 output
            .withPointer(gpuPointCloud0)
            .withPointer(gpuColorCloud0)
            .withPointer(size0Ptr)
            // Camera 1 depth
            .withPointer(experimentalDepth.getCUDADataPointer())
            .withLong(experimentalDepth.getGpuImageMat().step())
            .withInt(experimentalDepth.getWidth())
            .withInt(experimentalDepth.getHeight())
            .withFloat(experimentalDepth.getFocalLengthX())
            .withFloat(experimentalDepth.getFocalLengthY())
            .withFloat(experimentalDepth.getPrincipalPointX())
            .withFloat(experimentalDepth.getPrincipalPointY())
            .withFloat(experimentalDepth.getDepthDiscretization())
            .withPointer(transform1Ptr)
            .withFloat(minRangeExperimental)
            .withFloat(maxRangeExperimental)
            // Camera 1 colour
            .withPointer(colorGpu1.data())
            .withLong(colorGpu1.step())
            .withInt(colorGpu1.channels())
            // Camera 1 output
            .withPointer(gpuPointCloud1)
            .withPointer(gpuColorCloud1)
            .withPointer(size1Ptr)
            .run(stream, gridSize, blockSize, 0);

      CUDATools.checkCUDAError(cudaStreamSynchronize(stream));

      int n0 = size0Ptr.get();
      int n1 = size1Ptr.get();

      float[] raw0 = downloadFloats(gpuPointCloud0, n0);
      float[] col0 = downloadFloats(gpuColorCloud0, n0);
      float[] raw1 = downloadFloats(gpuPointCloud1, n1);
      float[] col1 = downloadFloats(gpuColorCloud1, n1);

      blockSize.close();
      gridSize.close();
      steppingDepth.release();
      steppingColor.release();
      experimentalDepth.release();
      experimentalColor.release();

      return new DualCameraPoints(
            toPoints(raw0, n0), toColors(col0, n0),
            toPoints(raw1, n1), toColors(col1, n1),
            ox0, oy0, oz0, ox1, oy1, oz1);
   }

   // ── Private helpers ───────────────────────────────────────────────────────

   private void ensureCapacity(FloatPointer ptr, long nFloats)
   {
      if (ptr.isNull() || ptr.limit() < nFloats)
      {
         if (!ptr.isNull())
            CUDATools.checkCUDAError(cudaFreeAsync(ptr, stream));
         CUDATools.mallocAsync(ptr, nFloats, stream);
         ptr.limit(nFloats);
      }
   }

   private float[] downloadFloats(FloatPointer gpuBuf, int pointCount)
   {
      if (pointCount == 0)
         return new float[0];
      long n = 3L * pointCount;
      FloatPointer cpu = new FloatPointer(n);
      CUDATools.memcpyAsync(cpu, gpuBuf, n, stream);
      CUDATools.checkCUDAError(cudaStreamSynchronize(stream));
      float[] arr = new float[(int) n];
      cpu.get(arr);
      cpu.close();
      return arr;
   }

   private static List<Point3D32> toPoints(float[] arr, int count)
   {
      List<Point3D32> list = new ArrayList<>(count);
      for (int i = 0; i < count; i++)
         list.add(new Point3D32(arr[3 * i], arr[3 * i + 1], arr[3 * i + 2]));
      return list;
   }

   private static List<float[]> toColors(float[] arr, int count)
   {
      List<float[]> list = new ArrayList<>(count);
      for (int i = 0; i < count; i++)
         list.add(new float[]{arr[3 * i], arr[3 * i + 1], arr[3 * i + 2]});
      return list;
   }

   @Override
   public void close()
   {
      CUDATools.checkCUDAError(cudaFreeHost(transform0Ptr));
      CUDATools.checkCUDAError(cudaFreeHost(transform1Ptr));
      CUDATools.checkCUDAError(cudaFreeHost(size0Ptr));
      CUDATools.checkCUDAError(cudaFreeHost(size1Ptr));
      freeIfAllocated(gpuPointCloud0);
      freeIfAllocated(gpuColorCloud0);
      freeIfAllocated(gpuPointCloud1);
      freeIfAllocated(gpuColorCloud1);
      kernel.close();
      program.close();
      CUDAStreamManager.releaseStream(stream);
   }

   private void freeIfAllocated(FloatPointer ptr)
   {
      if (!ptr.isNull())
         CUDATools.checkCUDAError(cudaFreeAsync(ptr, stream));
   }
}
