package us.ihmc.perception.cuda;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.IntPointer;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;

import java.net.URL;

import static org.bytedeco.cuda.global.cudart.*;

/**
 * Persistent, GPU-resident log-odds voxel grid for 3-D occupancy mapping.
 */
public class CUDAGPUVoxelGrid implements AutoCloseable
{
   // Robot-centric 3-D occupancy crop emitted as the RL observation (Gallant, arXiv:2511.14625).
   // 32 x 32 x 32 voxels at 5 cm = a ±0.8 m box centred on the robot. Dimensions must match
   // VoxelOccupancyPacking (the wire format) and the IsaacLab voxelizer.
   public static final int   CROP_NX          = 32;
   public static final int   CROP_NY          = 32;
   public static final int   CROP_NZ          = 32;
   public static final float CROP_RESOLUTION  = 0.05f;
   /** Total floats in one {@link #extractVoxelObservation} crop (z-as-channel layout). */
   public static final int   VOXEL_CROP_SIZE  = CROP_NX * CROP_NY * CROP_NZ;

   private static final int BLOCK_SIZE_1D     = 256;

   private final CUDAProgram program;
   private final CUDAKernel  kernelUpdateHits;
   private final CUDAKernel  kernelUpdateMisses;
   private final CUDAKernel  kernelClear;
   private final CUDAKernel  kernelDecay;
   private final CUDAKernel  kernelDilate;
   private final CUDAKernel  kernelExtractVoxelCrop;
   private final CUDAKernel  kernelExtractOccupiedVoxels;
   private final CUstream_st stream;

   /** Max occupied voxels read back per frame for whole-grid RDX visualisation. */
   private static final int MAX_RENDER_VOXELS = 1_000_000;

   private final int   nx, ny, nz;
   private final float resolution;
   private final int   totalVoxels;

   private float anchorX, anchorY, anchorZ;

   private final IntPointer   gpuGrid;          // nx*ny*nz ints
   private final FloatPointer gpuVoxelCropOut;  // VOXEL_CROP_SIZE floats (robot-centric occupancy)

   // RigidBodyTransformReadOnly has no array getter, so copy into a concrete transform to extract the 4x4.
   private final RigidBodyTransform baseToWorldCopy = new RigidBodyTransform();
   private final float[] baseTransformArray = new float[16];
   private final FloatPointer voxelCropReadbackPtr;   // VOXEL_CROP_SIZE floats pinned host

   // Whole-grid occupied-voxel readback (for RDX viz): GPU scratch + pinned-host mirrors + atomic count.
   private final FloatPointer gpuOccupiedPositions;         // 3 * MAX_RENDER_VOXELS floats
   private final FloatPointer gpuOccupiedProbabilities;     // MAX_RENDER_VOXELS floats
   private final IntPointer   gpuOccupiedCount;             // single atomic counter (pinned host, device-accessible)
   private final FloatPointer occupiedPositionsReadback;    // pinned host mirror
   private final FloatPointer occupiedProbabilitiesReadback;// pinned host mirror

   /**
    * Creates a GPU voxel grid with configurable dimensions.
    *
    * @param nx         Grid size in X (world, metres: nx × resolution).
    * @param ny         Grid size in Y.
    * @param nz         Grid size in Z.
    * @param resolution Voxel edge length in metres (e.g. 0.05 for 5 cm).
    */
   public CUDAGPUVoxelGrid(int nx, int ny, int nz, float resolution)
   {
      this.nx         = nx;
      this.ny         = ny;
      this.nz         = nz;
      this.resolution = resolution;
      this.totalVoxels = nx * ny * nz;

      URL kernelURL    = getClass().getResource("GPUVoxelGrid.cu");
      URL mathUtilsURL = getClass().getResource("MathUtils.cuh");
      try
      {
         program = new CUDAProgram(kernelURL, mathUtilsURL);
         kernelUpdateHits   = program.loadKernel("updateLogOddsHits");
         kernelUpdateMisses = program.loadKernel("updateLogOddsMisses");
         kernelClear        = program.loadKernel("clearGrid");
         kernelDecay        = program.loadKernel("decayLogOdds");
         kernelDilate       = program.loadKernel("dilateOccupancyXY");
         kernelExtractVoxelCrop = program.loadKernel("extractVoxelOccupancyCrop");
         kernelExtractOccupiedVoxels = program.loadKernel("extractOccupiedVoxelsWithProbability");
      }
      catch (Exception e)
      {
         throw new RuntimeException("Failed to compile GPUVoxelGrid CUDA kernels", e);
      }

      stream = CUDAStreamManager.getStream();

      gpuGrid = new IntPointer();
      CUDATools.mallocAsync(gpuGrid, (long) totalVoxels, stream);
      gpuGrid.limit((long) totalVoxels);

      gpuVoxelCropOut = new FloatPointer();
      CUDATools.mallocAsync(gpuVoxelCropOut, (long) VOXEL_CROP_SIZE, stream);
      gpuVoxelCropOut.limit((long) VOXEL_CROP_SIZE);

      voxelCropReadbackPtr = new FloatPointer();
      CUDATools.mallocHost(voxelCropReadbackPtr, (long) VOXEL_CROP_SIZE);

      gpuOccupiedPositions = new FloatPointer();
      CUDATools.mallocAsync(gpuOccupiedPositions, 3L * MAX_RENDER_VOXELS, stream);
      gpuOccupiedPositions.limit(3L * MAX_RENDER_VOXELS);
      gpuOccupiedProbabilities = new FloatPointer();
      CUDATools.mallocAsync(gpuOccupiedProbabilities, (long) MAX_RENDER_VOXELS, stream);
      gpuOccupiedProbabilities.limit((long) MAX_RENDER_VOXELS);
      gpuOccupiedCount = new IntPointer();
      CUDATools.mallocHost(gpuOccupiedCount, 1L);
      occupiedPositionsReadback = new FloatPointer();
      CUDATools.mallocHost(occupiedPositionsReadback, 3L * MAX_RENDER_VOXELS);
      occupiedProbabilitiesReadback = new FloatPointer();
      CUDATools.mallocHost(occupiedProbabilitiesReadback, (long) MAX_RENDER_VOXELS);

      CUDATools.checkCUDAError(cudaStreamSynchronize(stream));

      clear();
   }

   /**
    * Sets the world-frame anchor (position of voxel [0][0][0]).
    * Call once at episode start, centred half a grid width behind and beside the robot.
    */
   public void setAnchor(float worldX, float worldY, float worldZ)
   {
      anchorX = worldX;
      anchorY = worldY;
      anchorZ = worldZ;
   }

   /**
    * Centres the grid on the given robot position.
    * Z anchor covers [robotZ − nz*res*0.25, robotZ + nz*res*0.75].
    */
   public void centreOnRobot(float robotX, float robotY, float robotZ)
   {
      setAnchor(robotX - nx * resolution * 0.5f,
                robotY - ny * resolution * 0.5f,
                robotZ - nz * resolution * 0.25f);
   }

   /** Null device pointer used when hits are inserted without per-point confidence weights. */
   private static final FloatPointer NULL_POINTER = new FloatPointer();

   /**
    * Inserts a GPU-resident float3 point cloud as hit observations without a CPU round-trip.
    * Every point contributes a full-strength log-odds hit.
    *
    * @param gpuPoints GPU float3 buffer (world-frame points).
    * @param numPoints Number of points in the buffer.
    */
   public void updateHits(FloatPointer gpuPoints, int numPoints)
   {
      updateHits(gpuPoints, NULL_POINTER, numPoints);
   }

   /**
    * Inserts a GPU-resident float3 point cloud as confidence-weighted hit observations.
    *
    * <p>Each point's log-odds hit is scaled by its weight in {@code gpuPointWeights} (0–1), so
    * distant, higher-variance measurements nudge occupancy less than nearby ones. This lets two
    * cameras be weighted differently and makes the nearer-ranging camera dominate the fused map.
    *
    * @param gpuPoints       GPU float3 buffer (world-frame points).
    * @param gpuPointWeights GPU float buffer of per-point weights parallel to {@code gpuPoints};
    *                        pass a null pointer for full-strength hits.
    * @param numPoints       Number of points in the buffers.
    */
   public void updateHits(FloatPointer gpuPoints, FloatPointer gpuPointWeights, int numPoints)
   {
      if (numPoints <= 0 || gpuPoints.isNull())
         return;

      int gridSize = (numPoints + BLOCK_SIZE_1D - 1) / BLOCK_SIZE_1D;
      dim3 block = new dim3(BLOCK_SIZE_1D, 1, 1);
      dim3 grid  = new dim3(gridSize, 1, 1);

      kernelUpdateHits
            .withPointer(gpuGrid)
            .withInt(nx).withInt(ny).withInt(nz)
            .withFloat(resolution)
            .withFloat(anchorX).withFloat(anchorY).withFloat(anchorZ)
            .withPointer(gpuPoints)
            .withPointer(gpuPointWeights == null ? NULL_POINTER : gpuPointWeights)
            .withInt(numPoints)
            .run(stream, grid, block, 0);

      block.close();
      grid.close();
   }

   /**
    * Free-space carving: decrements log-odds along rays from {@code sensorOrigin} to each point.
    * Removes ghost obstacles from reflections and motion blur, at ~5× the cost of hits-only.
    * Every point contributes a full-strength miss along its ray.
    */
   public void updateMisses(FloatPointer gpuPoints, int numPoints,
                             float sensorOriginX, float sensorOriginY, float sensorOriginZ)
   {
      updateMisses(gpuPoints, NULL_POINTER, numPoints, sensorOriginX, sensorOriginY, sensorOriginZ);
   }

   /**
    * Confidence-weighted free-space carving: as {@link #updateMisses(FloatPointer, int, float, float, float)},
    * but each ray's miss log-odds is scaled by {@code gpuPointWeights} (0-1], matching the same
    * per-point confidence used for that point's hit. Without this, a weak/distant hit (already
    * discounted for range) gets out-voted by full-strength misses from nearer, unrelated rays
    * crossing the same voxel, systematically eroding legitimate far-away detections.
    *
    * @param gpuPointWeights GPU float buffer of per-point weights parallel to {@code gpuPoints};
    *                        pass a null pointer for full-strength misses.
    */
   public void updateMisses(FloatPointer gpuPoints, FloatPointer gpuPointWeights, int numPoints,
                             float sensorOriginX, float sensorOriginY, float sensorOriginZ)
   {
      if (numPoints <= 0 || gpuPoints.isNull())
         return;

      int gridSize = (numPoints + BLOCK_SIZE_1D - 1) / BLOCK_SIZE_1D;
      dim3 block = new dim3(BLOCK_SIZE_1D, 1, 1);
      dim3 grid  = new dim3(gridSize, 1, 1);

      kernelUpdateMisses
            .withPointer(gpuGrid)
            .withInt(nx).withInt(ny).withInt(nz)
            .withFloat(resolution)
            .withFloat(anchorX).withFloat(anchorY).withFloat(anchorZ)
            .withPointer(gpuPoints)
            .withPointer(gpuPointWeights == null ? NULL_POINTER : gpuPointWeights)
            .withInt(numPoints)
            .withFloat(sensorOriginX).withFloat(sensorOriginY).withFloat(sensorOriginZ)
            .run(stream, grid, block, 0);

      block.close();
      grid.close();
   }


   /**
    * Extracts a robot-centric, yaw-aligned 3-D binary occupancy crop ({@value #VOXEL_CROP_SIZE} floats,
    * z-as-channel layout) as the RL observation — the voxel-grid input from Gallant (arXiv:2511.14625).
    *
    * The persistent grid is world-anchored (it remembers geometry mapped before a fall); this samples a
    * {@link #CROP_NX}×{@link #CROP_NY}×{@link #CROP_NZ} box centred on the robot, rotated by yaw only so
    * the crop stays gravity-aligned even when the robot is prone. Each voxel is a continuous occupancy
    * probability (sigmoid of log-odds): confidently occupied → 1.0, confidently free → 0.0, unknown/
    * no-evidence/out-of-bounds → 0.5 (see {@code extractVoxelOccupancyCrop} in GPUVoxelGrid.cu).
    *
    * @param baseToWorld   Current robot base pose in world frame.
    * @param occupancyToPack Target float[] — the RL observation vector.
    * @param startIndex    Offset into {@code occupancyToPack}.
    */
   public void extractVoxelObservation(RigidBodyTransformReadOnly baseToWorld, float[] occupancyToPack, int startIndex)
   {
      baseToWorldCopy.set(baseToWorld);
      baseToWorldCopy.get(baseTransformArray); // row-major 4x4

      float robotX = baseTransformArray[3];
      float robotY = baseTransformArray[7];
      float robotZ = baseTransformArray[11];
      float yaw    = (float) Math.atan2(baseTransformArray[4], baseTransformArray[0]); // atan2(r10, r00)
      float cosYaw = (float) Math.cos(yaw);
      float sinYaw = (float) Math.sin(yaw);

      int gridSize = (VOXEL_CROP_SIZE + BLOCK_SIZE_1D - 1) / BLOCK_SIZE_1D;
      dim3 block = new dim3(BLOCK_SIZE_1D, 1, 1);
      dim3 grid  = new dim3(gridSize, 1, 1);

      kernelExtractVoxelCrop
            .withPointer(gpuGrid)
            .withInt(nx).withInt(ny).withInt(nz)
            .withFloat(resolution)
            .withFloat(anchorX).withFloat(anchorY).withFloat(anchorZ)
            .withFloat(robotX).withFloat(robotY).withFloat(robotZ)
            .withFloat(cosYaw).withFloat(sinYaw)
            .withInt(CROP_NX).withInt(CROP_NY).withInt(CROP_NZ)
            .withFloat(CROP_RESOLUTION)
            .withPointer(gpuVoxelCropOut)
            .run(stream, grid, block, 0);

      block.close();
      grid.close();

      CUDATools.memcpyAsync(voxelCropReadbackPtr, gpuVoxelCropOut, VOXEL_CROP_SIZE, stream);
      CUDATools.checkCUDAError(cudaStreamSynchronize(stream));
      voxelCropReadbackPtr.get(occupancyToPack, startIndex, VOXEL_CROP_SIZE);
   }

   /** Whole-grid occupied-voxel snapshot for visualisation: parallel world positions and occupancy probabilities. */
   public record OccupiedVoxels(float[] positions, float[] probabilities, int count) { }

   /**
    * Reads back every occupied-leaning voxel (log-odds &gt; threshold) of the entire world grid as
    * world-frame centres and occupancy probabilities (sigmoid of log-odds), for rendering the full
    * fused map in RDX coloured by confidence. Capped at {@value #MAX_RENDER_VOXELS} voxels.
    */
   public OccupiedVoxels extractOccupiedVoxels()
   {
      gpuOccupiedCount.put(0);

      int gridSize = (totalVoxels + BLOCK_SIZE_1D - 1) / BLOCK_SIZE_1D;
      dim3 block = new dim3(BLOCK_SIZE_1D, 1, 1);
      dim3 grid  = new dim3(gridSize, 1, 1);

      kernelExtractOccupiedVoxels
            .withPointer(gpuGrid)
            .withInt(nx).withInt(ny).withInt(nz)
            .withFloat(resolution)
            .withFloat(anchorX).withFloat(anchorY).withFloat(anchorZ)
            .withPointer(gpuOccupiedPositions)
            .withPointer(gpuOccupiedProbabilities)
            .withPointer(gpuOccupiedCount)
            .withInt(MAX_RENDER_VOXELS)
            .run(stream, grid, block, 0);

      block.close();
      grid.close();

      CUDATools.checkCUDAError(cudaStreamSynchronize(stream));
      int count = Math.min(gpuOccupiedCount.get(), MAX_RENDER_VOXELS);
      if (count <= 0)
         return new OccupiedVoxels(new float[0], new float[0], 0);

      CUDATools.memcpyAsync(occupiedPositionsReadback, gpuOccupiedPositions, 3L * count, stream);
      CUDATools.memcpyAsync(occupiedProbabilitiesReadback, gpuOccupiedProbabilities, (long) count, stream);
      CUDATools.checkCUDAError(cudaStreamSynchronize(stream));

      float[] positions = new float[3 * count];
      float[] probabilities = new float[count];
      occupiedPositionsReadback.get(positions, 0, 3 * count);
      occupiedProbabilitiesReadback.get(probabilities, 0, count);
      return new OccupiedVoxels(positions, probabilities, count);
   }

   /**
    * Resets all voxels to unknown (log-odds = 0). Call at episode start or when the robot is teleported.
    */
   public void clear()
   {
      int gridSize = (totalVoxels + BLOCK_SIZE_1D - 1) / BLOCK_SIZE_1D;
      dim3 block = new dim3(BLOCK_SIZE_1D, 1, 1);
      dim3 grid  = new dim3(gridSize, 1, 1);
      kernelClear.withPointer(gpuGrid).withInt(totalVoxels).run(stream, grid, block, 0);
      block.close();
      grid.close();
      CUDATools.checkCUDAError(cudaStreamSynchronize(stream));
   }

   /**
    * Fills single-voxel holes by XY dilation. Call once per frame after all hit updates.
    *
    * @param minNeighbours  Minimum occupied 4-connected XY neighbours required to fill (2 is conservative).
    * @param fillLogOdds    Log-odds assigned to filled voxels; use below LOG_ODDS_MAX so miss updates can override.
    */
   public void dilate(int minNeighbours, int fillLogOdds)
   {
      int gridSize = (totalVoxels + BLOCK_SIZE_1D - 1) / BLOCK_SIZE_1D;
      dim3 block = new dim3(BLOCK_SIZE_1D, 1, 1);
      dim3 grid  = new dim3(gridSize, 1, 1);
      kernelDilate
            .withPointer(gpuGrid)
            .withInt(nx).withInt(ny).withInt(nz)
            .withInt(minNeighbours)
            .withInt(fillLogOdds)
            .run(stream, grid, block, 0);
      block.close();
      grid.close();
   }

   /**
    * Decays every voxel's log-odds toward 0 by {@code logOddsPerDecay} fixed-point units per call.
    */
   public void decay(int logOddsPerDecay)
   {
      int gridSize = (totalVoxels + BLOCK_SIZE_1D - 1) / BLOCK_SIZE_1D;
      dim3 block = new dim3(BLOCK_SIZE_1D, 1, 1);
      dim3 grid  = new dim3(gridSize, 1, 1);
      kernelDecay.withPointer(gpuGrid).withInt(totalVoxels).withInt(logOddsPerDecay)
                 .run(stream, grid, block, 0);
      block.close();
      grid.close();
   }

   public IntPointer getGpuGrid()    { return gpuGrid; }
   public CUstream_st getStream()    { return stream; }
   public int getNx()                { return nx; }
   public int getNy()                { return ny; }
   public int getNz()                { return nz; }
   public float getResolution()      { return resolution; }
   public float getAnchorX()         { return anchorX; }
   public float getAnchorY()         { return anchorY; }
   public float getAnchorZ()         { return anchorZ; }

   @Override
   public void close()
   {
      CUDATools.checkCUDAError(cudaFreeAsync(gpuGrid, stream));
      CUDATools.checkCUDAError(cudaFreeAsync(gpuVoxelCropOut, stream));
      CUDATools.checkCUDAError(cudaFreeAsync(gpuOccupiedPositions, stream));
      CUDATools.checkCUDAError(cudaFreeAsync(gpuOccupiedProbabilities, stream));
      CUDATools.freeHost(voxelCropReadbackPtr);
      CUDATools.freeHost(gpuOccupiedCount);
      CUDATools.freeHost(occupiedPositionsReadback);
      CUDATools.freeHost(occupiedProbabilitiesReadback);
      CUDATools.checkCUDAError(cudaStreamSynchronize(stream));
      kernelUpdateHits.close();
      kernelUpdateMisses.close();
      kernelClear.close();
      kernelDecay.close();
      kernelDilate.close();
      kernelExtractVoxelCrop.close();
      kernelExtractOccupiedVoxels.close();
      program.close();
      CUDAStreamManager.releaseStream(stream);
   }
}
