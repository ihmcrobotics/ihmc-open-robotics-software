package us.ihmc.perception.cuda;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.IntPointer;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;

import java.net.URL;

import static org.bytedeco.cuda.global.cudart.*;

/**
 * Persistent, GPU-resident log-odds voxel grid for 3-D occupancy mapping.
 */
public class CUDAGPUVoxelGrid implements AutoCloseable
{
   public static final int   SCAN_NX          = 17;
   public static final int   SCAN_NY          = 11;
   public static final int   NUM_SCAN_POINTS  = SCAN_NX * SCAN_NY;
   private static final float SCAN_X_MIN      = -0.8f;
   private static final float SCAN_Y_MIN      = -0.5f;
   private static final float SCAN_STEP       = 0.10f;

   private static final int BLOCK_SIZE_1D     = 256;

   private final CUDAProgram program;
   private final CUDAKernel  kernelUpdateHits;
   private final CUDAKernel  kernelUpdateMisses;
   private final CUDAKernel  kernelClear;
   private final CUDAKernel  kernelDecay;
   private final CUDAKernel  kernelDilate;
   private final CUDAKernel  kernelExtractScan;
   private final CUstream_st stream;

   private final int   nx, ny, nz;
   private final float resolution;
   private final int   totalVoxels;

   private float anchorX, anchorY, anchorZ;

   private final IntPointer   gpuGrid;        // nx*ny*nz ints
   private final FloatPointer gpuScanX;       // NUM_SCAN_POINTS floats
   private final FloatPointer gpuScanY;       // NUM_SCAN_POINTS floats
   private final FloatPointer gpuHeightsOut;  // NUM_SCAN_POINTS floats

   private final FloatPointer gpuBaseTransform;       // 16 floats on device
   private final FloatPointer baseTransformStaging;   // 16 floats pinned host, staging for upload
   private final FloatPointer heightsReadbackPtr;     // NUM_SCAN_POINTS floats pinned host

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
         kernelExtractScan  = program.loadKernel("extractHeightScan");
      }
      catch (Exception e)
      {
         throw new RuntimeException("Failed to compile GPUVoxelGrid CUDA kernels", e);
      }

      stream = CUDAStreamManager.getStream();

      gpuGrid = new IntPointer();
      CUDATools.mallocAsync(gpuGrid, (long) totalVoxels, stream);
      gpuGrid.limit((long) totalVoxels);

      gpuHeightsOut = new FloatPointer();
      CUDATools.mallocAsync(gpuHeightsOut, (long) NUM_SCAN_POINTS, stream);
      gpuHeightsOut.limit((long) NUM_SCAN_POINTS);

      float[] patternX = buildScanPattern(true);
      float[] patternY = buildScanPattern(false);

      gpuScanX = new FloatPointer(NUM_SCAN_POINTS);
      gpuScanY = new FloatPointer(NUM_SCAN_POINTS);
      CUDATools.mallocAsync(gpuScanX, (long) NUM_SCAN_POINTS, stream);
      CUDATools.mallocAsync(gpuScanY, (long) NUM_SCAN_POINTS, stream);
      gpuScanX.limit((long) NUM_SCAN_POINTS);
      gpuScanY.limit((long) NUM_SCAN_POINTS);

      FloatPointer tmpX = new FloatPointer(patternX);
      FloatPointer tmpY = new FloatPointer(patternY);
      CUDATools.memcpyAsync(gpuScanX, tmpX, NUM_SCAN_POINTS, stream);
      CUDATools.memcpyAsync(gpuScanY, tmpY, NUM_SCAN_POINTS, stream);
      tmpX.close();
      tmpY.close();

      gpuBaseTransform = new FloatPointer();
      CUDATools.mallocAsync(gpuBaseTransform, 16L, stream);
      gpuBaseTransform.limit(16L);

      baseTransformStaging = new FloatPointer();
      CUDATools.mallocHost(baseTransformStaging, 16L);

      heightsReadbackPtr = new FloatPointer();
      CUDATools.mallocHost(heightsReadbackPtr, (long) NUM_SCAN_POINTS);

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

   /**
    * Inserts a GPU-resident float3 point cloud as hit observations without a CPU round-trip.
    *
    * @param gpuPoints GPU float3 buffer (world-frame points).
    * @param numPoints Number of points in the buffer.
    */
   public void updateHits(FloatPointer gpuPoints, int numPoints)
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
            .withInt(numPoints)
            .run(stream, grid, block, 0);

      block.close();
      grid.close();
   }

   /**
    * Uploads CPU-side points to a temporary GPU buffer and calls {@link #updateHits(FloatPointer, int)}.
    */
   public void updateHitsFromList(java.util.List<us.ihmc.euclid.tuple3D.Point3D32> points)
   {
      if (points == null || points.isEmpty())
         return;

      int count     = points.size();
      long nFloats  = 3L * count;
      float[] arr   = new float[(int) nFloats];
      for (int i = 0; i < count; i++)
      {
         arr[3 * i]     = points.get(i).getX32();
         arr[3 * i + 1] = points.get(i).getY32();
         arr[3 * i + 2] = points.get(i).getZ32();
      }

      FloatPointer cpuPtr = new FloatPointer(arr);
      FloatPointer gpuPtr = new FloatPointer();
      CUDATools.mallocAsync(gpuPtr, nFloats, stream);
      gpuPtr.limit(nFloats);
      CUDATools.memcpyAsync(gpuPtr, cpuPtr, nFloats, stream);
      updateHits(gpuPtr, count);
      CUDATools.checkCUDAError(cudaStreamSynchronize(stream));
      CUDATools.checkCUDAError(cudaFreeAsync(gpuPtr, stream));
      cpuPtr.close();
   }

   /**
    * Free-space carving: decrements log-odds along rays from {@code sensorOrigin} to each point.
    * Removes ghost obstacles from reflections and motion blur, at ~5× the cost of hits-only.
    */
   public void updateMisses(FloatPointer gpuPoints, int numPoints,
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
            .withInt(numPoints)
            .withFloat(sensorOriginX).withFloat(sensorOriginY).withFloat(sensorOriginZ)
            .run(stream, grid, block, 0);

      block.close();
      grid.close();
   }

   /**
    * Extracts a 187-element height-scan observation vector into {@code heightsToPack} at {@code startIndex}.
    *
    * @param baseToWorld  Current robot base pose in world frame.
    * @param heightsToPack Target float[] — the RL observation vector.
    * @param startIndex   Offset into {@code heightsToPack}.
    */
   public void extractHeightScan(RigidBodyTransformReadOnly baseToWorld,
                                  float[] heightsToPack, int startIndex)
   {
      float[] arr = new float[16];
      baseToWorld.get(arr); // row-major 4x4
      baseTransformStaging.put(arr);
      CUDATools.memcpyAsync(gpuBaseTransform, baseTransformStaging, 16L, stream);

      int gridSize = (NUM_SCAN_POINTS + BLOCK_SIZE_1D - 1) / BLOCK_SIZE_1D;
      dim3 block = new dim3(BLOCK_SIZE_1D, 1, 1);
      dim3 grid  = new dim3(gridSize, 1, 1);

      kernelExtractScan
            .withPointer(gpuGrid)
            .withInt(nx).withInt(ny).withInt(nz)
            .withFloat(resolution)
            .withFloat(anchorX).withFloat(anchorY).withFloat(anchorZ)
            .withPointer(gpuScanX)
            .withPointer(gpuScanY)
            .withInt(NUM_SCAN_POINTS)
            .withPointer(gpuBaseTransform)
            .withInt(2)              // searchRadiusVoxels: 2 voxels = 10 cm at 5 cm res
            .withFloat(-1.0f)        // defaultRelativeHeight: unknown → -1 m
            .withPointer(gpuHeightsOut)
            .run(stream, grid, block, 0);

      block.close();
      grid.close();

      CUDATools.memcpyAsync(heightsReadbackPtr, gpuHeightsOut, NUM_SCAN_POINTS, stream);
      CUDATools.checkCUDAError(cudaStreamSynchronize(stream));
      heightsReadbackPtr.get(heightsToPack, startIndex, NUM_SCAN_POINTS);
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

   private static float[] buildScanPattern(boolean xAxis)
   {
      float[] pattern = new float[NUM_SCAN_POINTS];
      int idx = 0;
      for (int xi = 0; xi < SCAN_NX; xi++)
         for (int yi = 0; yi < SCAN_NY; yi++)
            pattern[idx++] = xAxis ? SCAN_X_MIN + xi * SCAN_STEP
                                   : SCAN_Y_MIN + yi * SCAN_STEP;
      return pattern;
   }

   @Override
   public void close()
   {
      CUDATools.checkCUDAError(cudaFreeAsync(gpuGrid, stream));
      CUDATools.checkCUDAError(cudaFreeAsync(gpuScanX, stream));
      CUDATools.checkCUDAError(cudaFreeAsync(gpuScanY, stream));
      CUDATools.checkCUDAError(cudaFreeAsync(gpuHeightsOut, stream));
      CUDATools.checkCUDAError(cudaFreeAsync(gpuBaseTransform, stream));
      CUDATools.freeHost(baseTransformStaging);
      CUDATools.freeHost(heightsReadbackPtr);
      CUDATools.checkCUDAError(cudaStreamSynchronize(stream));
      kernelUpdateHits.close();
      kernelUpdateMisses.close();
      kernelClear.close();
      kernelDecay.close();
      kernelDilate.close();
      kernelExtractScan.close();
      program.close();
      CUDAStreamManager.releaseStream(stream);
   }
}
