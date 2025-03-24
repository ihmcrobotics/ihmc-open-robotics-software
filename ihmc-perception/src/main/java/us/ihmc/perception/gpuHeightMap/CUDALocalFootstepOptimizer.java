package us.ihmc.perception.gpuHeightMap;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.cuda.global.cudart;
import org.bytedeco.javacpp.DoublePointer;
import org.bytedeco.javacpp.FloatPointer;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.perception.cuda.CUDAKernel;
import us.ihmc.perception.cuda.CUDAProgram;
import us.ihmc.perception.cuda.CUDAStreamManager;
import us.ihmc.perception.cuda.CUDATools;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;

import java.net.URL;

import static org.bytedeco.cuda.global.cudart.cudaFreeAsync;

public class CUDALocalFootstepOptimizer implements AutoCloseable
{
   private static final double SEARCH_SPACE_RESOLUTION_XY = 0.02;
   private static final double SEARCH_SPACE_RESOLUTION_YAW = Math.toRadians(5);

   // CUDA stuff
   private final CUDAProgram program;
   private final CUDAKernel computeKernel;
   private final CUDAKernel resultKernel;
   private CUstream_st cudaStream;
   private dim3 blockSize;
   private dim3 gridSize;

   private final float searchRadius;
   private final int stepsXY;
   private final int stepsYaw;
   private final int searchSpaceDim;
   private final float footLength;
   private final float footWidth;

   // Pointers to CUDA memory
   private FloatPointer cpuCosts;
   private FloatPointer cpuSolutions;
   private FloatPointer gpuCosts;
   private FloatPointer gpuSolutions;
   private FloatPointer cpuBestCost;
   private FloatPointer cpuBestSolution;
   private FloatPointer gpuBestCost;
   private FloatPointer gpuBestSolution;

   public CUDALocalFootstepOptimizer(float footLength, float footWidth)
   {
      this.footLength = footLength;
      this.footWidth = footWidth;

      // Get URLs to the CUDA files
      URL optimizerExtractionURL = getClass().getResource("LocalFootstepOptimization.cu");

      // Compile the program, and get the kernels
      try
      {
         program = new CUDAProgram(optimizerExtractionURL);
         computeKernel = program.loadKernel("optimizeFootstep");
         resultKernel = program.loadKernel("findBestSolution");
      }
      catch (Exception e)
      {
         throw new RuntimeException(e);
      }

      searchRadius = footLength;
      stepsXY = (int) (2 * searchRadius / SEARCH_SPACE_RESOLUTION_XY) + 1;
      stepsYaw = (int) (Math.PI / 4 / SEARCH_SPACE_RESOLUTION_YAW);
      searchSpaceDim = stepsXY * stepsXY * stepsYaw;
   }

   private void initialize()
   {
      // Get a stream
      cudaStream = CUDAStreamManager.getStream();

      cpuCosts = new FloatPointer(searchSpaceDim);
      gpuCosts = new FloatPointer();
      CUDATools.mallocAsync(gpuCosts, searchSpaceDim, cudaStream);

      cpuSolutions = new FloatPointer(searchSpaceDim);
      gpuSolutions = new FloatPointer();
      CUDATools.mallocAsync(gpuSolutions, searchSpaceDim, cudaStream);

      cpuBestCost = new FloatPointer(1);
      gpuBestCost = new FloatPointer();
      CUDATools.mallocAsync(gpuBestCost, 1, cudaStream);

      cpuBestSolution = new FloatPointer(3);
      gpuBestSolution = new FloatPointer();
      CUDATools.mallocAsync(gpuBestSolution, 3, cudaStream);
   }

   /**
    * Computes the optimized footstep pose
    *
    * @param heightMapData The height map data of the environment.
    * @param initialPose   The initial pose of the footstep.
    * @return The optimized FramePose3D.
    */
   public FramePose3D compute(HeightMapData heightMapData, FramePose3DReadOnly initialPose)
   {
      initialize();

      blockSize = new dim3(512, 1, 1); // Older gpus have a limit of 512, newer ones of 1024
      gridSize = new dim3(searchSpaceDim / blockSize.x(), 1, 1);

      FloatPointer cpuInitialPose = new FloatPointer((float) initialPose.getX(), (float) initialPose.getY(), (float) initialPose.getYaw());
      FloatPointer gpuInitialPose = new FloatPointer();
      CUDATools.mallocAsync(gpuInitialPose, 3, cudaStream);

      DoublePointer cpuHeights = new DoublePointer(heightMapData.getHeights());
      DoublePointer gpuHeights = new DoublePointer();
      CUDATools.mallocAsync(gpuHeights, heightMapData.getHeights().length, cudaStream);

      DoublePointer cpuGridCenter = new DoublePointer(heightMapData.getGridCenter().getX(), (float) heightMapData.getGridCenter().getY());
      DoublePointer gpuGridCenter = new DoublePointer();
      CUDATools.mallocAsync(gpuGridCenter, 2, cudaStream);

      // Copying data from CPU to GPU
      CUDATools.memcpyAsync(gpuCosts, cpuCosts, searchSpaceDim, cudaStream);
      CUDATools.memcpyAsync(gpuSolutions, cpuSolutions, searchSpaceDim, cudaStream);
      CUDATools.memcpyAsync(gpuInitialPose, cpuInitialPose, 3, cudaStream);
      CUDATools.memcpyAsync(gpuHeights, cpuHeights, heightMapData.getHeights().length, cudaStream);
      CUDATools.memcpyAsync(gpuGridCenter, cpuGridCenter, 2, cudaStream);

      // Runs the kernel with the desired grid and block sizes
      computeKernel.withPointer(gpuHeights)
                   .withPointer(gpuGridCenter)
                   .withInt(heightMapData.getCenterIndex())
                   .withDouble(heightMapData.getGridResolutionXY())
                   .withPointer(gpuInitialPose)
                   .withFloat(footLength)
                   .withFloat(footWidth)
                   .withFloat(searchRadius)
                   .withInt(stepsXY)
                   .withInt(stepsYaw)
                   .withPointer(gpuCosts)
                   .withPointer(gpuSolutions)
                   .run(cudaStream, gridSize, blockSize, 0);

      blockSize.close();
      gridSize.close();
      cudaFreeAsync(gpuInitialPose, cudaStream);
      cudaFreeAsync(gpuHeights, cudaStream);
      cudaFreeAsync(gpuGridCenter, cudaStream);

      // Now run another kernel to retrieve the best solution among those in cpuSolutions
      blockSize = new dim3(256, 1, 1);
      gridSize = new dim3((searchSpaceDim + blockSize.x() -1) / blockSize.x(), 1, 1);
      int sharedMemSize = 2 * 256 * Float.BYTES; // Account for both sharedCosts

      CUDATools.memcpyAsync(gpuBestCost, cpuBestCost, 1, cudaStream);
      CUDATools.memcpyAsync(gpuBestSolution, cpuBestSolution, 3, cudaStream);
      // Run the kernel
      resultKernel.withPointer(gpuCosts)
                  .withPointer(gpuSolutions)
                  .withInt(searchSpaceDim)
                  .withPointer(gpuBestCost)
                  .withPointer(gpuBestSolution)
                  .run(cudaStream, gridSize, blockSize, sharedMemSize);

      // Synchronize the stream
      // This call waits until all asynchronous functions being executed on this stream finish.
      // We have to call this to ensure that the kernel finished, and the result is ready to be downloaded onto the CPU
      cudart.cudaStreamSynchronize(cudaStream);

      CUDATools.memcpyAsync(cpuBestSolution, gpuBestSolution, 3, cudaStream);

      FramePose3D optimalFootstepPose = new FramePose3D(ReferenceFrame.getWorldFrame(),
                                                        new Point3D(cpuBestSolution.get(0), cpuBestSolution.get(1), initialPose.getZ()),
                                                        new YawPitchRoll(cpuBestSolution.get(2), initialPose.getPitch(), initialPose.getRoll()));

      // Release stuff
      blockSize.close();
      gridSize.close();
      cudaFreeAsync(gpuCosts, cudaStream);
      cudaFreeAsync(gpuSolutions, cudaStream);
      cudaFreeAsync(gpuBestCost, cudaStream);
      cudaFreeAsync(gpuBestSolution, cudaStream);
      CUDAStreamManager.releaseStream(cudaStream);

      return optimalFootstepPose;
   }

   @Override
   public void close()
   {
      computeKernel.close();
      resultKernel.close();
      program.close();
      CUDAStreamManager.releaseStream(cudaStream);
   }
}
