package us.ihmc.perception.gpuHeightMap;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.cuda.global.cudart;
import org.bytedeco.javacpp.DoublePointer;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.IntPointer;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.log.LogTools;
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

   private final PlanarityChecker planarityChecker;

   // CUDA stuff
   private final CUDAProgram program;
   private final CUDAKernel computeKernel;
   private final CUDAKernel blockResultKernel;
   private final CUDAKernel globalResultKernel;

   private CUstream_st cudaStream;
   private dim3 blockSize;
   private dim3 gridSize;

   private final float searchRadius;
   private final int stepsXY;
   private final int stepsYaw;
   private int searchSpaceDim;
   private final float footLength;
   private final float footWidth;

   // Pointers to CUDA memory
   private FloatPointer cpuCosts;
   private FloatPointer gpuCosts;
   private FloatPointer cpuBestCosts;
   private FloatPointer gpuBestCosts;
   private IntPointer cpuBestIndices;
   private IntPointer gpuBestIndices;
   private IntPointer cpuGlobalBestIndex;
   private IntPointer gpuGlobalBestIndex;
   private FloatPointer cpuGlobalBestCost;
   private FloatPointer gpuGlobalBestCost;
   private final float[] bestSolution = new float[3];

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
         blockResultKernel = program.loadKernel("findBlockMinima");
         globalResultKernel = program.loadKernel("findGlobalMinimum");
      }
      catch (Exception e)
      {
         throw new RuntimeException(e);
      }

      searchRadius = footLength;
      stepsXY = (int) (2 * searchRadius / SEARCH_SPACE_RESOLUTION_XY) + 1;
      stepsYaw = (int) (Math.PI / 4 / SEARCH_SPACE_RESOLUTION_YAW);

      planarityChecker = new PlanarityChecker(footLength, footWidth);
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
      if (planarityChecker.isOnPlane(heightMapData, initialPose))
      {
         LogTools.info("Initial solution is valid, no optimization needed.");
         return new FramePose3D(initialPose);
      }
      // Get a stream
      cudaStream = CUDAStreamManager.getStream();
      searchSpaceDim = stepsXY * stepsXY * stepsYaw;

      cpuCosts = new FloatPointer(searchSpaceDim);
      gpuCosts = new FloatPointer();
      CUDATools.mallocAsync(gpuCosts, searchSpaceDim, cudaStream);

      FloatPointer cpuSolutions = new FloatPointer(3L * searchSpaceDim);
      FloatPointer gpuSolutions = new FloatPointer();
      CUDATools.mallocAsync(gpuSolutions, 3L * searchSpaceDim, cudaStream);

      blockSize = new dim3(512, 1, 1); // Older gpus have a limit of 512, newer ones of 1024
      gridSize = new dim3(searchSpaceDim / blockSize.x(), 1, 1);

      FloatPointer cpuInitialPose = new FloatPointer((float) initialPose.getX(), (float) initialPose.getY(), (float) initialPose.getZ(), (float) initialPose.getYaw());
      FloatPointer gpuInitialPose = new FloatPointer();
      CUDATools.mallocAsync(gpuInitialPose, 4, cudaStream);

      DoublePointer cpuHeights = new DoublePointer(heightMapData.getHeights());
      DoublePointer gpuHeights = new DoublePointer();
      CUDATools.mallocAsync(gpuHeights, heightMapData.getHeights().length, cudaStream);

      FloatPointer cpuGridCenter = new FloatPointer((float) heightMapData.getGridCenter().getX(), (float) heightMapData.getGridCenter().getY());
      FloatPointer gpuGridCenter = new FloatPointer();
      CUDATools.mallocAsync(gpuGridCenter, 2, cudaStream);

      // Copying data from CPU to GPU
      CUDATools.memcpyAsync(gpuCosts, cpuCosts, searchSpaceDim, cudaStream);
      CUDATools.memcpyAsync(gpuSolutions, cpuSolutions, 3L * searchSpaceDim, cudaStream);
      CUDATools.memcpyAsync(gpuInitialPose, cpuInitialPose, 4, cudaStream);
      CUDATools.memcpyAsync(gpuHeights, cpuHeights, heightMapData.getHeights().length, cudaStream);
      CUDATools.memcpyAsync(gpuGridCenter, cpuGridCenter, 2, cudaStream);

      LogTools.info("Running kernel compute");
      // Runs the kernel with the desired grid and block sizes
      computeKernel.withPointer(gpuHeights)
                   .withPointer(gpuGridCenter)
                   .withInt(heightMapData.getCenterIndex())
                   .withFloat((float) heightMapData.getGridResolutionXY())
                   .withPointer(gpuInitialPose)
                   .withFloat(footLength)
                   .withFloat(footWidth)
                   .withFloat(searchRadius)
                   .withInt(stepsXY)
                   .withInt(stepsYaw)
                   .withPointer(gpuCosts)
                   .withPointer(gpuSolutions)
                   .run(cudaStream, gridSize, blockSize, 0);

      // Synchronize the stream
      // This call waits until all asynchronous functions being executed on this stream finish.
      // We have to call this to ensure that the kernel finished, and the result is ready to be downloaded onto the CPU
      cudart.cudaStreamSynchronize(cudaStream);

      CUDATools.memcpyAsync(cpuSolutions, gpuSolutions, 3L * searchSpaceDim, cudaStream);

      blockSize.close();
      gridSize.close();
      cudaFreeAsync(gpuInitialPose, cudaStream);
      cudaFreeAsync(gpuHeights, cudaStream);
      cudaFreeAsync(gpuGridCenter, cudaStream);

      // Now run another kernel to retrieve the best solution among those in cpuSolutions
      blockSize = new dim3(Math.min(searchSpaceDim, 512), 1, 1);
      gridSize = new dim3(searchSpaceDim / blockSize.x(), 1, 1);

      int resultSize = searchSpaceDim / blockSize.x();

      cpuBestCosts = new FloatPointer(resultSize);
      gpuBestCosts = new FloatPointer();
      CUDATools.mallocAsync(gpuBestCosts, resultSize, cudaStream);

      cpuBestIndices = new IntPointer(resultSize);
      gpuBestIndices = new IntPointer();
      CUDATools.mallocAsync(gpuBestIndices, resultSize, cudaStream);

      CUDATools.memcpyAsync(gpuBestCosts, cpuBestCosts, resultSize, cudaStream);
      CUDATools.memcpyAsync(gpuBestIndices, cpuBestIndices, resultSize, cudaStream);
      LogTools.info("Running block kernel result");
      // Run the kernel
      blockResultKernel.withPointer(gpuCosts)
                       .withInt(searchSpaceDim)
                       .withPointer(gpuBestCosts)
                       .withPointer(gpuBestIndices)
                       .run(cudaStream, gridSize, blockSize, 0);

      // Synchronize the stream
      // This call waits until all asynchronous functions being executed on this stream finish.
      // We have to call this to ensure that the kernel finished, and the result is ready to be downloaded onto the CPU
      cudart.cudaStreamSynchronize(cudaStream);

      searchSpaceDim = resultSize;
      cpuGlobalBestIndex = new IntPointer(1);
      gpuGlobalBestIndex = new IntPointer();
      CUDATools.mallocAsync(gpuGlobalBestIndex, 1, cudaStream);

      cpuGlobalBestCost = new FloatPointer(1);
      gpuGlobalBestCost = new FloatPointer();
      CUDATools.mallocAsync(gpuGlobalBestCost, 1, cudaStream);

      LogTools.info("Running global kernel result");
      // Run the kernel
      globalResultKernel.withPointer(gpuBestCosts)
                        .withPointer(gpuBestIndices)
                        .withInt(searchSpaceDim)
                        .withPointer(gpuGlobalBestCost)
                        .withPointer(gpuGlobalBestIndex)
                        .run(cudaStream, gridSize, blockSize, 0);

      // Synchronize the stream
      // This call waits until all asynchronous functions being executed on this stream finish.
      // We have to call this to ensure that the kernel finished, and the result is ready to be downloaded onto the CPU
      cudart.cudaStreamSynchronize(cudaStream);

      CUDATools.memcpyAsync(cpuGlobalBestCost, gpuGlobalBestCost, 1, cudaStream);
      CUDATools.memcpyAsync(cpuGlobalBestIndex, gpuGlobalBestIndex, 1, cudaStream);

      LogTools.info("Grid search completed");
      int startIndex = cpuGlobalBestIndex.get(0);
      bestSolution[0] = cpuSolutions.get(startIndex);
      bestSolution[1] = cpuSolutions.get(startIndex + 1);
      bestSolution[2] = cpuSolutions.get(startIndex + 2);

      FramePose3D optimalFootstepPose = new FramePose3D(ReferenceFrame.getWorldFrame(),
                                                        new Point3D(bestSolution[0], bestSolution[1], heightMapData.getHeightAt(bestSolution[0], bestSolution[1])),
                                                        new YawPitchRoll(bestSolution[2], initialPose.getPitch(), initialPose.getRoll()));

      // Release stuff
      blockSize.close();
      gridSize.close();
      cudaFreeAsync(gpuCosts, cudaStream);
      cudaFreeAsync(gpuGlobalBestCost, cudaStream);
      cudaFreeAsync(gpuGlobalBestIndex, cudaStream);
      cudaFreeAsync(gpuSolutions, cudaStream);
      cudaFreeAsync(gpuBestCosts, cudaStream);
      cudaFreeAsync(gpuBestIndices, cudaStream);
      CUDAStreamManager.releaseStream(cudaStream);

      return optimalFootstepPose;
   }

   @Override
   public void close()
   {
      computeKernel.close();
      blockResultKernel.close();
      globalResultKernel.close();
      program.close();
   }

   public void testResultKernel()
   {
      cudaStream = CUDAStreamManager.getStream();

      blockSize = new dim3(Math.min(searchSpaceDim, 512), 1, 1);
      gridSize = new dim3(searchSpaceDim / blockSize.x(), 1, 1);

      int resultSize = searchSpaceDim / blockSize.x();

      cpuBestCosts = new FloatPointer(resultSize);
      gpuBestCosts = new FloatPointer();
      CUDATools.mallocAsync(gpuBestCosts, resultSize, cudaStream);

      cpuBestIndices = new IntPointer(resultSize);
      gpuBestIndices = new IntPointer();
      CUDATools.mallocAsync(gpuBestIndices, resultSize, cudaStream);

      CUDATools.memcpyAsync(gpuBestCosts, cpuBestCosts, resultSize, cudaStream);
      CUDATools.memcpyAsync(gpuBestIndices, cpuBestIndices, resultSize, cudaStream);
      LogTools.warn("Running block kernel result");
      // Run the kernel
      blockResultKernel.withPointer(gpuCosts)
                       .withInt(searchSpaceDim)
                       .withPointer(gpuBestCosts)
                       .withPointer(gpuBestIndices)
                       .run(cudaStream, gridSize, blockSize, 0);

      // Synchronize the stream
      // This call waits until all asynchronous functions being executed on this stream finish.
      // We have to call this to ensure that the kernel finished, and the result is ready to be downloaded onto the CPU
      cudart.cudaStreamSynchronize(cudaStream);

      CUDATools.memcpyAsync(cpuBestCosts, gpuBestCosts, resultSize, cudaStream);
      CUDATools.memcpyAsync(cpuBestIndices, gpuBestIndices, resultSize, cudaStream);

      for (int i=0; i<resultSize; i++)
         LogTools.info("Best Block Costs: {}, Indices: {}", cpuBestCosts.get(i), cpuBestIndices.get(i));

      searchSpaceDim = resultSize;
      cpuGlobalBestIndex = new IntPointer(1);
      gpuGlobalBestIndex = new IntPointer();
      CUDATools.mallocAsync(gpuGlobalBestIndex, 1, cudaStream);

      cpuGlobalBestCost = new FloatPointer(1);
      gpuGlobalBestCost = new FloatPointer();
      CUDATools.mallocAsync(gpuGlobalBestCost, 1, cudaStream);

      LogTools.warn("Running global kernel result");
      // Run the kernel
      globalResultKernel.withPointer(gpuBestCosts)
                        .withPointer(gpuBestIndices)
                        .withInt(searchSpaceDim)
                        .withPointer(gpuGlobalBestCost)
                        .withPointer(gpuGlobalBestIndex)
                        .run(cudaStream, gridSize, blockSize, 0);

      // Synchronize the stream
      // This call waits until all asynchronous functions being executed on this stream finish.
      // We have to call this to ensure that the kernel finished, and the result is ready to be downloaded onto the CPU
      cudart.cudaStreamSynchronize(cudaStream);

      CUDATools.memcpyAsync(cpuGlobalBestCost, gpuGlobalBestCost, 1, cudaStream);
      CUDATools.memcpyAsync(cpuGlobalBestIndex, gpuGlobalBestIndex, 1, cudaStream);

      LogTools.info("Best Cost: {}, Idx: {}", cpuGlobalBestCost.get(0), cpuGlobalBestIndex.get(0));

      // Release stuff
      blockSize.close();
      gridSize.close();
      cudaFreeAsync(gpuCosts, cudaStream);
      cudaFreeAsync(cpuGlobalBestCost, cudaStream);
      cudaFreeAsync(gpuGlobalBestIndex, cudaStream);
      cudaFreeAsync(gpuBestCosts, cudaStream);
      cudaFreeAsync(gpuBestIndices, cudaStream);
      CUDAStreamManager.releaseStream(cudaStream);
   }

   public void setGpuCosts(float[] costs)
   {
      cpuCosts = new FloatPointer(costs.length);
      gpuCosts = new FloatPointer();
      CUDATools.mallocAsync(gpuCosts, costs.length, cudaStream);
      cpuCosts.put(costs);
      CUDATools.memcpyAsync(gpuCosts, cpuCosts, costs.length, cudaStream);
      searchSpaceDim = costs.length;
   }

   public float getBestCost()
   {
      return cpuGlobalBestCost.get(0);
   }

   public float getBestIndex()
   {
      return cpuGlobalBestIndex.get(0);
   }
}
