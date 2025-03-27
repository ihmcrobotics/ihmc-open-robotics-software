package us.ihmc.perception.gpuHeightMap;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.cuda.global.cudart;
import org.bytedeco.javacpp.DoublePointer;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.IntPointer;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
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
   private final FloatPointer cpuSolutions;
   private final FloatPointer gpuSolutions;
   private FloatPointer gpuBestCosts;
   private IntPointer gpuBestIndices;
   private final IntPointer cpuGlobalBestIndex;
   private final IntPointer gpuGlobalBestIndex;
   private final FloatPointer cpuGlobalBestCost;
   private final FloatPointer gpuGlobalBestCost;
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
      searchSpaceDim = stepsXY * stepsXY * stepsYaw;
      // Get a stream
      cudaStream = CUDAStreamManager.getStream();

      planarityChecker = new PlanarityChecker(footLength, footWidth);

      blockSize = new dim3(512, 1, 1);
      gridSize = new dim3((searchSpaceDim + blockSize.x() - 1) / blockSize.x(), 1, 1);
      // create pointers used in kernels
      cpuCosts = new FloatPointer(searchSpaceDim);
      gpuCosts = new FloatPointer();
      cpuSolutions = new FloatPointer(3L * searchSpaceDim);
      gpuSolutions = new FloatPointer();
      gpuBestCosts = new FloatPointer();
      gpuBestIndices = new IntPointer();
      cpuGlobalBestIndex = new IntPointer(1);
      gpuGlobalBestIndex = new IntPointer();
      cpuGlobalBestCost = new FloatPointer(1);
      gpuGlobalBestCost = new FloatPointer();
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

      CUDATools.mallocAsync(gpuCosts, searchSpaceDim, cudaStream);
      CUDATools.mallocAsync(gpuSolutions, 3L * searchSpaceDim, cudaStream);

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

      CUDATools.memcpyAsync(cpuSolutions, gpuSolutions, 3L * searchSpaceDim, cudaStream);

      // Now run another kernel to retrieve the best solution among those in cpuSolutions
      int resultSize = searchSpaceDim / blockSize.x();
      CUDATools.mallocAsync(gpuBestCosts, resultSize, cudaStream);
      CUDATools.mallocAsync(gpuBestIndices, resultSize, cudaStream);

      // Run the kernel
      blockResultKernel.withPointer(gpuCosts)
                       .withInt(searchSpaceDim)
                       .withPointer(gpuBestCosts)
                       .withPointer(gpuBestIndices)
                       .run(cudaStream, gridSize, blockSize, 0);

      searchSpaceDim = resultSize;
      CUDATools.mallocAsync(gpuGlobalBestIndex, 1, cudaStream);
      CUDATools.mallocAsync(gpuGlobalBestCost, 1, cudaStream);
      
      // Run the kernel
      globalResultKernel.withPointer(gpuBestCosts)
                        .withPointer(gpuBestIndices)
                        .withInt(searchSpaceDim)
                        .withPointer(gpuGlobalBestCost)
                        .withPointer(gpuGlobalBestIndex)
                        .run(cudaStream, new dim3(), blockSize, 0);

      CUDATools.memcpyAsync(cpuGlobalBestIndex, gpuGlobalBestIndex, 1, cudaStream);
      CUDATools.memcpyAsync(cpuGlobalBestCost, gpuGlobalBestCost, 1, cudaStream);
      cudart.cudaStreamSynchronize(cudaStream);

      int startIndex = cpuGlobalBestIndex.get(0);
      LogTools.info("Grid search finished. Best cost: {}, idx: {}", cpuGlobalBestCost.get(0), startIndex);
      bestSolution[0] = cpuSolutions.get(startIndex*3L);
      bestSolution[1] = cpuSolutions.get(startIndex*3L + 1);
      bestSolution[2] = cpuSolutions.get(startIndex*3L + 2);

      FramePose3D optimalFootstepPose = new FramePose3D(initialPose);
      optimalFootstepPose.setX(bestSolution[0]);
      optimalFootstepPose.setY(bestSolution[1]);
      optimalFootstepPose.getRotation().set(new YawPitchRoll(bestSolution[2], initialPose.getPitch(), initialPose.getRoll()));
      optimalFootstepPose.setZ(heightMapData.getHeightAt(bestSolution[0], bestSolution[1]));

      // Release stuff
      blockSize.close();
      gridSize.close();
      cudaFreeAsync(gpuInitialPose, cudaStream);
      cudaFreeAsync(gpuHeights, cudaStream);
      cudaFreeAsync(gpuGridCenter, cudaStream);
      cudaFreeAsync(gpuCosts, cudaStream);
      cudaFreeAsync(gpuGlobalBestCost, cudaStream);
      cudaFreeAsync(gpuGlobalBestIndex, cudaStream);
      cudaFreeAsync(gpuSolutions, cudaStream);
      cudaFreeAsync(gpuBestCosts, cudaStream);
      cudaFreeAsync(gpuBestIndices, cudaStream);

      return optimalFootstepPose;
   }

   @Override
   public void close()
   {
      computeKernel.close();
      blockResultKernel.close();
      globalResultKernel.close();
      program.close();
      CUDAStreamManager.releaseStream(cudaStream);
   }

   public void testResultKernel()
   {
      int resultSize = searchSpaceDim / blockSize.x();
      FloatPointer cpuBestCosts = new FloatPointer(resultSize);
      gpuBestCosts = new FloatPointer();
      CUDATools.mallocAsync(gpuBestCosts, resultSize, cudaStream);

      IntPointer cpuBestIndices = new IntPointer(resultSize);
      gpuBestIndices = new IntPointer();
      CUDATools.mallocAsync(gpuBestIndices, resultSize, cudaStream);

      LogTools.info("Running block kernel result");
      // Run the kernel
      blockResultKernel.withPointer(gpuCosts)
                       .withInt(searchSpaceDim)
                       .withPointer(gpuBestCosts)
                       .withPointer(gpuBestIndices)
                       .run(cudaStream, gridSize, blockSize, 0);

      CUDATools.memcpyAsync(cpuBestCosts, gpuBestCosts, resultSize, cudaStream);
      CUDATools.memcpyAsync(cpuBestIndices, gpuBestIndices, resultSize, cudaStream);

      for (int i=0; i<resultSize; i++)
         LogTools.info("Best Block Costs: {}, Indices: {}", cpuBestCosts.get(i), cpuBestIndices.get(i));

      searchSpaceDim = resultSize;
      CUDATools.mallocAsync(gpuGlobalBestIndex, 1, cudaStream);
      CUDATools.mallocAsync(gpuGlobalBestCost, 1, cudaStream);

      LogTools.info("Running global kernel result");
      // Run the kernel
      globalResultKernel.withPointer(gpuBestCosts)
                        .withPointer(gpuBestIndices)
                        .withInt(searchSpaceDim)
                        .withPointer(gpuGlobalBestCost)
                        .withPointer(gpuGlobalBestIndex)
                        .run(cudaStream, new dim3(), blockSize, 0);

      CUDATools.memcpyAsync(cpuGlobalBestCost, gpuGlobalBestCost, 1, cudaStream);
      CUDATools.memcpyAsync(cpuGlobalBestIndex, gpuGlobalBestIndex, 1, cudaStream);
      cudart.cudaStreamSynchronize(cudaStream);

      LogTools.info("Best Cost: {}, Idx: {}", cpuGlobalBestCost.get(0), cpuGlobalBestIndex.get(0));

      // Release stuff
      blockSize.close();
      gridSize.close();
      cudaFreeAsync(gpuCosts, cudaStream);
      cudaFreeAsync(cpuGlobalBestCost, cudaStream);
      cudaFreeAsync(gpuGlobalBestIndex, cudaStream);
      cudaFreeAsync(gpuBestCosts, cudaStream);
      cudaFreeAsync(gpuBestIndices, cudaStream);
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
