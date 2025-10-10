package us.ihmc.perception.cuda;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.cuda.global.cudart;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.javacpp.PointerScope;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.log.LogTools;
import us.ihmc.perception.gpuMapping.PlanarityChecker;
import us.ihmc.perception.gpuMapping.HeightMapData;

import java.net.URL;

import static org.bytedeco.cuda.global.cudart.cudaFreeAsync;

public class CUDAFootstepOptimizer implements AutoCloseable
{
   private static final double SEARCH_SPACE_RESOLUTION_XY = 0.02;
   private static final double SEARCH_SPACE_RESOLUTION_YAW = Math.toRadians(5);

   private final PlanarityChecker planarityChecker;

   // CUDA stuff
   private final CUDAProgram program;
   private final CUDAKernel computeKernel;
   private final CUDAKernel blockResultKernel;
   private final CUDAKernel globalResultKernel;

   private final CUstream_st cudaStream;
   private final dim3 blockSize;
   private final dim3 gridSize;

   private final float searchRadius;
   private final float searchYawLimit;
   private final int stepsXY;
   private final int stepsYaw;
   private int searchSpaceDim;
   private final float footLength;
   private final float footWidth;

   // Pointers to CUDA memory
   private FloatPointer gpuCosts;
   private IntPointer cpuGlobalBestIndex;
   private FloatPointer cpuGlobalBestCost;
   private final float[] bestSolution = new float[3];

   public CUDAFootstepOptimizer(float footLength, float footWidth)
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

      searchRadius = 0.5f * footLength;
      searchYawLimit = (float) Math.PI / 4;
      stepsXY = (int) (2 * searchRadius / SEARCH_SPACE_RESOLUTION_XY) + 1;
      stepsYaw = (int) (2 * searchYawLimit / SEARCH_SPACE_RESOLUTION_YAW);
      searchSpaceDim = stepsXY * stepsXY * stepsYaw;
      // Get a stream
      cudaStream = CUDAStreamManager.getStream();

      planarityChecker = new PlanarityChecker(footLength, footWidth);

      blockSize = new dim3();
      gridSize = new dim3();
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
      HeightMapData currentHeightMapData = new HeightMapData(heightMapData);

      try (PointerScope ignored = new PointerScope())
      {
         FloatPointer gpuCosts = new FloatPointer();
         CUDATools.mallocAsync(gpuCosts, searchSpaceDim, cudaStream);

         FloatPointer cpuSolutions = new FloatPointer(3L * searchSpaceDim);
         FloatPointer gpuSolutions = new FloatPointer();
         CUDATools.mallocAsync(gpuSolutions, 3L * searchSpaceDim, cudaStream);

         FloatPointer cpuInitialPose = new FloatPointer((float) initialPose.getX(),
                                                        (float) initialPose.getY(),
                                                        (float) initialPose.getZ(),
                                                        (float) initialPose.getYaw());
         FloatPointer gpuInitialPose = new FloatPointer();
         CUDATools.mallocAsync(gpuInitialPose, 4, cudaStream);

         FloatPointer cpuHeights = new FloatPointer(currentHeightMapData.getHeights());
         FloatPointer gpuHeights = new FloatPointer();
         CUDATools.mallocAsync(gpuHeights, currentHeightMapData.getHeights().length, cudaStream);

         FloatPointer cpuGridCenter = new FloatPointer((float) currentHeightMapData.getGridCenter().getX(),
                                                       (float) currentHeightMapData.getGridCenter().getY());
         FloatPointer gpuGridCenter = new FloatPointer();
         CUDATools.mallocAsync(gpuGridCenter, 2, cudaStream);

         // Copying data from CPU to GPU
         CUDATools.memcpyAsync(gpuInitialPose, cpuInitialPose, 4, cudaStream);
         CUDATools.memcpyAsync(gpuHeights, cpuHeights, currentHeightMapData.getHeights().length, cudaStream);
         CUDATools.memcpyAsync(gpuGridCenter, cpuGridCenter, 2, cudaStream);

         LogTools.warn("Running compute kernel");
         // Runs the kernel with the desired grid and block sizes
         blockSize.x(512);
         gridSize.x((searchSpaceDim + blockSize.x() - 1) / blockSize.x());
         computeKernel.withPointer(gpuHeights)
                      .withPointer(gpuGridCenter)
                      .withInt(currentHeightMapData.getCenterIndex())
                      .withFloat((float) currentHeightMapData.getCellSize())
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
         int resultSize = (searchSpaceDim + blockSize.x() - 1) / blockSize.x();
         FloatPointer gpuBestCosts = new FloatPointer();
         IntPointer gpuBestIndices = new IntPointer();
         CUDATools.mallocAsync(gpuBestCosts, resultSize, cudaStream);
         CUDATools.mallocAsync(gpuBestIndices, resultSize, cudaStream);

         LogTools.warn("Running block result kernel");
         // Run the kernel
         blockSize.x(512);
         gridSize.x(resultSize);
         blockResultKernel.withPointer(gpuCosts)
                          .withInt(searchSpaceDim)
                          .withPointer(gpuBestCosts)
                          .withPointer(gpuBestIndices)
                          .run(cudaStream, gridSize, blockSize, 0);

         FloatPointer gpuGlobalBestCost = new FloatPointer();
         IntPointer gpuGlobalBestIndex = new IntPointer();
         CUDATools.mallocAsync(gpuGlobalBestIndex, 1, cudaStream);
         CUDATools.mallocAsync(gpuGlobalBestCost, 1, cudaStream);

         LogTools.warn("Running global result kernel");
         // Run the kernel
         blockSize.x(resultSize);
         gridSize.x(1);
         globalResultKernel.withPointer(gpuBestCosts)
                           .withPointer(gpuBestIndices)
                           .withInt(resultSize)
                           .withPointer(gpuGlobalBestCost)
                           .withPointer(gpuGlobalBestIndex)
                           .run(cudaStream, gridSize, blockSize, 0);
         LogTools.warn("Done global result kernel");
         IntPointer cpuGlobalBestIndex = new IntPointer(1);
         FloatPointer cpuGlobalBestCost = new FloatPointer(1);
         CUDATools.memcpyAsync(cpuGlobalBestIndex, gpuGlobalBestIndex, 1, cudaStream);
         CUDATools.memcpyAsync(cpuGlobalBestCost, gpuGlobalBestCost, 1, cudaStream);

         cudart.cudaStreamSynchronize(cudaStream);

         int startIndex = cpuGlobalBestIndex.get(0);
         LogTools.info("Grid search finished. Best cost: {}, idx: {}", cpuGlobalBestCost.get(0), startIndex);
         bestSolution[0] = cpuSolutions.get(startIndex * 3L);
         bestSolution[1] = cpuSolutions.get(startIndex * 3L + 1);
         bestSolution[2] = cpuSolutions.get(startIndex * 3L + 2);

         FramePose3D optimalFootstepPose = new FramePose3D(initialPose);
         optimalFootstepPose.setX(bestSolution[0]);
         optimalFootstepPose.setY(bestSolution[1]);
         optimalFootstepPose.getRotation().set(new YawPitchRoll(bestSolution[2], initialPose.getPitch(), initialPose.getRoll()));
         optimalFootstepPose.setZ(currentHeightMapData.getHeight(bestSolution[0], bestSolution[1]));

         // Release stuff
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
   }

   @Override
   public void close()
   {
      blockSize.close();
      gridSize.close();
      computeKernel.close();
      blockResultKernel.close();
      globalResultKernel.close();
      program.close();
      CUDAStreamManager.releaseStream(cudaStream);
   }

   void testResultKernel()
   {
      cpuGlobalBestIndex = new IntPointer(1);
      cpuGlobalBestCost = new FloatPointer(1);
      try (PointerScope ignored = new PointerScope())
      {
         blockSize.x(512);

         int resultSize = (searchSpaceDim + blockSize.x() -1) / blockSize.x();
         FloatPointer cpuBestCosts = new FloatPointer(resultSize);
         FloatPointer gpuBestCosts = new FloatPointer();
         CUDATools.mallocAsync(gpuBestCosts, resultSize, cudaStream);

         IntPointer cpuBestIndices = new IntPointer(resultSize);
         IntPointer gpuBestIndices = new IntPointer();
         CUDATools.mallocAsync(gpuBestIndices, resultSize, cudaStream);

         LogTools.info("Running block kernel result");
         // Run the kernel
         blockSize.x(512);
         gridSize.x(resultSize);
         blockResultKernel.withPointer(gpuCosts)
                          .withInt(searchSpaceDim)
                          .withPointer(gpuBestCosts)
                          .withPointer(gpuBestIndices)
                          .run(cudaStream, gridSize, blockSize, 0);

         CUDATools.memcpyAsync(cpuBestCosts, gpuBestCosts, resultSize, cudaStream);
         CUDATools.memcpyAsync(cpuBestIndices, gpuBestIndices, resultSize, cudaStream);

         for (int i = 0; i < resultSize; i++)
            LogTools.info("Best Block Costs: {}, Indices: {}", cpuBestCosts.get(i), cpuBestIndices.get(i));

         searchSpaceDim = resultSize;
         FloatPointer gpuGlobalBestCost = new FloatPointer();
         IntPointer gpuGlobalBestIndex = new IntPointer();
         CUDATools.mallocAsync(gpuGlobalBestIndex, 1, cudaStream);
         CUDATools.mallocAsync(gpuGlobalBestCost, 1, cudaStream);

         LogTools.info("Running global kernel result");
         // Run the kernel
         blockSize.x(resultSize);
         gridSize.x(1);
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
   }

   void setGpuCosts(float[] costs)
   {
      FloatPointer cpuCosts = new FloatPointer(costs.length);
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
