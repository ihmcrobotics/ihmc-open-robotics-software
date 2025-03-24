package us.ihmc.perception.gpuHeightMap;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.cuda.global.cudart;
import org.bytedeco.javacpp.FloatPointer;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.perception.cuda.CUDAKernel;
import us.ihmc.perception.cuda.CUDAProgram;
import us.ihmc.perception.cuda.CUDAStreamManager;
import us.ihmc.perception.cuda.CUDATools;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;

import java.net.URL;
import java.util.Arrays;
import java.util.stream.IntStream;

import static org.bytedeco.cuda.global.cudart.*;

public class CUDALocalFootstepOptimizer implements AutoCloseable
{
   private static final double SEARCH_SPACE_RESOLUTION_XY = 0.02;
   private static final double SEARCH_SPACE_RESOLUTION_YAW = Math.toRadians(5);

   // CUDA stuff
   private final CUDAProgram program;
   private final CUDAKernel computeKernel;
   private final CUDAKernel resultKernel;
   private final CUstream_st stream;
   private final dim3 blockSize;
   private final dim3 gridSize;

   private final float searchRadius;
   private final int stepsXY;
   private final int stepsYaw;

   // Pointers to CUDA memory
   private final FloatPointer costs = new FloatPointer();
   private final FloatPointer solutions = new FloatPointer();
   private final FloatPointer bestCost = new FloatPointer();
   private final FloatPointer bestSolution = new FloatPointer();

   public CUDALocalFootstepOptimizer(float footLength, float footWidth)
   {
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

      // Get a stream
      stream = CUDAStreamManager.getStream();

      searchRadius = footLength;
      stepsXY = (int) (2 * searchRadius / SEARCH_SPACE_RESOLUTION_XY) + 1;
      stepsYaw = (int) (Math.PI/4 / SEARCH_SPACE_RESOLUTION_YAW);

      int totalNumberOfThreads = stepsXY * stepsXY * stepsYaw;
      blockSize = new dim3(512, 1, 1); // Older gpus have a limit of 512, newer ones of 1024
      gridSize = new dim3(totalNumberOfThreads / blockSize.x(), 1, 1);
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
      // Runs the kernel with the desired grid and block sizes
      computeKernel.withPointer(heightmap)
                   .withPointer(initialPose)
                   .withFloat(searchRadius)
                   .withInt(stepsXY)
                   .withInt(stepsYaw)
                   .withPointer(costs)
                   .withPointer(solutions)
                   .run(stream, gridSize, blockSize, 0);

      // Synchronize the stream
      // This call waits until all asynchronous functions being executed on this stream finish.
      // We have to call this to ensure that the kernel finished, and the result is ready to be downloaded onto the CPU
      cudart.cudaStreamSynchronize(stream);

      // This is where we are pulling the result from the gpu. The download packs the variable being passed in
      gpuResult.download(cpuResult);

      printResult(cpuResult);

      // Remember to close these!
      gridDim.close();
      blockDim.close();

      // We need to worry about closing these variables
      dim3 gridDim = new dim3(); // The same thing as ( new dim3(1, 1, 1); )
      dim3 blockDim = new dim3(width, height, 1);

      // Runs the kernel with the desired grid and block sizes
      cudaKernel.run(stream, gridDim, blockDim, 0);




      dim3 blockSize(256, 1, 1);
      dim3 gridSize((totalThreads + blockSize.x - 1) / blockSize.x, 1, 1);
      int sharedMemSize = 2 * 256 * Float.BYTES; // Account for both sharedCosts

      // Run the kernel
      kernel.withPointer(depthImage.getCUDADataPointer())
            .withLong(depthImage.getGpuImageMat().step())
            .withInt(depthImage.getWidth())
            .withInt(depthImage.getHeight())
            .withFloat(depthImage.getFocalLengthX())
            .withFloat(depthImage.getFocalLengthY())
            .withFloat(depthImage.getPrincipalPointX())
            .withFloat(depthImage.getPrincipalPointY())
            .withFloat(depthImage.getDepthDiscretization())
            .withPointer(transformPointer)
            .withPointer(gpuPointCloudPointer)
            .withPointer(pointCloudSize)
            .run(stream, gridSize, blockSize, sharedMemSize);

      // Synchronize the stream to ensure we can read the point cloud size
      error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);
      int numberOfPoints = pointCloudSize.get();
      long numberOfFloats = 3L * numberOfPoints;

      // Copy the point cloud data from GPU to CPU, then to a Java float[]
      FloatPointer cpuPointCloudPointer = new FloatPointer(numberOfFloats);
      float[] pointsArray = new float[(int) numberOfFloats];
      CUDATools.memcpyAsync(cpuPointCloudPointer, gpuPointCloudPointer, numberOfFloats, stream);
      error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);
      cpuPointCloudPointer.get(pointsArray);

      // Create a list of points from the float[]
      Point3D32[] result = new Point3D32[numberOfPoints];
      IntStream.range(0, numberOfPoints).parallel().forEach(i ->
                                                            {
                                                               float x = pointsArray[3 * i];
                                                               float y = pointsArray[3 * i + 1];
                                                               float z = pointsArray[3 * i + 2];
                                                               result[i] = new Point3D32(x, y, z);
                                                            });

      // Release stuff
      blockSize.close();
      gridSize.close();
      cpuPointCloudPointer.close();
      depthImage.release();

      return Arrays.asList(result);
   }

   @Override
   public void close()
   {
      computeKernel.close();
      resultKernel.close();
      program.close();
      CUDAStreamManager.releaseStream(stream);
   }
}
