package us.ihmc.perception.gpuMapping;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Stream;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.perception.cuda.CUDAKernel;
import us.ihmc.perception.cuda.CUDAProgram;
import us.ihmc.perception.cuda.CUDAStreamManager;
import us.ihmc.perception.cuda.CUDATools;

import java.net.URL;

import static org.bytedeco.cuda.global.cudart.cudaFree;
import static org.bytedeco.cuda.global.cudart.cudaStreamSynchronize;

public class TerrainMapExtractor
{
   private static final boolean PRINT_TIMING_FOR_KERNELS = false;
   /**
    * The choice of 16 here is to utilize more SMs (Multi Processors) on the GPU.
    * This was chosen based on GPU profiling and significantly effects performance.
    */
   private static final int BLOCK_SIZE_XY = 16;

   private final TerrainMapData terrainMapData;
   private final HeightMapParameters heightMapParameters;
   private final TerrainMapParameters terrainMapParameters;

   private final Stream downloadStreamOne;
   private final Stream downloadStreamTwo;
   private final CUstream_st stream;
   private final CUDAProgram terrainMapProgram;
   private final CUDAKernel terrainMapKernel;

   private final dim3 blockSize;
   private int cellsPerAxisTerrain;

   private final FloatPointer terrainMapParametersHostPointer;
   private final FloatPointer terrainMapParametersDevicePointer;

   /**
    * The types of these mats depend on the data we are trying to store.
    * Check the {@link perception_msgs.msg.dds.TerrainMapMessage} to ensure these are the same
    */
   private final GpuMat normalXMat;
   private final GpuMat normalYMat;
   private final GpuMat normalZMat;
   private final GpuMat traversabilityMat;
   private final GpuMat traversabilityClassMat;

   /**
    * This class extracts terrain data from a height map.
    * That data is stored in a {@link TerrainMapData} object.
    * With the terrain data, we can plan precise footsteps over the height map
    *
    * @param heightMapParameters parameters used to compute terrain data
    */
   public TerrainMapExtractor(HeightMapParameters heightMapParameters, TerrainMapParameters terrainMapParameters)
   {
      this.heightMapParameters = heightMapParameters;
      this.terrainMapParameters = terrainMapParameters;

      downloadStreamOne = new Stream();
      downloadStreamTwo = new Stream();
      stream = CUDAStreamManager.getStream();

      // We have to try to load the kernel inside this try catch cause it may throw an exception
      try
      {
         // Load header and main file
         URL heightMapUtilsHeaderPath = getClass().getResource("/us/ihmc/perception/gpuMapping/HeightMapUtils.cuh");
         URL mathUtilsHeaderPath = getClass().getResource("/us/ihmc/perception/cuda/MathUtils.cuh");
         URL kernelPath = getClass().getResource("TerrainMapExtractor.cu");

         terrainMapProgram = new CUDAProgram(kernelPath, heightMapUtilsHeaderPath, mathUtilsHeaderPath);
         terrainMapKernel = terrainMapProgram.loadKernel("computeTerrainData");
         terrainMapKernel.enableKernelTimings(PRINT_TIMING_FOR_KERNELS);

         blockSize = new dim3(BLOCK_SIZE_XY, BLOCK_SIZE_XY, 1);
      }
      catch (Exception e)
      {
         throw new RuntimeException(e);
      }

      // This is the number of parameters being passed in as floats to the kernel
      terrainMapParametersHostPointer = new FloatPointer(11);
      terrainMapParametersDevicePointer = new FloatPointer();
      CUDATools.mallocAsync(terrainMapParametersDevicePointer, terrainMapParametersHostPointer.limit(), stream);

      computeDerivedParameters();

      terrainMapData = new TerrainMapData(heightMapParameters.getCellSize(), heightMapParameters.getGlobalWidthInMeters(), 0.0, 0.0);

      // Initialize matrices and images
      normalXMat = new GpuMat(cellsPerAxisTerrain, cellsPerAxisTerrain, opencv_core.CV_8UC1);
      normalYMat = new GpuMat(cellsPerAxisTerrain, cellsPerAxisTerrain, opencv_core.CV_8UC1);
      normalZMat = new GpuMat(cellsPerAxisTerrain, cellsPerAxisTerrain, opencv_core.CV_8UC1);
      traversabilityMat = new GpuMat(cellsPerAxisTerrain, cellsPerAxisTerrain, opencv_core.CV_32FC1);
      traversabilityClassMat = new GpuMat(cellsPerAxisTerrain, cellsPerAxisTerrain, opencv_core.CV_8UC1);
   }

   /**
    * Compute the value of the Mat objects based on the height map parameters.
    * This needs to be an odd number of the indexing will be messed up
    */
   private void computeDerivedParameters()
   {
      int terrainCenterIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getGlobalWidthInMeters(), heightMapParameters.getCellSize());
      cellsPerAxisTerrain = 2 * terrainCenterIndex + 1;
   }

   public void update(GpuMat gpuHeightMap, Point3DReadOnly gridCenter)
   {
      int error;

      // Populate parameters buffer for the terrain map kernel
      float[] terrainMapParametersArray = populationTerrainMapParameters();
      terrainMapParametersHostPointer.put(terrainMapParametersArray);

      // Handle memory allocation and copy values to the GPU
      CUDATools.memcpyAsync(terrainMapParametersDevicePointer, terrainMapParametersHostPointer, terrainMapParametersArray.length, stream);
      checkCUDAError();

      // Pass all the parameters to the kernel so that its setup to run correctly
      terrainMapKernel.withPointer(gpuHeightMap.data()).withLong(gpuHeightMap.step());
      terrainMapKernel.withPointer(traversabilityMat.data()).withLong(traversabilityMat.step());
      terrainMapKernel.withPointer(traversabilityClassMat.data()).withLong(traversabilityClassMat.step());
      terrainMapKernel.withPointer(normalXMat.data()).withLong(normalXMat.step());
      terrainMapKernel.withPointer(normalYMat.data()).withLong(normalYMat.step());
      terrainMapKernel.withPointer(normalZMat.data()).withLong(normalZMat.step());
      terrainMapKernel.withPointer(terrainMapParametersDevicePointer).withInt(cellsPerAxisTerrain);

      // Compute the correct number of threads to run with the kernel
      int terrainMapKernelGridSizeXY = (cellsPerAxisTerrain + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
      dim3 terrainKernelGridDim = new dim3(terrainMapKernelGridSizeXY, terrainMapKernelGridSizeXY, 1);

      // Run the kernel and check for errors
      terrainMapKernel.run(stream, terrainKernelGridDim, blockSize, 0);
      checkCUDAError();

      // This has to be done because we start to download to the CPU, so the data on the GPU needs to be finalized
      // We need to synchronize here so we can download the data to the CPU on other threads, fast
      error = cudaStreamSynchronize(stream);
      terrainKernelGridDim.close();
      CUDATools.checkCUDAError(error);

      // --------------------------- Download all the data from the GPU and set the terrain data object ----------------------------
      {
         Mat cpuHeightMap = new Mat();
         gpuHeightMap.download(cpuHeightMap, downloadStreamOne);

         Mat cpuNormalXMap = new Mat();
         normalXMat.download(cpuNormalXMap, downloadStreamTwo);

         Mat cpuNormalYMap = new Mat();
         normalYMat.download(cpuNormalYMap, downloadStreamOne);

         Mat cpuNormalZMap = new Mat();
         normalZMat.download(cpuNormalZMap, downloadStreamTwo);

         Mat cpuTraversabilityMap = new Mat();
         traversabilityMat.download(cpuTraversabilityMap, downloadStreamOne);

         Mat cpuTraversabilityClassMap = new Mat();
         traversabilityClassMat.download(cpuTraversabilityClassMap, downloadStreamTwo);

         // Going to start using the data on the CPU so let's make sure its actually on the CPU
         downloadStreamTwo.waitForCompletion();
         downloadStreamTwo.waitForCompletion();
         TerrainMapTools.convertToTerrainMapData(cpuHeightMap,
                                                 cpuNormalXMap,
                                                 cpuNormalYMap,
                                                 cpuNormalZMap,
                                                 cpuTraversabilityMap,
                                                 cpuTraversabilityClassMap,
                                                 gridCenter,
                                                 terrainMapData);

         cpuHeightMap.close();
         cpuNormalXMap.close();
         cpuNormalYMap.close();
         cpuNormalZMap.close();
         cpuTraversabilityMap.close();
         cpuTraversabilityClassMap.close();
      }
   }

   /**
    * Populate the parameter's array for the terrain map kernels.
    *
    * @return a float array with the parameters for the terrain map kernels. The order matters as it needs to match the order things are defined by in the kernel
    */
   public float[] populationTerrainMapParameters()
   {
      return new float[] {(float) heightMapParameters.getCellSize(),
                          (float) heightMapParameters.getGlobalWidthInMeters(),
                          (float) terrainMapParameters.getNormalSearchRadius(),
                          (float) terrainMapParameters.getCliffSearchRadius(),
                          (float) terrainMapParameters.getCliffHeightThreshold(),
                          (float) terrainMapParameters.getCliffHeightTolerance(),
                          (float) terrainMapParameters.getMinSupportAreaFraction(),
                          (float) terrainMapParameters.getMinSnapHeightThreshold(),
                          (float) terrainMapParameters.getSnapHeightThresholdAtSearchEdge(),
                          (float) terrainMapParameters.getSteppingCosineThreshold(),
                          (float) terrainMapParameters.getSquaredErrorThreshold()};
   }

   public void destroy()
   {
      terrainMapProgram.close();
      terrainMapKernel.close();

      normalXMat.close();
      normalYMat.close();
      normalZMat.close();
      traversabilityMat.close();
      traversabilityClassMat.close();

      int error = cudaFree(terrainMapParametersDevicePointer);
      terrainMapParametersHostPointer.close();
      terrainMapParametersDevicePointer.close();

      blockSize.close();
      CUDATools.checkCUDAError(error);

      // At the end we have to destroy the stream to release the memory
      downloadStreamOne.close();
      downloadStreamTwo.close();
      CUDAStreamManager.releaseStream(stream);
   }

   /**
    * If we are debugging the kernels with {@link TerrainMapExtractor#PRINT_TIMING_FOR_KERNELS} then we want to synchronize the GPU
    * The reason we synchronize because we are checking for errors, so this would help identify where the error is happening
    */
   private void checkCUDAError()
   {
      int error;
      if (PRINT_TIMING_FOR_KERNELS)
      {
         // Check for errors after the async calls
         error = cudaStreamSynchronize(stream);
         CUDATools.checkCUDAError(error);
      }
   }

   public TerrainMapData getTerrainMapData()
   {
      return terrainMapData;
   }
}
