package us.ihmc.perception.heightMap;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
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
   private final SteppableRegionCalculatorParameters steppableRegionParameters;

   private final CUstream_st stream;
   private final CUDAProgram snappingTerrainProgram;
   private final CUDAKernel snappingTerrainKernel;

   private dim3 snappingKernelGridDim;
   private dim3 blockSize;
   private int cellsPerAxisTerrain;

   private final FloatPointer snappingParametersHostPointer;
   private final FloatPointer snappingParametersDevicePointer;

   /**
    * The types of these mats depend on the data we are trying to store.
    * Check the {@link perception_msgs.msg.dds.TerrainMapMessage} to ensure these are the same
    */
   private final GpuMat snapNormalXMat;
   private final GpuMat snapNormalYMat;
   private final GpuMat snapNormalZMat;
   private final GpuMat traversabilityMat;
   private final GpuMat traversabilityClassMat;

   /**
    * This class extracts terrain data from a height map.
    * That data is stored in a {@link TerrainMapData} object.
    * With the terrain data, we can plan precise footsteps over the height map
    *
    * @param heightMapParameters parameters used to compute terrain data
    */
   public TerrainMapExtractor(HeightMapParameters heightMapParameters, SteppableRegionCalculatorParameters steppableRegionCalculatorParameters)
   {
      this.heightMapParameters = heightMapParameters;
      this.steppableRegionParameters = steppableRegionCalculatorParameters;

      stream = CUDAStreamManager.getStream();

      // We have to try to load the kernel inside this try catch cause it may throw an exception
      try
      {
         // Load header and main file
         URL heightMapUtilsHeaderPath = getClass().getResource("/us/ihmc/perception/heightMap/HeightMapUtils.cuh");
         URL mathUtilsHeaderPath = getClass().getResource("/us/ihmc/perception/cuda/MathUtils.cuh");
         URL kernelPath = getClass().getResource("TerrainMapExtractor.cu");

         snappingTerrainProgram = new CUDAProgram(kernelPath, heightMapUtilsHeaderPath, mathUtilsHeaderPath);
         snappingTerrainKernel = snappingTerrainProgram.loadKernel("computeTerrainData");
         snappingTerrainKernel.enableKernelTimings(PRINT_TIMING_FOR_KERNELS);
      }
      catch (Exception e)
      {
         throw new RuntimeException(e);
      }

      // This is the number of parameters being passed in as floats to the kernel
      snappingParametersHostPointer = new FloatPointer(18);
      snappingParametersDevicePointer = new FloatPointer();

      computeDerivedParameters();

      terrainMapData = new TerrainMapData(heightMapParameters.getCellSize(), heightMapParameters.getTerrainWidthInMeters(), 0.0, 0.0);

      // Initialize matrices and images
      snapNormalXMat = new GpuMat(cellsPerAxisTerrain, cellsPerAxisTerrain, opencv_core.CV_8UC1);
      snapNormalYMat = new GpuMat(cellsPerAxisTerrain, cellsPerAxisTerrain, opencv_core.CV_8UC1);
      snapNormalZMat = new GpuMat(cellsPerAxisTerrain, cellsPerAxisTerrain, opencv_core.CV_8UC1);
      traversabilityMat = new GpuMat(cellsPerAxisTerrain, cellsPerAxisTerrain, opencv_core.CV_32FC1);
      traversabilityClassMat = new GpuMat(cellsPerAxisTerrain, cellsPerAxisTerrain, opencv_core.CV_8UC1);
   }

   /**
    * Compute the value of the Mat objects based on the height map parameters.
    * This needs to be an odd number of the indexing will be messed up
    */
   private void computeDerivedParameters()
   {
      int terrainCenterIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getTerrainWidthInMeters(), heightMapParameters.getCellSize());
      cellsPerAxisTerrain = 2 * terrainCenterIndex + 1;
   }

   public void update(HeightMapData heightMapData)
   {
      int error;

      terrainMapData.setHeightMapData(heightMapData);

      Mat heightMap = new Mat(heightMapData.getCellsPerAxis(), heightMapData.getCellsPerAxis(), opencv_core.CV_32FC1);
      HeightMapTools.convertHeightMapDataToMat(heightMap, heightMapData);
      GpuMat gpuHeightMap = new GpuMat();
      gpuHeightMap.upload(heightMap);

      // Populate parameters buffer for the snapping kernel
      float[] snappingParametersArray = populateSnappingParametersArray();
      snappingParametersHostPointer.put(snappingParametersArray);

      // Handle memory allocation and copy values to the GPU
      CUDATools.mallocAsync(snappingParametersDevicePointer, snappingParametersArray.length, stream);
      CUDATools.memcpyAsync(snappingParametersDevicePointer, snappingParametersHostPointer, snappingParametersArray.length, stream);
      checkCUDAError();

      // Pass all the parameters to the kernel so that its setup to run correctly
      snappingTerrainKernel.withPointer(gpuHeightMap.data()).withLong(gpuHeightMap.step());
      snappingTerrainKernel.withPointer(traversabilityMat.data()).withLong(traversabilityMat.step());
      snappingTerrainKernel.withPointer(traversabilityClassMat.data()).withLong(traversabilityClassMat.step());
      snappingTerrainKernel.withPointer(snapNormalXMat.data()).withLong(snapNormalXMat.step());
      snappingTerrainKernel.withPointer(snapNormalYMat.data()).withLong(snapNormalYMat.step());
      snappingTerrainKernel.withPointer(snapNormalZMat.data()).withLong(snapNormalZMat.step());
      snappingTerrainKernel.withPointer(snappingParametersDevicePointer).withInt(cellsPerAxisTerrain);

      // Compute the correct number of threads to run with the kernel
      int snappedKernelGridSizeXY = (cellsPerAxisTerrain + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
      snappingKernelGridDim = new dim3(snappedKernelGridSizeXY, snappedKernelGridSizeXY, 1);
      blockSize = new dim3(BLOCK_SIZE_XY, BLOCK_SIZE_XY, 1);

      // Run the kernel and check for errors
      snappingTerrainKernel.run(stream, snappingKernelGridDim, blockSize, 0);
      checkCUDAError();

      // This has to be done because we start to download to the CPU, so the data on the GPU needs to be finalized
      error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);

      // --------------------------- Download all the data from the GPU and set the terrain data object ----------------------------
      {
         Mat cpuHeightMap = new Mat();
         gpuHeightMap.download(cpuHeightMap);

         Mat cpuSnapNormalXMap = new Mat();
         snapNormalXMat.download(cpuSnapNormalXMap);

         Mat cpuSnapNormalYMap = new Mat();
         snapNormalYMat.download(cpuSnapNormalYMap);

         Mat cpuSnapNormalZMap = new Mat();
         snapNormalZMat.download(cpuSnapNormalZMap);

         Mat cpuTraversabilityMap = new Mat();
         traversabilityMat.download(cpuTraversabilityMap);

         Mat cpuTraversabilityClassMap = new Mat();
         traversabilityClassMat.download(cpuTraversabilityClassMap);

         TerrainMapTools.convertToTerrainMapData(cpuSnapNormalXMap,
                                                 cpuSnapNormalYMap,
                                                 cpuSnapNormalZMap,
                                                 cpuTraversabilityMap,
                                                 cpuTraversabilityClassMap,
                                                 terrainMapData);

         cpuHeightMap.close();
         cpuSnapNormalXMap.close();
         cpuSnapNormalYMap.close();
         cpuSnapNormalZMap.close();
         cpuTraversabilityMap.close();
         cpuTraversabilityClassMap.close();
      }
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

   /**
    * Populate the parameter's array for the snapping kernels.
    *
    * @return a float array with the parameters for the snapping kernels. The order matters as it needs to match the order things are defined by in the kernel
    */
   public float[] populateSnappingParametersArray()
   {
      return new float[] {(float) heightMapParameters.getCellSize(),
                          (float) heightMapParameters.getTerrainWidthInMeters(),
                          (float) steppableRegionParameters.getNormalSearchRadius(),
                          (float) steppableRegionParameters.getCliffSearchRadius(),
                          (float) steppableRegionParameters.getCliffHeightThreshold(),
                          (float) steppableRegionParameters.getMinSupportAreaFraction(),
                          (float) steppableRegionParameters.getMinSnapHeightThreshold(),
                          (float) steppableRegionParameters.getSnapHeightThresholdAtSearchEdge(),
                          (float) steppableRegionParameters.getSteppingCosineThreshold(),
                          (float) steppableRegionParameters.getSquaredErrorThreshold()};
   }

   public void destroy()
   {
      snappingTerrainProgram.close();
      snappingTerrainKernel.close();
      snappingKernelGridDim.close();
      blockSize.close();

      snapNormalXMat.close();
      snapNormalYMat.close();
      snapNormalZMat.close();
      traversabilityMat.close();
      traversabilityClassMat.close();

      snappingParametersHostPointer.close();
      snappingParametersDevicePointer.close();

      int error = cudaFree(snappingParametersDevicePointer);
      CUDATools.checkCUDAError(error);

      // At the end we have to destroy the stream to release the memory
      CUDAStreamManager.releaseStream(stream);
   }

   public TerrainMapData getTerrainMapData()
   {
      return terrainMapData;
   }
}
