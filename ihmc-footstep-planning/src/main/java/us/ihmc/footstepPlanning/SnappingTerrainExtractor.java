package us.ihmc.footstepPlanning;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.perception.cuda.CUDAKernel;
import us.ihmc.perception.cuda.CUDAProgram;
import us.ihmc.perception.cuda.CUDAStreamManager;
import us.ihmc.perception.cuda.CUDATools;
import us.ihmc.footstepPlanning.steppableRegions.TerrainMapData;
import us.ihmc.footstepPlanning.steppableRegions.SteppableRegionCalculatorParameters;
import us.ihmc.perception.heightMap.HeightMapData;
import us.ihmc.perception.heightMap.HeightMapParameters;
import us.ihmc.perception.heightMap.HeightMapTools;

import java.net.URL;

import static org.bytedeco.cuda.global.cudart.cudaFree;
import static org.bytedeco.cuda.global.cudart.cudaStreamSynchronize;

public class SnappingTerrainExtractor
{
   private static final boolean PRINT_TIMING_FOR_KERNELS = false;
   /**
    * The choice of 16 here is to utilize more SMs (Multi Processors) on the GPU.
    * This was chosen based on GPU profiling and significantly effects performance.
    */
   private static final int BLOCK_SIZE_XY = 16;

   private final TerrainMapData terrainMapData;
   private final HeightMapParameters heightMapParameters;
   private final SteppableRegionCalculatorParameters steppableRegionParameters = new SteppableRegionCalculatorParameters();

   private final CUstream_st stream;
   private final CUDAProgram snappingTerrainProgram;
   private final CUDAKernel snappingTerrainKernel;
   private final CUDAKernel terrainCostKernel;
   private final CUDAKernel contactMapKernel;
   private final CUDAKernel steppableConnectionsKernel;

   private dim3 snappingKernelGridDim;
   private dim3 terrainCostKernelGridDim;
   private dim3 contactMapKernelGridDim;
   private dim3 steppableConnectionsKernelGridDim;
   private dim3 blockSize;
   private int cellsPerAxisTerrain;

   private final FloatPointer snappingParametersHostPointer;
   private final FloatPointer snappingParametersDevicePointer;

   /**
    * The types of these mats depend on the data we are trying to store.
    * Check the {@link perception_msgs.msg.dds.TerrainMapMessage} to ensure these are the same
    */
   private final GpuMat terrainCostMat;
   private final GpuMat contactMat;
   private final GpuMat snapHeightMat;
   private final GpuMat snapNormalXMat;
   private final GpuMat snapNormalYMat;
   private final GpuMat snapNormalZMat;
   private final GpuMat snappedAreaFractionMat;
   private final GpuMat steppabilityMat;
   private final GpuMat steppabilityConnectionsMat;

   /**
    * This class extracts terrain data from a height map.
    * That data is stored in a {@link TerrainMapData} object.
    * With the terrain data, we can plan precise footsteps over the height map
    *
    * @param heightMapParameters parameters used to compute terrain data
    */
   public SnappingTerrainExtractor(HeightMapParameters heightMapParameters)
   {
      this.heightMapParameters = heightMapParameters;

      stream = CUDAStreamManager.getStream();

      // We have to try to load the kernel inside this try catch cause it may throw an exception
      try
      {
         // Load header and main file
         URL heightMapUtilsHeaderPath = getClass().getResource("/us/ihmc/perception/gpuHeightMap/HeightMapUtils.cuh");
         URL mathUtilsHeaderPath = getClass().getResource("/us/ihmc/perception/cuda/MathUtils.cuh");
         URL kernelPath = getClass().getResource("SnappingTerrainExtractor.cu");

         snappingTerrainProgram = new CUDAProgram(kernelPath, heightMapUtilsHeaderPath, mathUtilsHeaderPath);
         snappingTerrainKernel = snappingTerrainProgram.loadKernel("computeTerrainData");
         snappingTerrainKernel.enableKernelTimings(PRINT_TIMING_FOR_KERNELS);

         steppableConnectionsKernel = snappingTerrainProgram.loadKernel("computeSteppabilityConnections");
         steppableConnectionsKernel.enableKernelTimings(PRINT_TIMING_FOR_KERNELS);

         terrainCostKernel = snappingTerrainProgram.loadKernel("computeTerrainCost");
         terrainCostKernel.enableKernelTimings(PRINT_TIMING_FOR_KERNELS);

         contactMapKernel = snappingTerrainProgram.loadKernel("computeContactMap");
         contactMapKernel.enableKernelTimings(PRINT_TIMING_FOR_KERNELS);
      }
      catch (Exception e)
      {
         throw new RuntimeException(e);
      }

      // 16 is the number of parameters being passed in as floats
      snappingParametersHostPointer = new FloatPointer(19);
      snappingParametersDevicePointer = new FloatPointer();

      computeDerivedParameters();

      terrainMapData = new TerrainMapData(cellsPerAxisTerrain,
                                          cellsPerAxisTerrain,
                                          heightMapParameters.getHeightScaleFactor(),
                                          heightMapParameters.getHeightOffset(),
                                          heightMapParameters.getCellSize(),
                                          heightMapParameters.getTerrainWidthInMeters());

      // Initialize matrices and images
      terrainCostMat = new GpuMat(cellsPerAxisTerrain, cellsPerAxisTerrain, opencv_core.CV_8UC1);
      contactMat = new GpuMat(cellsPerAxisTerrain, cellsPerAxisTerrain, opencv_core.CV_8UC1);
      snapHeightMat = new GpuMat(cellsPerAxisTerrain, cellsPerAxisTerrain, opencv_core.CV_16UC1);
      snapNormalXMat = new GpuMat(cellsPerAxisTerrain, cellsPerAxisTerrain, opencv_core.CV_8UC1);
      snapNormalYMat = new GpuMat(cellsPerAxisTerrain, cellsPerAxisTerrain, opencv_core.CV_8UC1);
      snapNormalZMat = new GpuMat(cellsPerAxisTerrain, cellsPerAxisTerrain, opencv_core.CV_8UC1);
      snappedAreaFractionMat = new GpuMat(cellsPerAxisTerrain, cellsPerAxisTerrain, opencv_core.CV_8UC1);
      steppabilityMat = new GpuMat(cellsPerAxisTerrain, cellsPerAxisTerrain, opencv_core.CV_8UC1);
      steppabilityConnectionsMat = new GpuMat(cellsPerAxisTerrain, cellsPerAxisTerrain, opencv_core.CV_8UC1);
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

      Point2D gridCenter = heightMapData.getGridCenter();

      Mat heightMap = new Mat(heightMapData.getCellsPerAxis(), heightMapData.getCellsPerAxis(), opencv_core.CV_32FC1);
      HeightMapTools.convertHeightMapDataToMat(heightMap, heightMapData);
      GpuMat gpuHeightMap = new GpuMat();
      gpuHeightMap.upload(heightMap);

      // Populate parameters buffer for the snapping kernel
      float[] snappingParametersArray = populateSnappingParametersArray(gridCenter);
      snappingParametersHostPointer.put(snappingParametersArray);

      // Handle memory allocation and copy values to the GPU
      CUDATools.mallocAsync(snappingParametersDevicePointer, snappingParametersArray.length, stream);
      CUDATools.memcpyAsync(snappingParametersDevicePointer, snappingParametersHostPointer, snappingParametersArray.length, stream);
      checkCUDAError();

      // Pass all the parameters to the kernel so that its setup to run correctly
      snappingTerrainKernel.withPointer(gpuHeightMap.data()).withLong(gpuHeightMap.step());
      snappingTerrainKernel.withPointer(steppabilityMat.data()).withLong(steppabilityMat.step());
      snappingTerrainKernel.withPointer(snapHeightMat.data()).withLong(snapHeightMat.step());
      snappingTerrainKernel.withPointer(snapNormalXMat.data()).withLong(snapNormalXMat.step());
      snappingTerrainKernel.withPointer(snapNormalYMat.data()).withLong(snapNormalYMat.step());
      snappingTerrainKernel.withPointer(snapNormalZMat.data()).withLong(snapNormalZMat.step());
      snappingTerrainKernel.withPointer(snappedAreaFractionMat.data()).withLong(snappedAreaFractionMat.step());
      snappingTerrainKernel.withPointer(snappingParametersDevicePointer).withInt(cellsPerAxisTerrain);

      // Compute the correct number of threads to run with the kernel
      int snappedKernelGridSizeXY = (cellsPerAxisTerrain + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
      snappingKernelGridDim = new dim3(snappedKernelGridSizeXY, snappedKernelGridSizeXY, 1);
      blockSize = new dim3(BLOCK_SIZE_XY, BLOCK_SIZE_XY, 1);

      // Run the kernel and check for errors
      snappingTerrainKernel.run(stream, snappingKernelGridDim, blockSize, 0);
      checkCUDAError();

      // --------------------- Run additional kernels for even more data to be used with the terrain map ------------------

      terrainCostKernel.withPointer(gpuHeightMap.data()).withLong(gpuHeightMap.step());
      terrainCostKernel.withPointer(terrainCostMat.data()).withLong(terrainCostMat.step());
      terrainCostKernel.withPointer(snappingParametersDevicePointer);

      // Compute the correct number of threads to run with the kernel
      int terrainCostGridSizeXY = (cellsPerAxisTerrain + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
      terrainCostKernelGridDim = new dim3(terrainCostGridSizeXY, terrainCostGridSizeXY, 1);

      terrainCostKernel.run(stream, terrainCostKernelGridDim, blockSize, 0);
      checkCUDAError();

      contactMapKernel.withPointer(terrainCostMat.data()).withLong(terrainCostMat.step());
      contactMapKernel.withPointer(contactMat.data()).withLong(contactMat.step());
      contactMapKernel.withPointer(snappingParametersDevicePointer);

      // Compute the correct number of threads to run with the kernel
      int contactCostGridSizeXY = (cellsPerAxisTerrain + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
      contactMapKernelGridDim = new dim3(contactCostGridSizeXY, contactCostGridSizeXY, 1);

      contactMapKernel.run(stream, contactMapKernelGridDim, blockSize, 0);
      checkCUDAError();

      // --------------------- Run additional kernels for even more data to be used with the terrain map ------------------

      steppableConnectionsKernel.withPointer(steppabilityMat.data()).withLong(steppabilityMat.step());
      steppableConnectionsKernel.withPointer(steppabilityConnectionsMat.data()).withLong(steppabilityConnectionsMat.step());
      steppableConnectionsKernel.withPointer(snappingParametersDevicePointer);

      // Compute the correct number of threads to run with the kernel
      int steppableConnectionsGridSizeXY = (cellsPerAxisTerrain + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
      steppableConnectionsKernelGridDim = new dim3(steppableConnectionsGridSizeXY, steppableConnectionsGridSizeXY, 1);

      // Now that we have the steppability mat, run the steppable regions kernel and check for errors
      steppableConnectionsKernel.run(stream, steppableConnectionsKernelGridDim, blockSize, 0);
      checkCUDAError();

      // Update the terrain map data with the new results
      terrainMapData.setSensorOrigin(gridCenter.getX(), gridCenter.getY());

      // This has to be done because we start to download to the CPU, so the data on the GPU needs to be finalized
      error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);

      // --------------------------- Download all the data from the GPU and set the terrain data object ----------------------------
      {
         Mat cpuTerrainCostMap = new Mat();
         terrainCostMat.download(cpuTerrainCostMap);
         terrainMapData.setTerrainCostMap(cpuTerrainCostMap);
         cpuTerrainCostMap.close();

         Mat cpuContactMap = new Mat();
         contactMat.download(cpuContactMap);
         terrainMapData.setContactMap(cpuContactMap);
         cpuContactMap.close();

         Mat cpuHeightMap = new Mat();
         gpuHeightMap.download(cpuHeightMap);
         terrainMapData.setHeightMap(cpuHeightMap);
         cpuHeightMap.close();

         Mat cpuSnapNormalXMap = new Mat();
         snapNormalXMat.download(cpuSnapNormalXMap);
         terrainMapData.setSnapNormalXMat(cpuSnapNormalXMap);
         cpuSnapNormalXMap.close();

         Mat cpuSnapNormalYMap = new Mat();
         snapNormalYMat.download(cpuSnapNormalYMap);
         terrainMapData.setSnapNormalYMat(cpuSnapNormalYMap);
         cpuSnapNormalYMap.close();

         Mat cpuSnapNormalZMap = new Mat();
         snapNormalZMat.download(cpuSnapNormalZMap);
         terrainMapData.setSnapNormalZMat(cpuSnapNormalZMap);
         cpuSnapNormalZMap.close();

         Mat cpuSnappedAreaFractionMap = new Mat();
         snappedAreaFractionMat.download(cpuSnappedAreaFractionMap);
         terrainMapData.setSnappedAreaFractionMat(cpuSnappedAreaFractionMap);
         cpuSnappedAreaFractionMap.close();

         Mat cpuSteppabilityMap = new Mat();
         steppabilityMat.download(cpuSteppabilityMap);
         terrainMapData.setSteppabilityMat(cpuSteppabilityMap);
         cpuSteppabilityMap.close();

         Mat cpuSteppableConnections = new Mat();
         steppabilityConnectionsMat.download(cpuSteppableConnections);
         terrainMapData.setSteppabilityConnectionsMat(cpuSteppableConnections);
         cpuSteppableConnections.close();
      }
   }

   /**
    * If we are debugging the kernels with {@link SnappingTerrainExtractor#PRINT_TIMING_FOR_KERNELS} then we want to synchronize the GPU
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
    * @param gridCenter is the location of the sensor origin, that will be the center of our map
    * @return a float array with the parameters for the snapping kernels. The order matters as it needs to match the order things are defined by in the kernel
    */
   public float[] populateSnappingParametersArray(Tuple2DReadOnly gridCenter)
   {
      return new float[] {(float) gridCenter.getX(),
                          (float) gridCenter.getY(),
                          (float) heightMapParameters.getCellSize(),
                          (float) heightMapParameters.getTerrainWidthInMeters(),
                          (float) heightMapParameters.getHeightScaleFactor(),
                          (float) heightMapParameters.getHeightOffset(),
                          (float) steppableRegionParameters.getFootLength(),
                          (float) steppableRegionParameters.getFootWidth(),
                          (float) steppableRegionParameters.getDistanceFromCliffTops(),
                          (float) steppableRegionParameters.getDistanceFromCliffBottoms(),
                          (float) steppableRegionParameters.getCliffStartHeightToAvoid(),
                          (float) steppableRegionParameters.getCliffEndHeightToAvoid(),
                          (float) steppableRegionParameters.getMinSupportAreaFraction(),
                          (float) steppableRegionParameters.getMinSnapHeightThreshold(),
                          (float) steppableRegionParameters.getSnapHeightThresholdAtSearchEdge(),
                          (float) steppableRegionParameters.getInequalityActivationSlope(),
                          (float) heightMapParameters.getSteppingCosineThreshold(),
                          (float) heightMapParameters.getSteppingContactThreshold(),
                          (float) heightMapParameters.getContactWindowSize()};
   }

   public void close()
   {
      snappingTerrainProgram.close();
      snappingTerrainKernel.close();
      terrainCostKernel.close();
      contactMapKernel.close();
      steppableConnectionsKernel.close();
      snappingKernelGridDim.close();
      terrainCostKernelGridDim.close();
      contactMapKernelGridDim.close();
      steppableConnectionsKernelGridDim.close();
      blockSize.close();

      snapHeightMat.close();
      snapNormalXMat.close();
      snapNormalYMat.close();
      snapNormalZMat.close();
      snappedAreaFractionMat.close();
      steppabilityMat.close();
      steppabilityConnectionsMat.close();

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
