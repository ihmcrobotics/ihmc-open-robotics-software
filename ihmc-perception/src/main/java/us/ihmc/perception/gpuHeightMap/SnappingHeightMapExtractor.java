package us.ihmc.perception.gpuHeightMap;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.perception.cuda.CUDAKernel;
import us.ihmc.perception.cuda.CUDAProgram;
import us.ihmc.perception.cuda.CUDAStreamManager;
import us.ihmc.perception.cuda.CUDATools;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.perception.steppableRegions.SteppableRegionCalculatorParameters;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;

import java.net.URL;

import static org.bytedeco.cuda.global.cudart.cudaFree;
import static org.bytedeco.cuda.global.cudart.cudaStreamSynchronize;

public class SnappingHeightMapExtractor
{
   private static final int BLOCK_SIZE_XY = 32;
   private static final boolean PRINT_TIMING_FOR_KERNELS = false;

   private final SteppableRegionCalculatorParameters steppableRegionParameters = new SteppableRegionCalculatorParameters();

   private final HeightMapParameters heightMapParameters;
   private final TerrainMapData terrainMapData;

   private final CUstream_st stream;

   private final CUDAProgram snappingHeightMapProgram;
   private final CUDAKernel snappingKernel;
   private dim3 snappingKernelGridDim;
   private dim3 blockSize;

   private final FloatPointer snappingParametersHostPointer;
   private final FloatPointer snappingParametersDevicePointer;

   private final GpuMat steppabilityImage;
   private final GpuMat snapHeightImage;
   private final GpuMat snapNormalXImage;
   private final GpuMat snapNormalYImage;
   private final GpuMat snapNormalZImage;
   private final GpuMat snappedAreaFractionImage;
   private int cellsPerAxisTerrain;

   public SnappingHeightMapExtractor(HeightMapParameters heightMapParameters)
   {
      this.heightMapParameters = heightMapParameters;

      try
      {
         stream = CUDAStreamManager.getStream();

         // Load header and main file
         URL heightMapUtilsHeaderPath = getClass().getResource("HeightMapUtils.cuh");
         URL mathUtilsHeaderPath = getClass().getResource("/us/ihmc/perception/cuda/MathUtils.cuh");
         URL kernelPath = getClass().getResource("SnappingHeightMapExtractor.cu");

         snappingHeightMapProgram = new CUDAProgram(kernelPath, heightMapUtilsHeaderPath, mathUtilsHeaderPath);
         snappingKernel = snappingHeightMapProgram.loadKernel("computeSnappedValuesKernel");
         snappingKernel.enableKernelTimings(PRINT_TIMING_FOR_KERNELS);

         snappingParametersHostPointer = new FloatPointer(16);
         snappingParametersDevicePointer = new FloatPointer();
      }
      catch (Exception e)
      {
         throw new RuntimeException(e);
      }

      recomputeDerivedParameters();

      terrainMapData = new TerrainMapData(cellsPerAxisTerrain,
                                          cellsPerAxisTerrain,
                                          heightMapParameters.getHeightScaleFactor(),
                                          heightMapParameters.getHeightOffset());

      // Initialize matrices and images
      steppabilityImage = new GpuMat(cellsPerAxisTerrain, cellsPerAxisTerrain, opencv_core.CV_8UC1);
      snapHeightImage = new GpuMat(cellsPerAxisTerrain, cellsPerAxisTerrain, opencv_core.CV_16UC1);
      snapNormalXImage = new GpuMat(cellsPerAxisTerrain, cellsPerAxisTerrain, opencv_core.CV_8UC1);
      snapNormalYImage = new GpuMat(cellsPerAxisTerrain, cellsPerAxisTerrain, opencv_core.CV_8UC1);
      snapNormalZImage = new GpuMat(cellsPerAxisTerrain, cellsPerAxisTerrain, opencv_core.CV_8UC1);
      snappedAreaFractionImage = new GpuMat(cellsPerAxisTerrain, cellsPerAxisTerrain, opencv_core.CV_8UC1);
   }

   private void recomputeDerivedParameters()
   {
      int terrainCenterIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getTerrainWidthInMeters(), heightMapParameters.getCellSizeInMeters());
      cellsPerAxisTerrain = 2 * terrainCenterIndex + 1;
   }

   public void update(GpuMat terrainHeightMap, Point3D sensorOrigin)
   {
      int error;

      recomputeDerivedParameters();

      // Populate parameters buffer for the snapping kernel
      float[] snappingParametersArray = populateSnappingParametersArray(sensorOrigin);
      snappingParametersHostPointer.put(snappingParametersArray);

      // Handle memory allocation and copy values to the GPU
      CUDATools.mallocAsync(snappingParametersDevicePointer, snappingParametersArray.length, stream);
      CUDATools.memcpyAsync(snappingParametersDevicePointer, snappingParametersHostPointer, snappingParametersArray.length, stream);

      // Pass all the parameters to the kernel so that its setup to run correctly
      snappingKernel.withPointer(terrainHeightMap.data()).withLong(terrainHeightMap.step());
      snappingKernel.withPointer(steppabilityImage.data()).withLong(steppabilityImage.step());
      snappingKernel.withPointer(snapHeightImage.data()).withLong(snapHeightImage.step());
      snappingKernel.withPointer(snapNormalXImage.data()).withLong(snapNormalXImage.step());
      snappingKernel.withPointer(snapNormalYImage.data()).withLong(snapNormalYImage.step());
      snappingKernel.withPointer(snapNormalZImage.data()).withLong(snapNormalZImage.step());
      snappingKernel.withPointer(snappedAreaFractionImage.data()).withLong(snappedAreaFractionImage.step());
      snappingKernel.withPointer(snappingParametersDevicePointer).withInt(cellsPerAxisTerrain);

      // Compute the correct number of threads to run with the kernel
      int snappedKernelGridSizeXY = (cellsPerAxisTerrain + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
      snappingKernelGridDim = new dim3(snappedKernelGridSizeXY, snappedKernelGridSizeXY, 1);
      blockSize = new dim3(BLOCK_SIZE_XY, BLOCK_SIZE_XY, 1);

      // Run the kernel and check for errors
      snappingKernel.run(stream, snappingKernelGridDim, blockSize, 0);
      error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);

      // Update the terrain map data with the new results
      terrainMapData.setSensorOrigin(sensorOrigin.getX(), sensorOrigin.getY());

      // Download all the data from the GPU and set the terrain data object
      {
         Mat cpuHeightMap = new Mat();
         terrainHeightMap.download(cpuHeightMap);
         terrainMapData.setHeightMap(cpuHeightMap);

         Mat cpuSteppabilityMap = new Mat();
         steppabilityImage.download(cpuSteppabilityMap);
         terrainMapData.setSteppabilityMat(cpuSteppabilityMap);

         Mat cpuSnapHeightMap = new Mat();
         snapHeightImage.download(cpuSnapHeightMap);
         terrainMapData.setSnapHeightMat(cpuSnapHeightMap);

         Mat cpuSnapNormalXMap = new Mat();
         snapNormalXImage.download(cpuSnapNormalXMap);
         terrainMapData.setSnapNormalXMat(cpuSnapNormalXMap);

         Mat cpuSnapNormalYMap = new Mat();
         snapNormalYImage.download(cpuSnapNormalYMap);
         terrainMapData.setSnapNormalYMat(cpuSnapNormalYMap);

         Mat cpuSnapNormalZMap = new Mat();
         snapNormalZImage.download(cpuSnapNormalZMap);
         terrainMapData.setSnapNormalZMat(cpuSnapNormalZMap);

         Mat cpuSnappedAreaFractionMap = new Mat();
         snappedAreaFractionImage.download(cpuSnappedAreaFractionMap);
         terrainMapData.setSnappedAreaFractionMat(cpuSnappedAreaFractionMap);
      }
   }

   /**
    * Populate the parameter's array for the snapping kernels.
    *
    * @param gridCenter is the location of the sensor origin, that will be the center of our map
    * @return a float array with the parameters for the snapping kernels. The order matters as it needs to match the order things are defined by in the kernel
    */
   public float[] populateSnappingParametersArray(Tuple3DReadOnly gridCenter)
   {
      return new float[] {(float) gridCenter.getX(),
                          (float) gridCenter.getY(),
                          (float) heightMapParameters.getCellSizeInMeters(),
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
                          (float) steppableRegionParameters.getInequalityActivationSlope()};
   }

   public void destroy()
   {
      snappingHeightMapProgram.close();
      snappingKernel.close();
      snappingKernelGridDim.close();
      blockSize.close();

      steppabilityImage.close();
      snapHeightImage.close();
      snapNormalXImage.close();
      snapNormalYImage.close();
      snapNormalZImage.close();
      snappedAreaFractionImage.close();

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
