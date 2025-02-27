package us.ihmc.perception.gpuHeightMap;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.cuda.CUDAKernel;
import us.ihmc.perception.cuda.CUDAProgram;
import us.ihmc.perception.cuda.CUDAStreamManager;
import us.ihmc.perception.cuda.CUDATools;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;

import java.net.URL;

import static org.bytedeco.cuda.global.cudart.*;

public class RapidHeightMapExtractorCUDA implements RapidHeightMapExtractorInterface
{
   private static final boolean PRINT_TIMING_FOR_KERNELS = false;
   static final int BLOCK_SIZE_XY = 32;

   private final SideDependentList<ReferenceFrame> footSoleFrames = new SideDependentList<>();
   private final TerrainMapData terrainMapData;
   private final CameraIntrinsics cameraIntrinsics;
   private final Point3D sensorOrigin = new Point3D();
   private final int mode; // 0 -> Ouster, 1 -> Realsense
   private final HeightMapParameters heightMapParameters;

   private final GpuMat inputDepthImage;
   private final GpuMat localHeightMapImage;
   private GpuMat globalHeightMapImage;
   private final GpuMat terrainCostImage;
   private final GpuMat contactMapImage;
   private final GpuMat sensorCroppedHeightMapImage;
   private final GpuMat emptyGlobalHeightMapImage;
   private final CUDAProgram heightMapCUDAProgram;

   private final CUstream_st stream;
   private final CUDAKernel updateKernel;
   private final CUDAKernel registerKernel;
   private final CUDAKernel croppingKernel;
   private final CUDAKernel planOffsetKernel;
   private final CUDAKernel emptyRegisterKernel;

   private final float[] worldToGroundTransformArray = new float[16];
   private final float[] groundToWorldTransformArray = new float[16];
   private final float[] groundToSensorTransformArray = new float[16];
   private final float[] sensorToGroundTransformArray = new float[16];

   private final FloatPointer groundToSensorTransformHostPointer;
   private final FloatPointer groundToSensorTransformDevicePointer;
   private final FloatPointer sensorToGroundTransformHostPointer;
   private final FloatPointer sensorToGroundTransformDevicePointer;
   private final FloatPointer worldToGroundTransformHostPointer;
   private final FloatPointer worldToGroundTransformDevicePointer;
   private final FloatPointer parametersHostPointer;
   private final FloatPointer parametersDevicePointer;
   private final FilteredRapidHeightMapExtractor filteredRapidHeightMapExtractor;

   public int sequenceNumber = 0;
   private float gridOffsetX;
   private int centerIndex;
   private int localCellsPerAxis;
   private int globalCenterIndex;
   private int cropCenterIndex;
   private int globalCellsPerAxis;

   private dim3 blockSize;
   private dim3 updateKernelGridDim;
   private dim3 registerKernelGridDim;
   private dim3 croppingKernelGridDim;
   private dim3 planOffsetKernelGridDim;
   private int resetOffset;

   private final SnappingHeightMapExtractor snappedFootstepsExtractor;

   public RapidHeightMapExtractorCUDA(ReferenceFrame leftFootSoleFrame,
                                      ReferenceFrame rightFootSoleFrame,
                                      GpuMat depthImage,
                                      CameraIntrinsics depthImageIntrinsics,
                                      int mode,
                                      HeightMapParameters heightMapParameters)
   {
      inputDepthImage = depthImage;
      this.cameraIntrinsics = depthImageIntrinsics;
      this.mode = mode;
      this.heightMapParameters = heightMapParameters;

      footSoleFrames.put(RobotSide.LEFT, leftFootSoleFrame);
      footSoleFrames.put(RobotSide.RIGHT, rightFootSoleFrame);

      stream = CUDAStreamManager.getStream();

      // Load header and main file
      URL heightMapUtilsHeaderPath = getClass().getResource("HeightMapUtils.cuh");
      URL mathUtilsHeaderPath = getClass().getResource("/us/ihmc/perception/cuda/MathUtils.cuh");
      URL kernelPath = getClass().getResource("RapidHeightMapExtractor.cu");

      terrainMapData = new TerrainMapData(heightMapParameters.getCropWindowSize(), heightMapParameters.getCropWindowSize());

      recomputeDerivedParameters();
      // Need to initialize this after the parameters have been computed to get the right size
      filteredRapidHeightMapExtractor = new FilteredRapidHeightMapExtractor(stream, globalCellsPerAxis, globalCellsPerAxis, 6);

      try
      {
         heightMapCUDAProgram = new CUDAProgram(kernelPath, heightMapUtilsHeaderPath, mathUtilsHeaderPath);

         updateKernel = heightMapCUDAProgram.loadKernel("heightMapUpdateKernel");
         registerKernel = heightMapCUDAProgram.loadKernel("heightMapRegistrationKernel");
         croppingKernel = heightMapCUDAProgram.loadKernel("croppingKernel");
         planOffsetKernel = heightMapCUDAProgram.loadKernel("planOffsetKernel");
         emptyRegisterKernel = heightMapCUDAProgram.loadKernel("heightMapRegistrationKernel");

         updateKernel.enableKernelTimings(PRINT_TIMING_FOR_KERNELS);
         registerKernel.enableKernelTimings(PRINT_TIMING_FOR_KERNELS);
         croppingKernel.enableKernelTimings(PRINT_TIMING_FOR_KERNELS);
         planOffsetKernel.enableKernelTimings(PRINT_TIMING_FOR_KERNELS);
         emptyRegisterKernel.enableKernelTimings(PRINT_TIMING_FOR_KERNELS);

         // Initialize matrices and images
         localHeightMapImage = new GpuMat(localCellsPerAxis, localCellsPerAxis, opencv_core.CV_16UC1);
         globalHeightMapImage = new GpuMat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_16UC1);
         terrainCostImage = new GpuMat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_8UC1);
         contactMapImage = new GpuMat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_8UC1);
         sensorCroppedHeightMapImage = new GpuMat(heightMapParameters.getCropWindowSize(), heightMapParameters.getCropWindowSize(), opencv_core.CV_16UC1);

         emptyGlobalHeightMapImage = new GpuMat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_16UC1);

         // Initialize transformation pointers
         groundToSensorTransformHostPointer = new FloatPointer(16);
         groundToSensorTransformDevicePointer = new FloatPointer();

         sensorToGroundTransformHostPointer = new FloatPointer(16);
         sensorToGroundTransformDevicePointer = new FloatPointer();

         worldToGroundTransformHostPointer = new FloatPointer(16);
         worldToGroundTransformDevicePointer = new FloatPointer();

         parametersHostPointer = new FloatPointer(37);
         parametersDevicePointer = new FloatPointer();

         snappedFootstepsExtractor = new SnappingHeightMapExtractor(heightMapParameters, terrainMapData);
      }
      catch (Exception e)
      {
         throw new RuntimeException(e);
      }

      reset();
   }

   private void recomputeDerivedParameters()
   {
      centerIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getLocalWidthInMeters(), heightMapParameters.getLocalCellSizeInMeters());
      localCellsPerAxis = 2 * centerIndex + 1;
      gridOffsetX = (float) heightMapParameters.getLocalWidthInMeters() / 2.0f;
      globalCenterIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getInternalGlobalWidthInMeters(),
                                                            heightMapParameters.getInternalGlobalCellSizeInMeters());
      globalCellsPerAxis = 2 * globalCenterIndex + 1;

      cropCenterIndex = (heightMapParameters.getCropWindowSize() - 1) / 2;

      if (2 * cropCenterIndex + 1 != heightMapParameters.getCropWindowSize())
         throw new RuntimeException("The crop center index was computed incorrectly.");
   }

   public void reset()
   {
      double thicknessOfTheFoot = 0.02;
      double height = 0.0;

      if (footSoleFrames.sides().length == 2)
      {
         height = Math.min(footSoleFrames.get(RobotSide.LEFT).getTransformToWorldFrame().getTranslationZ(),
                           footSoleFrames.get(RobotSide.RIGHT).getTransformToWorldFrame().getTranslationZ()) - thicknessOfTheFoot;
      }
      resetOffset = (int) ((height + heightMapParameters.getHeightOffset()) * heightMapParameters.getHeightScaleFactor());

      localHeightMapImage.setTo(new Scalar(resetOffset));
      globalHeightMapImage.setTo(new Scalar(resetOffset));
      emptyGlobalHeightMapImage.setTo(new Scalar(resetOffset));

      filteredRapidHeightMapExtractor.reset();
      snappedFootstepsExtractor.reset(resetOffset);

      sequenceNumber = 0;
   }

   public void update(RigidBodyTransform sensorToWorldTransform, RigidBodyTransform sensorToGroundTransform, RigidBodyTransform groundToWorldTransform)
   {
      int error;

      // Update the Z translation of the sensor to match the world transform (to handle the sensor's vertical position)
      sensorToGroundTransform.getTranslation().setZ(sensorToWorldTransform.getTranslationZ());

      // Compute the inverse transforms for later use
      RigidBodyTransform groundToSensorTransform = new RigidBodyTransform(sensorToGroundTransform);
      groundToSensorTransform.invert();
      RigidBodyTransform worldToGroundTransform = new RigidBodyTransform(groundToWorldTransform);
      worldToGroundTransform.invert();

      //Store the sensor's origin for later use in parameter population
      sensorOrigin.set(sensorToWorldTransform.getTranslation());

      // Populate parameter buffers with the necessary values
      float[] parametersArray = populateParameterArray(heightMapParameters, cameraIntrinsics, sensorOrigin);
      parametersHostPointer.put(parametersArray);

      //Extract the transform arrays for memory transfer
      groundToSensorTransform.get(groundToSensorTransformArray);
      sensorToGroundTransform.get(sensorToGroundTransformArray);
      worldToGroundTransform.get(worldToGroundTransformArray);
      groundToWorldTransform.get(groundToWorldTransformArray);

      //Transfer the transform arrays to the host memory
      groundToSensorTransformHostPointer.put(groundToSensorTransformArray);
      sensorToGroundTransformHostPointer.put(sensorToGroundTransformArray);
      worldToGroundTransformHostPointer.put(worldToGroundTransformArray);

      //Allocate memory on the GPU for each of the transforms and images
      //This step involves allocating CUDA memory asynchronously, and it's important to check for allocation errors
      CUDATools.mallocAsync(groundToSensorTransformDevicePointer, groundToSensorTransformArray.length, stream);
      CUDATools.mallocAsync(sensorToGroundTransformDevicePointer, sensorToGroundTransformArray.length, stream);
      CUDATools.mallocAsync(worldToGroundTransformDevicePointer, worldToGroundTransformArray.length, stream);
      CUDATools.mallocAsync(parametersDevicePointer, parametersArray.length, stream);

      error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);

      //Copy the data from host memory to device memory asynchronously
      // This ensures the device has the latest data available for kernel processing
      CUDATools.memcpyAsync(groundToSensorTransformDevicePointer, groundToSensorTransformHostPointer, groundToSensorTransformArray.length, stream);
      CUDATools.memcpyAsync(sensorToGroundTransformDevicePointer, sensorToGroundTransformHostPointer, sensorToGroundTransformArray.length, stream);
      CUDATools.memcpyAsync(worldToGroundTransformDevicePointer, worldToGroundTransformHostPointer, worldToGroundTransformArray.length, stream);
      CUDATools.memcpyAsync(parametersDevicePointer, parametersHostPointer, parametersArray.length, stream);

      error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);

      //Execute the CUDA kernels with the provided stream
      //Each kernel performs a specific task related to the height map update, registration, and cropping
      int updateKernelGridSizeXY = (localCellsPerAxis + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
      int registerKernelGridSizeXY = (globalCellsPerAxis + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
      int croppingKernelGridSizeXY = (heightMapParameters.getCropWindowSize() + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;

      blockSize = new dim3(BLOCK_SIZE_XY, BLOCK_SIZE_XY, 1);
      updateKernelGridDim = new dim3(updateKernelGridSizeXY, updateKernelGridSizeXY, 1);
      registerKernelGridDim = new dim3(registerKernelGridSizeXY, registerKernelGridSizeXY, 1);
      croppingKernelGridDim = new dim3(croppingKernelGridSizeXY, croppingKernelGridSizeXY, 1);

      // Run the update kernel
      updateKernel.withPointer(inputDepthImage.data()).withLong(inputDepthImage.step());
      updateKernel.withPointer(localHeightMapImage.data()).withLong(localHeightMapImage.step());
      updateKernel.withPointer(parametersDevicePointer);
      updateKernel.withPointer(sensorToGroundTransformDevicePointer);
      updateKernel.withPointer(groundToSensorTransformDevicePointer);
      updateKernel.withInt(localCellsPerAxis);

      updateKernel.run(stream, updateKernelGridDim, blockSize, 0);
      error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);

      // Run the registration kernel
      registerKernel.withPointer(localHeightMapImage.data()).withLong(localHeightMapImage.step());
      registerKernel.withPointer(globalHeightMapImage.data()).withLong(globalHeightMapImage.step());
      registerKernel.withPointer(parametersDevicePointer);
      registerKernel.withPointer(worldToGroundTransformDevicePointer);
      registerKernel.withPointer(sensorToGroundTransformDevicePointer);

      registerKernel.run(stream, registerKernelGridDim, blockSize, 0);
      error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);

      if (heightMapParameters.getEnableAlphaFilter())
      {
         GpuMat filteredHeightMap = filteredRapidHeightMapExtractor.update(globalHeightMapImage);
         globalHeightMapImage.close();
         globalHeightMapImage = filteredHeightMap;
      }

      // Run the cropping kernel
      croppingKernel.withPointer(globalHeightMapImage.data()).withLong(globalHeightMapImage.step());
      croppingKernel.withPointer(sensorCroppedHeightMapImage.data()).withLong(sensorCroppedHeightMapImage.step());
      croppingKernel.withPointer(parametersDevicePointer);
      croppingKernel.withInt(heightMapParameters.getCropWindowSize());
      error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);

      croppingKernel.run(stream, croppingKernelGridDim, blockSize, 0);
      error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);

      snappedFootstepsExtractor.update(globalHeightMapImage, sensorOrigin, globalCenterIndex, cropCenterIndex);

      //Update the terrain map data with the new results
      terrainMapData.setSensorOrigin(groundToWorldTransform.getTranslationX(), groundToWorldTransform.getTranslationY());

      Mat finalCroppedHeightMap = new Mat();  // Assuming the height map is 201x201
      sensorCroppedHeightMapImage.download(finalCroppedHeightMap);  // Download the image from the GPU to the Mat object
      error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);
      terrainMapData.setHeightMap(finalCroppedHeightMap);
   }

   @Override
   public void updateHeightOffset(float z)
   {
      int error;

      // Populate parameter buffers with the necessary values
      float[] parametersArray = populateParameterArray(heightMapParameters, cameraIntrinsics, sensorOrigin);
      parametersHostPointer.put(parametersArray);

      CUDATools.mallocAsync(worldToGroundTransformDevicePointer, worldToGroundTransformArray.length, stream);
      CUDATools.mallocAsync(sensorToGroundTransformDevicePointer, sensorToGroundTransformArray.length, stream);
      CUDATools.mallocAsync(parametersDevicePointer, parametersArray.length, stream);
      error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);

      CUDATools.memcpyAsync(worldToGroundTransformDevicePointer, worldToGroundTransformHostPointer, worldToGroundTransformArray.length, stream);
      CUDATools.memcpyAsync(sensorToGroundTransformDevicePointer, sensorToGroundTransformHostPointer, sensorToGroundTransformArray.length, stream);
      CUDATools.memcpyAsync(parametersDevicePointer, parametersHostPointer, parametersArray.length, stream);

      error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);

      blockSize = new dim3(BLOCK_SIZE_XY, BLOCK_SIZE_XY, 1);
      int registerKernelGridSizeXY = (globalCellsPerAxis + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
      planOffsetKernelGridDim = new dim3(registerKernelGridSizeXY, registerKernelGridSizeXY, 1);
      registerKernelGridDim = new dim3(registerKernelGridSizeXY, registerKernelGridSizeXY, 1);

      // Need to reset the empty global map before using it so when its filled it starts with all "zero" values
      emptyGlobalHeightMapImage.setTo(new Scalar(resetOffset));

      emptyRegisterKernel.withPointer(localHeightMapImage.data()).withLong(localHeightMapImage.step());
      emptyRegisterKernel.withPointer(emptyGlobalHeightMapImage.data()).withLong(emptyGlobalHeightMapImage.step());
      emptyRegisterKernel.withPointer(parametersDevicePointer);
      emptyRegisterKernel.withPointer(worldToGroundTransformDevicePointer);
      emptyRegisterKernel.withPointer(sensorToGroundTransformDevicePointer);

      emptyRegisterKernel.run(stream, registerKernelGridDim, blockSize, 0);
      error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);

      // Run the plan offset kernel
      planOffsetKernel.withPointer(globalHeightMapImage.data()).withLong(globalHeightMapImage.step());
      planOffsetKernel.withPointer(emptyGlobalHeightMapImage.data()).withLong(emptyGlobalHeightMapImage.step());
      planOffsetKernel.withFloat(z).withInt(globalHeightMapImage.rows()).withInt(globalHeightMapImage.cols());
      planOffsetKernel.withFloat(resetOffset);

      planOffsetKernel.run(stream, planOffsetKernelGridDim, blockSize, 0);
      error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);
   }

   public float[] populateParameterArray(HeightMapParameters parameters, CameraIntrinsics cameraIntrinsics, Tuple3DReadOnly gridCenter)
   {
      return new float[] {(float) parameters.getLocalCellSizeInMeters(),
                          (float) centerIndex,
                          (float) cameraIntrinsics.getHeight(),
                          (float) cameraIntrinsics.getWidth(),
                          (float) gridCenter.getX(),
                          (float) gridCenter.getY(),
                          (float) mode,
                          (float) cameraIntrinsics.getCx(),
                          (float) cameraIntrinsics.getCy(),
                          (float) cameraIntrinsics.getFx(),
                          (float) cameraIntrinsics.getFy(),
                          (float) parameters.getGlobalCellSizeInMeters(),
                          (float) globalCenterIndex,
                          (float) parameters.getRobotCollisionCylinderRadius(),
                          gridOffsetX,
                          (float) parameters.getHeightFilterAlpha(),
                          (float) localCellsPerAxis,
                          (float) globalCellsPerAxis,
                          (float) parameters.getHeightScaleFactor(),
                          (float) parameters.getMinHeightRegistration(),
                          (float) parameters.getMaxHeightRegistration(),
                          (float) parameters.getMinHeightDifference(),
                          (float) parameters.getMaxHeightDifference(),
                          (float) parameters.getSearchWindowHeight(),
                          (float) parameters.getSearchWindowWidth(),
                          (float) cropCenterIndex,
                          (float) parameters.getMinClampHeight(),
                          (float) parameters.getMaxClampHeight(),
                          (float) parameters.getHeightOffset(),
                          (float) parameters.getSteppingCosineThreshold(),
                          (float) parameters.getSteppingContactThreshold(),
                          (float) parameters.getContactWindowSize(),
                          (float) parameters.getSpatialAlpha(),
                          (float) parameters.getSearchSkipSize(),
                          (float) parameters.getVerticalSearchSize(),
                          (float) parameters.getVerticalSearchResolution(),
                          (float) parameters.getFastSearchSize()};
   }

   public void destroy()
   {
      heightMapCUDAProgram.close();
      updateKernel.close();
      registerKernel.close();
      croppingKernel.close();

      emptyGlobalHeightMapImage.close();
      planOffsetKernelGridDim.close();
      planOffsetKernel.close();

      // Clean up each resource
      deallocateFloatPointer(groundToSensorTransformHostPointer, groundToSensorTransformDevicePointer);
      deallocateFloatPointer(sensorToGroundTransformHostPointer, sensorToGroundTransformDevicePointer);
      deallocateFloatPointer(worldToGroundTransformHostPointer, worldToGroundTransformDevicePointer);
      deallocateFloatPointer(parametersHostPointer, parametersDevicePointer);

      blockSize.close();
      updateKernelGridDim.close();
      registerKernelGridDim.close();
      croppingKernelGridDim.close();

      inputDepthImage.close();
      localHeightMapImage.close();
      globalHeightMapImage.close();
      terrainCostImage.close();
      contactMapImage.close();
      sensorCroppedHeightMapImage.close();

      snappedFootstepsExtractor.destroy();
      filteredRapidHeightMapExtractor.destroy();

      // At the end we have to destroy the stream to release the memory
      CUDAStreamManager.releaseStream(stream);
   }

   private void deallocateFloatPointer(FloatPointer hostPointer, Pointer devicePointer)
   {
      if (hostPointer != null)
      {
         hostPointer.close();
         System.out.println("Deallocated host pointer.");
      }

      if (devicePointer != null)
      {
         devicePointer.close();
         cudaFree(devicePointer);
         System.out.println("Deallocated device pointer.");
      }
   }

   public int getSequenceNumber()
   {
      return sequenceNumber;
   }

   public HeightMapData getHeightMapData()
   {
      HeightMapData latestHeightMapData = new HeightMapData((float) heightMapParameters.getGlobalCellSizeInMeters(),
                                                            (float) heightMapParameters.getGlobalWidthInMeters(),
                                                            getSensorOrigin().getX(),
                                                            getSensorOrigin().getY());

      Mat heightMapMat = getTerrainMapData().getHeightMap();
      PerceptionMessageTools.convertToHeightMapData(heightMapMat,
                                                    latestHeightMapData,
                                                    getSensorOrigin(),
                                                    (float) heightMapParameters.getGlobalWidthInMeters(),
                                                    (float) heightMapParameters.getGlobalCellSizeInMeters());

      return latestHeightMapData;
   }

   public TerrainMapData getTerrainMapData()
   {
      return terrainMapData;
   }

   public Point3D getSensorOrigin()
   {
      return sensorOrigin;
   }

   public int getCenterIndex()
   {
      return centerIndex;
   }
}
