package us.ihmc.perception.gpuHeightMap;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Scalar;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.cuda.CUDAKernel;
import us.ihmc.perception.cuda.CUDAProgram;
import us.ihmc.perception.cuda.CUDAStreamManager;
import us.ihmc.perception.cuda.CUDATools;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;

import java.net.URL;

import static org.bytedeco.cuda.global.cudart.*;

public class RapidHeightMapExtractorCUDA
{
   private static final boolean PRINT_TIMING_FOR_KERNELS = false;
   static final int BLOCK_SIZE_XY = 32;

   private final int mode; // 0 -> Ouster, 1 -> Realsense
   private final HeightMapParameters heightMapParameters;

   private final GpuMat localHeightMapImage;
   private final GpuMat globalHeightMapImage;
   private final GpuMat oldGlobalHeightMapImage;
   private final GpuMat terrainCostImage;
   private final GpuMat contactMapImage;
   private final GpuMat croppedHeightMapImage;
   private final GpuMat emptyGlobalHeightMapImage;
   private final CUDAProgram heightMapCUDAProgram;

   private final CUstream_st stream;
   private final CUDAKernel updateKernel;
   private final CUDAKernel registerKernel;
   private final CUDAKernel shiftHeightMap;
   private final CUDAKernel croppingKernel;
   private final CUDAKernel planOffsetKernel;
   private final CUDAKernel emptyRegisterKernel;

   private final float[] worldToGroundTransformArray = new float[16];
   private final float[] groundToSensorTransformArray = new float[16];
   private final float[] sensorToGroundTransformArray = new float[16];
   private final float[] previousToCurrentSensorTransformArray = new float[16];

   private final FloatPointer groundToSensorTransformHostPointer;
   private final FloatPointer groundToSensorTransformDevicePointer;
   private final FloatPointer sensorToGroundTransformHostPointer;
   private final FloatPointer sensorToGroundTransformDevicePointer;
   private final FloatPointer worldToGroundTransformHostPointer;
   private final FloatPointer worldToGroundTransformDevicePointer;
   private final FloatPointer previousToCurrentSensorHostPointer;
   private final FloatPointer previousToCurrentSensorDevicePointer;
   private final FloatPointer parametersHostPointer;
   private final FloatPointer parametersDevicePointer;
   private final FilteredRapidHeightMapExtractor filteredRapidHeightMapExtractor;
   private final FilteredVerticalSurfacesExtractor verticalSurfacesExtractor;
   public int sequenceNumber = 0;
   private float gridOffsetX;
   private int centerIndex;
   private int cellsPerAxisLocal;
   private int globalCenterIndex;
   private int croppedCenterIndex;
   private int cellsPerAxisCropped;
   private int globalCellsPerAxis;
   private dim3 blockSize;
   private dim3 updateKernelGridDim;
   private dim3 registerKernelGridDim;
   private dim3 croppingKernelGridDim;
   private dim3 planOffsetKernelGridDim;
   private int resetOffset;

   private final RigidBodyTransform currentGroundToWorldTransform = new RigidBodyTransform();
   private final RigidBodyTransform previousGroundToWorldTransform = new RigidBodyTransform();
   private final RigidBodyTransform previousToCurrentSensorTransform = new RigidBodyTransform();

   public RapidHeightMapExtractorCUDA(int mode, HeightMapParameters heightMapParameters)
   {
      this.mode = mode;
      this.heightMapParameters = heightMapParameters;

      stream = CUDAStreamManager.getStream();

      // Load header and main file
      URL heightMapUtilsHeaderPath = getClass().getResource("HeightMapUtils.cuh");
      URL mathUtilsHeaderPath = getClass().getResource("/us/ihmc/perception/cuda/MathUtils.cuh");
      URL kernelPath = getClass().getResource("RapidHeightMapExtractor.cu");

      recomputeDerivedParameters();

      // Need to initialize this after the parameters have been computed to get the right size
      filteredRapidHeightMapExtractor = new FilteredRapidHeightMapExtractor(stream, globalCellsPerAxis, globalCellsPerAxis, 6);
      verticalSurfacesExtractor = new FilteredVerticalSurfacesExtractor(stream, globalCellsPerAxis, globalCellsPerAxis);

      try
      {
         heightMapCUDAProgram = new CUDAProgram(kernelPath, heightMapUtilsHeaderPath, mathUtilsHeaderPath);

         updateKernel = heightMapCUDAProgram.loadKernel("heightMapUpdateKernel");
         registerKernel = heightMapCUDAProgram.loadKernel("heightMapRegistrationKernel");
         shiftHeightMap = heightMapCUDAProgram.loadKernel("shiftGlobalMapKernel");
         croppingKernel = heightMapCUDAProgram.loadKernel("croppingKernel");
         planOffsetKernel = heightMapCUDAProgram.loadKernel("planOffsetKernel");
         emptyRegisterKernel = heightMapCUDAProgram.loadKernel("heightMapRegistrationKernel");

         updateKernel.enableKernelTimings(PRINT_TIMING_FOR_KERNELS);
         registerKernel.enableKernelTimings(PRINT_TIMING_FOR_KERNELS);
         shiftHeightMap.enableKernelTimings(PRINT_TIMING_FOR_KERNELS);
         croppingKernel.enableKernelTimings(PRINT_TIMING_FOR_KERNELS);
         planOffsetKernel.enableKernelTimings(PRINT_TIMING_FOR_KERNELS);
         emptyRegisterKernel.enableKernelTimings(PRINT_TIMING_FOR_KERNELS);

         // Initialize matrices and images
         localHeightMapImage = new GpuMat(cellsPerAxisLocal, cellsPerAxisLocal, opencv_core.CV_16UC1);
         globalHeightMapImage = new GpuMat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_16UC1);
         oldGlobalHeightMapImage = new GpuMat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_16UC1);
         terrainCostImage = new GpuMat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_8UC1);
         contactMapImage = new GpuMat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_8UC1);
         croppedHeightMapImage = new GpuMat(cellsPerAxisCropped, cellsPerAxisCropped, opencv_core.CV_16UC1);

         emptyGlobalHeightMapImage = new GpuMat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_16UC1);

         // Initialize transformation pointers
         groundToSensorTransformHostPointer = new FloatPointer(16);
         groundToSensorTransformDevicePointer = new FloatPointer();

         sensorToGroundTransformHostPointer = new FloatPointer(16);
         sensorToGroundTransformDevicePointer = new FloatPointer();

         worldToGroundTransformHostPointer = new FloatPointer(16);
         worldToGroundTransformDevicePointer = new FloatPointer();

         previousToCurrentSensorHostPointer = new FloatPointer(16);
         previousToCurrentSensorDevicePointer = new FloatPointer();

         parametersHostPointer = new FloatPointer(37);
         parametersDevicePointer = new FloatPointer();
      }
      catch (Exception e)
      {
         throw new RuntimeException(e);
      }
   }

   private void recomputeDerivedParameters()
   {
      centerIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getLocalWidthInMeters(), heightMapParameters.getCellSizeInMeters());
      cellsPerAxisLocal = 2 * centerIndex + 1;

      globalCenterIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getInternalGlobalWidthInMeters(), heightMapParameters.getCellSizeInMeters());
      globalCellsPerAxis = 2 * globalCenterIndex + 1;

      croppedCenterIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getCroppedWidthInMeters(), heightMapParameters.getCellSizeInMeters());
      cellsPerAxisCropped = 2 * croppedCenterIndex + 1;

      gridOffsetX = (float) heightMapParameters.getLocalWidthInMeters() / 2.0f;
   }

   public void reset(double footHeight)
   {
      reset(footHeight, 0);
   }

   public void reset(double footHeight, int loweredValue)
   {
      resetOffset = (int) ((footHeight + heightMapParameters.getHeightOffset()) * heightMapParameters.getHeightScaleFactor());
      resetOffset -= loweredValue;

      localHeightMapImage.setTo(new Scalar(resetOffset));
      globalHeightMapImage.setTo(new Scalar(resetOffset));
      oldGlobalHeightMapImage.setTo(new Scalar(resetOffset));
      emptyGlobalHeightMapImage.setTo(new Scalar(resetOffset));

      filteredRapidHeightMapExtractor.reset();
      sequenceNumber = 0;
   }

   public void update(GpuMat latestDepthImageGPU,
                      CameraIntrinsics cameraIntrinsics,
                      RigidBodyTransformReadOnly sensorToWorldTransform,
                      RigidBodyTransform sensorToGroundTransform,
                      RigidBodyTransformReadOnly groundToWorldTransform,
                      Point3D sensorOrigin,
                      double footHeight)
   {
      int error;

      // Update the Z translation of the sensor to match the world transform (to handle the sensor's vertical position)
      sensorToGroundTransform.getTranslation().setZ(sensorToWorldTransform.getTranslationZ());

      // Compute the inverse transforms for later use
      RigidBodyTransform groundToSensorTransform = new RigidBodyTransform(sensorToGroundTransform);
      groundToSensorTransform.invert();
      RigidBodyTransform worldToGroundTransform = new RigidBodyTransform(groundToWorldTransform);
      worldToGroundTransform.invert();

      // Populate parameter buffers with the necessary values
      float[] parametersArray = populateParameterArray(heightMapParameters, cameraIntrinsics, sensorOrigin, footHeight);
      parametersHostPointer.put(parametersArray);

      //Extract the transform arrays for memory transfer
      groundToSensorTransform.get(groundToSensorTransformArray);
      sensorToGroundTransform.get(sensorToGroundTransformArray);
      worldToGroundTransform.get(worldToGroundTransformArray);

      //Transfer the transform arrays to the host memory
      groundToSensorTransformHostPointer.put(groundToSensorTransformArray);
      sensorToGroundTransformHostPointer.put(sensorToGroundTransformArray);
      worldToGroundTransformHostPointer.put(worldToGroundTransformArray);

      // Allocate memory on the GPU for each of the transforms and images
      // This step involves allocating CUDA memory asynchronously, and it's important to check for allocation errors
      CUDATools.mallocAsync(groundToSensorTransformDevicePointer, groundToSensorTransformArray.length, stream);
      CUDATools.mallocAsync(sensorToGroundTransformDevicePointer, sensorToGroundTransformArray.length, stream);
      CUDATools.mallocAsync(worldToGroundTransformDevicePointer, worldToGroundTransformArray.length, stream);
      CUDATools.mallocAsync(parametersDevicePointer, parametersArray.length, stream);

      // Copy the data from host memory to device memory asynchronously
      // This ensures the device has the latest data available for kernel processing
      CUDATools.memcpyAsync(groundToSensorTransformDevicePointer, groundToSensorTransformHostPointer, groundToSensorTransformArray.length, stream);
      CUDATools.memcpyAsync(sensorToGroundTransformDevicePointer, sensorToGroundTransformHostPointer, sensorToGroundTransformArray.length, stream);
      CUDATools.memcpyAsync(worldToGroundTransformDevicePointer, worldToGroundTransformHostPointer, worldToGroundTransformArray.length, stream);
      CUDATools.memcpyAsync(parametersDevicePointer, parametersHostPointer, parametersArray.length, stream);

      //Execute the CUDA kernels with the provided stream
      //Each kernel performs a specific task related to the height map update, registration, and cropping
      int updateKernelGridSizeXY = (cellsPerAxisLocal + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
      int registerKernelGridSizeXY = (globalCellsPerAxis + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
      int croppingKernelGridSizeXY = (cellsPerAxisCropped + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;

      blockSize = new dim3(BLOCK_SIZE_XY, BLOCK_SIZE_XY, 1);
      updateKernelGridDim = new dim3(updateKernelGridSizeXY, updateKernelGridSizeXY, 1);
      registerKernelGridDim = new dim3(registerKernelGridSizeXY, registerKernelGridSizeXY, 1);
      croppingKernelGridDim = new dim3(croppingKernelGridSizeXY, croppingKernelGridSizeXY, 1);

      // Run the update kernel
      updateKernel.withPointer(latestDepthImageGPU.data()).withLong(latestDepthImageGPU.step());
      updateKernel.withPointer(localHeightMapImage.data()).withLong(localHeightMapImage.step());
      updateKernel.withPointer(parametersDevicePointer);
      updateKernel.withPointer(sensorToGroundTransformDevicePointer);
      updateKernel.withPointer(groundToSensorTransformDevicePointer);
      updateKernel.withInt(cellsPerAxisLocal);

      updateKernel.run(stream, updateKernelGridDim, blockSize, 0);
      error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);

      // Set the previous-to-current to the previous, inverts it, then transforms it to the current
      // This gives the transform from the previous to the current transform of the sensor
      previousToCurrentSensorTransform.set(previousGroundToWorldTransform);
      previousToCurrentSensorTransform.invert();
      currentGroundToWorldTransform.transform(previousToCurrentSensorTransform);

      // Allocate gpu memory for the transform
      previousToCurrentSensorTransform.get(previousToCurrentSensorTransformArray);
      previousToCurrentSensorHostPointer.put(previousToCurrentSensorTransformArray);
      CUDATools.mallocAsync(previousToCurrentSensorDevicePointer, previousToCurrentSensorTransformArray.length, stream);
      CUDATools.memcpyAsync(previousToCurrentSensorDevicePointer, previousToCurrentSensorHostPointer, previousToCurrentSensorTransformArray.length, stream);

      globalHeightMapImage.copyTo(oldGlobalHeightMapImage);

      shiftHeightMap.withPointer(oldGlobalHeightMapImage.data()).withLong(oldGlobalHeightMapImage.step());
      shiftHeightMap.withPointer(globalHeightMapImage.data()).withLong(globalHeightMapImage.step());
      shiftHeightMap.withPointer(previousToCurrentSensorDevicePointer);
      shiftHeightMap.withInt(globalCellsPerAxis);
      shiftHeightMap.withPointer(parametersDevicePointer);

      shiftHeightMap.run(stream, registerKernelGridDim, blockSize, 0);
      error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);

      // Set the previous to the current
      previousGroundToWorldTransform.set(currentGroundToWorldTransform);

      // Always update the current
      currentGroundToWorldTransform.set(groundToWorldTransform);

      // Run the registration kernel
      registerKernel.withPointer(localHeightMapImage.data()).withLong(localHeightMapImage.step());
      registerKernel.withPointer(globalHeightMapImage.data()).withLong(globalHeightMapImage.step());
      registerKernel.withPointer(parametersDevicePointer);

      registerKernel.run(stream, registerKernelGridDim, blockSize, 0);
      error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);

      //      if (heightMapParameters.getEnableAlphaFilter())
      //      {
      //         filteredRapidHeightMapExtractor.update(globalHeightMapImage, resetOffset);
      //      }
      //
      //      if (heightMapParameters.getEnableVerticalFilter())
      //      {
      //         verticalSurfacesExtractor.update(globalHeightMapImage);
      //      }
      globalHeightMapImage.copyTo(croppedHeightMapImage);

      // Run the cropping kernel
      //      croppingKernel.withPointer(globalHeightMapImage.data()).withLong(globalHeightMapImage.step());
      //      croppingKernel.withPointer(croppedHeightMapImage.data()).withLong(croppedHeightMapImage.step());
      //      croppingKernel.withPointer(parametersDevicePointer);
      //      croppingKernel.withInt(cellsPerAxisCropped);
      //      error = cudaStreamSynchronize(stream);
      //      CUDATools.checkCUDAError(error);
      //
      //      croppingKernel.run(stream, croppingKernelGridDim, blockSize, 0);
      //      error = cudaStreamSynchronize(stream);
      //      CUDATools.checkCUDAError(error);

      // All that memory we allocated on the GPU, need to free that up now
      cudaFreeAsync(groundToSensorTransformDevicePointer, stream);
      cudaFreeAsync(sensorToGroundTransformDevicePointer, stream);
      cudaFreeAsync(worldToGroundTransformDevicePointer, stream);
      cudaFreeAsync(parametersDevicePointer, stream);
   }

   public void updateHeightOffset(float z, CameraIntrinsics cameraIntrinsics, Point3DReadOnly sensorOrigin, double footHeight)
   {
      int error;

      // Populate parameter buffers with the necessary values
      float[] parametersArray = populateParameterArray(heightMapParameters, cameraIntrinsics, sensorOrigin, footHeight);
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

      // We reset the filtered height map for the drift to take affect, otherwise it gets overridden.
      filteredRapidHeightMapExtractor.reset();

      // Run the plan offset kernel
      planOffsetKernel.withPointer(globalHeightMapImage.data()).withLong(globalHeightMapImage.step());
      planOffsetKernel.withPointer(emptyGlobalHeightMapImage.data()).withLong(emptyGlobalHeightMapImage.step());
      planOffsetKernel.withFloat(z).withInt(globalHeightMapImage.rows()).withInt(globalHeightMapImage.cols());
      planOffsetKernel.withFloat(resetOffset);

      planOffsetKernel.run(stream, planOffsetKernelGridDim, blockSize, 0);
      error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);

      // All that memory we allocated on the GPU, need to free that up now
      cudaFreeAsync(groundToSensorTransformDevicePointer, stream);
      cudaFreeAsync(sensorToGroundTransformDevicePointer, stream);
      cudaFreeAsync(parametersDevicePointer, stream);
   }

   public float[] populateParameterArray(HeightMapParameters parameters,
                                         CameraIntrinsics cameraIntrinsics,
                                         Tuple3DReadOnly gridCenter,
                                         double groundHeightGuess)
   {
      return new float[] {(float) parameters.getCellSizeInMeters(),
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
                          (float) parameters.getCellSizeInMeters(),
                          (float) globalCenterIndex,
                          (float) parameters.getRobotCollisionCylinderRadius(),
                          gridOffsetX,
                          (float) parameters.getHeightFilterAlpha(),
                          (float) cellsPerAxisLocal,
                          (float) globalCellsPerAxis,
                          (float) parameters.getHeightScaleFactor(),
                          (float) parameters.getMinHeightRegistration(),
                          (float) parameters.getMaxHeightRegistration(),
                          (float) parameters.getMinHeightDifference(),
                          (float) parameters.getMaxHeightDifference(),
                          (float) parameters.getSearchWindowHeight(),
                          (float) parameters.getSearchWindowWidth(),
                          (float) croppedCenterIndex,
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
                          (float) parameters.getFastSearchSize(),
                          (float) groundHeightGuess};
   }

   public void destroy()
   {
      heightMapCUDAProgram.close();
      updateKernel.close();
      registerKernel.close();
      croppingKernel.close();
      planOffsetKernel.close();

      emptyGlobalHeightMapImage.close();
      if (planOffsetKernelGridDim != null)
         planOffsetKernelGridDim.close();

      // Clean up each resource
      deallocateFloatPointer(groundToSensorTransformHostPointer, groundToSensorTransformDevicePointer);
      deallocateFloatPointer(sensorToGroundTransformHostPointer, sensorToGroundTransformDevicePointer);
      deallocateFloatPointer(worldToGroundTransformHostPointer, worldToGroundTransformDevicePointer);
      deallocateFloatPointer(parametersHostPointer, parametersDevicePointer);

      blockSize.close();
      updateKernelGridDim.close();
      registerKernelGridDim.close();
      croppingKernelGridDim.close();

      localHeightMapImage.close();
      globalHeightMapImage.close();
      terrainCostImage.close();
      contactMapImage.close();
      croppedHeightMapImage.close();

      filteredRapidHeightMapExtractor.destroy();
      verticalSurfacesExtractor.destroy();

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

   public GpuMat getCroppedHeightMap()
   {
      return croppedHeightMapImage.clone();
   }
}
