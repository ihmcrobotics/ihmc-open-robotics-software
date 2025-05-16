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

   private final HeightMapParameters heightMapParameters;
   private final CUstream_st stream;
   private final CUDAProgram heightMapProgram;
   private final dim3 blockSize;
   private final int mode; // 0 -> Ouster, 1 -> Realsense

   private final GpuMat localHeightMapImage;
   private final GpuMat globalHeightMapImage;
   private final GpuMat previousGlobalHeightMapImage;
   private final GpuMat emptyGlobalHeightMapImage;

   private final CUDAKernel updateKernel;
   private final CUDAKernel translateKernel;
   private final CUDAKernel registerKernel;
   private final CUDAKernel planOffsetKernel;
   private final CUDAKernel emptyRegisterKernel;

   private final float[] worldToGroundTransformArray = new float[16];
   private final float[] groundToSensorTransformArray = new float[16];
   private final float[] sensorToGroundTransformArray = new float[16];

   private final FloatPointer groundToSensorTransformHostPointer;
   private final FloatPointer groundToSensorTransformDevicePointer;
   private final FloatPointer sensorToGroundTransformHostPointer;
   private final FloatPointer sensorToGroundTransformDevicePointer;
   private final FloatPointer zUpCameraToWorldAlignedGroundHostPointer;
   private final FloatPointer ZUpCameraToWorldAlignedGroundDevicePointer;

   private final FloatPointer parametersHostPointer;
   private final FloatPointer parametersDevicePointer;

   private int centerIndexLocal;
   private int cellsPerAxisLocal;
   private int centerIndexGlobal;
   private int cellsPerAxisGlobal;

   private final Point3D previousSensorOrigin = new Point3D();
   private int previousCellX;
   private int previousCellY;
   private int resetOffset;

   public RapidHeightMapExtractorCUDA(int mode, HeightMapParameters heightMapParameters)
   {
      this.mode = mode;
      this.heightMapParameters = heightMapParameters;

      stream = CUDAStreamManager.getStream();

      // Load header and main file
      URL heightMapUtilsHeaderPath = getClass().getResource("HeightMapUtils.cuh");
      URL mathUtilsHeaderPath = getClass().getResource("/us/ihmc/perception/cuda/MathUtils.cuh");
      URL kernelPath = getClass().getResource("RapidHeightMapExtractor.cu");

      computeDerivedParameters();
      blockSize = new dim3(BLOCK_SIZE_XY, BLOCK_SIZE_XY, 1);

      try
      {
         heightMapProgram = new CUDAProgram(kernelPath, heightMapUtilsHeaderPath, mathUtilsHeaderPath);

         updateKernel = heightMapProgram.loadKernel("heightMapUpdateKernel");
         translateKernel = heightMapProgram.loadKernel("translateHeightMapKernel");
         registerKernel = heightMapProgram.loadKernel("heightMapRegistrationKernel");
         planOffsetKernel = heightMapProgram.loadKernel("planOffsetKernel");
         emptyRegisterKernel = heightMapProgram.loadKernel("heightMapRegistrationKernel");

         updateKernel.enableKernelTimings(PRINT_TIMING_FOR_KERNELS);
         translateKernel.enableKernelTimings(PRINT_TIMING_FOR_KERNELS);
         registerKernel.enableKernelTimings(PRINT_TIMING_FOR_KERNELS);
         planOffsetKernel.enableKernelTimings(PRINT_TIMING_FOR_KERNELS);
         emptyRegisterKernel.enableKernelTimings(PRINT_TIMING_FOR_KERNELS);

         // Initialize matrices and images
         localHeightMapImage = new GpuMat(cellsPerAxisLocal, cellsPerAxisLocal, opencv_core.CV_16UC1);
         globalHeightMapImage = new GpuMat(cellsPerAxisGlobal, cellsPerAxisGlobal, opencv_core.CV_16UC1);
         previousGlobalHeightMapImage = new GpuMat(cellsPerAxisGlobal, cellsPerAxisGlobal, opencv_core.CV_16UC1);
         emptyGlobalHeightMapImage = new GpuMat(cellsPerAxisGlobal, cellsPerAxisGlobal, opencv_core.CV_16UC1);

         // Initialize transformation pointers
         groundToSensorTransformHostPointer = new FloatPointer(16);
         groundToSensorTransformDevicePointer = new FloatPointer();

         sensorToGroundTransformHostPointer = new FloatPointer(16);
         sensorToGroundTransformDevicePointer = new FloatPointer();

         zUpCameraToWorldAlignedGroundHostPointer = new FloatPointer(16);
         ZUpCameraToWorldAlignedGroundDevicePointer = new FloatPointer();

         parametersHostPointer = new FloatPointer(34);
         parametersDevicePointer = new FloatPointer();
      }
      catch (Exception e)
      {
         throw new RuntimeException(e);
      }
   }

   private void computeDerivedParameters()
   {
      centerIndexLocal = HeightMapTools.computeCenterIndex(heightMapParameters.getLocalWidthInMeters(), heightMapParameters.getCellSizeInMeters());
      cellsPerAxisLocal = 2 * centerIndexLocal + 1;

      centerIndexGlobal = HeightMapTools.computeCenterIndex(heightMapParameters.getInternalGlobalWidthInMeters(), heightMapParameters.getCellSizeInMeters());
      cellsPerAxisGlobal = 2 * centerIndexGlobal + 1;
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
      previousGlobalHeightMapImage.setTo(new Scalar(resetOffset));
      emptyGlobalHeightMapImage.setTo(new Scalar(resetOffset));
   }

   public void update(GpuMat latestDepthImageGPU,
                      CameraIntrinsics cameraIntrinsics,
                      RigidBodyTransform sensorToGroundTransform,
                      RigidBodyTransformReadOnly groundToWorldTransform,
                      Point3D sensorOrigin,
                      double footHeight)
   {
      int error;

      // Compute the inverse transforms for later use
      RigidBodyTransform groundToSensorTransform = new RigidBodyTransform(sensorToGroundTransform);
      groundToSensorTransform.invert();

      RigidBodyTransform groundToWorldNoRotation = new RigidBodyTransform(groundToWorldTransform);
      // Remove rotation from transformation
      groundToWorldNoRotation.getRotation().setIdentity();

      RigidBodyTransform worldToGroundNoRotation = new RigidBodyTransform(groundToWorldNoRotation);
      // Invert translation-only transform
      worldToGroundNoRotation.invert();

      // This transformation only has rotation
      RigidBodyTransform groundToWorldAlignedGround = new RigidBodyTransform(worldToGroundNoRotation);
      groundToWorldAlignedGround.multiply(groundToWorldTransform);

      // Populate parameter buffers with the necessary values
      float[] parametersArray = populateParameterArray(heightMapParameters, cameraIntrinsics, footHeight);
      parametersHostPointer.put(parametersArray);
      CUDATools.mallocAsync(parametersDevicePointer, parametersArray.length, stream);
      CUDATools.memcpyAsync(parametersDevicePointer, parametersHostPointer, parametersArray.length, stream);

      // --------- Run the update kernel ---------
      {
         sensorToGroundTransform.get(sensorToGroundTransformArray);
         sensorToGroundTransformHostPointer.put(sensorToGroundTransformArray);
         CUDATools.mallocAsync(sensorToGroundTransformDevicePointer, sensorToGroundTransformArray.length, stream);
         CUDATools.memcpyAsync(sensorToGroundTransformDevicePointer, sensorToGroundTransformHostPointer, sensorToGroundTransformArray.length, stream);

         groundToSensorTransform.get(groundToSensorTransformArray);
         groundToSensorTransformHostPointer.put(groundToSensorTransformArray);
         CUDATools.mallocAsync(groundToSensorTransformDevicePointer, groundToSensorTransformArray.length, stream);
         CUDATools.memcpyAsync(groundToSensorTransformDevicePointer, groundToSensorTransformHostPointer, groundToSensorTransformArray.length, stream);

         int updateKernelGridSizeXY = (cellsPerAxisLocal + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
         dim3 updateKernelGridDim = new dim3(updateKernelGridSizeXY, updateKernelGridSizeXY, 1);

         updateKernel.withPointer(latestDepthImageGPU.data()).withLong(latestDepthImageGPU.step());
         updateKernel.withPointer(localHeightMapImage.data()).withLong(localHeightMapImage.step());
         updateKernel.withPointer(parametersDevicePointer);
         updateKernel.withPointer(sensorToGroundTransformDevicePointer);
         updateKernel.withPointer(groundToSensorTransformDevicePointer);
         updateKernel.withInt(cellsPerAxisLocal);

         updateKernel.run(stream, updateKernelGridDim, blockSize, 0);

         cudaFreeAsync(sensorToGroundTransformDevicePointer, stream);
         cudaFreeAsync(groundToSensorTransformDevicePointer, stream);
         updateKernelGridDim.close();
         error = cudaStreamSynchronize(stream);
         CUDATools.checkCUDAError(error);
      }

      // ---------- Run the translate kernel ---------
      {
         int currentCellX = (int) Math.round(sensorOrigin.getX32() / heightMapParameters.getCellSizeInMeters());
         int currentCellY = (int) Math.round(sensorOrigin.getY32() / heightMapParameters.getCellSizeInMeters());

         // This means we have moved more then 2cm. So each cell should shift to one of its neighboring cells
         if (currentCellX != previousCellX || currentCellY != previousCellY)
         {
            // We will be updating the global height map with the applied translation of the data
            globalHeightMapImage.copyTo(previousGlobalHeightMapImage);

            int translateKernelGridSizeXY = (cellsPerAxisGlobal + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
            dim3 translateKernelGridDim = new dim3(translateKernelGridSizeXY, translateKernelGridSizeXY, 1);

            translateKernel.withPointer(previousGlobalHeightMapImage.data()).withLong(previousGlobalHeightMapImage.step());
            translateKernel.withPointer(globalHeightMapImage.data()).withLong(globalHeightMapImage.step());
            translateKernel.withFloat(sensorOrigin.getX32()).withFloat(sensorOrigin.getY32());
            translateKernel.withFloat(previousSensorOrigin.getX32()).withFloat(previousSensorOrigin.getY32());
            translateKernel.withInt(cellsPerAxisGlobal);
            translateKernel.withInt(resetOffset);
            translateKernel.withFloat((float) heightMapParameters.getCellSizeInMeters());

            translateKernel.run(stream, translateKernelGridDim, blockSize, 0);

            translateKernelGridDim.close();
            error = cudaStreamSynchronize(stream);
            CUDATools.checkCUDAError(error);

            previousCellX = currentCellX;
            previousCellY = currentCellY;
            previousSensorOrigin.set(sensorOrigin);
         }
      }

      // ---------- Run the registration kernel ----------
      {
         groundToWorldAlignedGround.get(worldToGroundTransformArray);
         zUpCameraToWorldAlignedGroundHostPointer.put(worldToGroundTransformArray);
         CUDATools.mallocAsync(ZUpCameraToWorldAlignedGroundDevicePointer, worldToGroundTransformArray.length, stream);
         CUDATools.memcpyAsync(ZUpCameraToWorldAlignedGroundDevicePointer,
                               zUpCameraToWorldAlignedGroundHostPointer,
                               worldToGroundTransformArray.length,
                               stream);

         int registerKernelGridSizeXY = (cellsPerAxisGlobal + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
         dim3 registerKernelGridDim = new dim3(registerKernelGridSizeXY, registerKernelGridSizeXY, 1);

         registerKernel.withPointer(localHeightMapImage.data()).withLong(localHeightMapImage.step());
         registerKernel.withPointer(globalHeightMapImage.data()).withLong(globalHeightMapImage.step());
         registerKernel.withPointer(ZUpCameraToWorldAlignedGroundDevicePointer);
         registerKernel.withPointer(parametersDevicePointer);

         registerKernel.run(stream, registerKernelGridDim, blockSize, 0);

         cudaFreeAsync(ZUpCameraToWorldAlignedGroundDevicePointer, stream);
         registerKernelGridDim.close();
         error = cudaStreamSynchronize(stream);
         CUDATools.checkCUDAError(error);
      }

      // All that memory we allocated on the GPU, need to free that up now
      cudaFreeAsync(parametersDevicePointer, stream);
   }

   public void updateHeightOffset(float z, CameraIntrinsics cameraIntrinsics, double footHeight)
   {
      int error;

      // Populate parameter buffers with the necessary values
      float[] parametersArray = populateParameterArray(heightMapParameters, cameraIntrinsics, footHeight);
      parametersHostPointer.put(parametersArray);

      CUDATools.mallocAsync(ZUpCameraToWorldAlignedGroundDevicePointer, worldToGroundTransformArray.length, stream);
      CUDATools.mallocAsync(sensorToGroundTransformDevicePointer, sensorToGroundTransformArray.length, stream);
      CUDATools.mallocAsync(parametersDevicePointer, parametersArray.length, stream);
      error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);

      CUDATools.memcpyAsync(ZUpCameraToWorldAlignedGroundDevicePointer, zUpCameraToWorldAlignedGroundHostPointer, worldToGroundTransformArray.length, stream);
      CUDATools.memcpyAsync(sensorToGroundTransformDevicePointer, sensorToGroundTransformHostPointer, sensorToGroundTransformArray.length, stream);
      CUDATools.memcpyAsync(parametersDevicePointer, parametersHostPointer, parametersArray.length, stream);

      error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);

      int registerKernelGridSizeXY = (cellsPerAxisGlobal + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
      dim3 planOffsetKernelGridDim = new dim3(registerKernelGridSizeXY, registerKernelGridSizeXY, 1);
      dim3 registerKernelGridDim = new dim3(registerKernelGridSizeXY, registerKernelGridSizeXY, 1);

      // Need to reset the empty global map before using it so when its filled it starts with all "zero" values
      emptyGlobalHeightMapImage.setTo(new Scalar(resetOffset));

      emptyRegisterKernel.withPointer(localHeightMapImage.data()).withLong(localHeightMapImage.step());
      emptyRegisterKernel.withPointer(emptyGlobalHeightMapImage.data()).withLong(emptyGlobalHeightMapImage.step());
      emptyRegisterKernel.withPointer(parametersDevicePointer);
      emptyRegisterKernel.withPointer(ZUpCameraToWorldAlignedGroundDevicePointer);
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

      // All that memory we allocated on the GPU, need to free that up now
      planOffsetKernelGridDim.close();
      registerKernelGridDim.close();
      cudaFreeAsync(sensorToGroundTransformDevicePointer, stream);
      cudaFreeAsync(parametersDevicePointer, stream);
   }

   public float[] populateParameterArray(HeightMapParameters parameters, CameraIntrinsics cameraIntrinsics, double groundHeightGuess)
   {
      return new float[] {(float) parameters.getCellSizeInMeters(),
                          (float) centerIndexLocal,
                          (float) cameraIntrinsics.getHeight(),
                          (float) cameraIntrinsics.getWidth(),
                          (float) mode,
                          (float) cameraIntrinsics.getCx(),
                          (float) cameraIntrinsics.getCy(),
                          (float) cameraIntrinsics.getFx(),
                          (float) cameraIntrinsics.getFy(),
                          (float) centerIndexGlobal,
                          (float) parameters.getRobotCollisionCylinderRadius(),
                          (float) heightMapParameters.getLocalWidthInMeters() / 2,
                          (float) parameters.getHeightFilterAlpha(),
                          (float) cellsPerAxisLocal,
                          (float) cellsPerAxisGlobal,
                          (float) parameters.getHeightScaleFactor(),
                          (float) parameters.getMinHeightRegistration(),
                          (float) parameters.getMaxHeightRegistration(),
                          (float) parameters.getMinHeightDifference(),
                          (float) parameters.getMaxHeightDifference(),
                          (float) parameters.getSearchWindowHeight(),
                          (float) parameters.getSearchWindowWidth(),
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
      heightMapProgram.close();
      updateKernel.close();
      registerKernel.close();
      planOffsetKernel.close();

      // Clean up each resource
      deallocateFloatPointer(groundToSensorTransformHostPointer, groundToSensorTransformDevicePointer);
      deallocateFloatPointer(sensorToGroundTransformHostPointer, sensorToGroundTransformDevicePointer);
      deallocateFloatPointer(zUpCameraToWorldAlignedGroundHostPointer, ZUpCameraToWorldAlignedGroundDevicePointer);
      deallocateFloatPointer(parametersHostPointer, parametersDevicePointer);

      blockSize.close();

      localHeightMapImage.close();
      globalHeightMapImage.close();
      previousGlobalHeightMapImage.close();
      emptyGlobalHeightMapImage.close();

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

   public GpuMat getHeightMap()
   {
      return globalHeightMapImage.clone();
   }
}
