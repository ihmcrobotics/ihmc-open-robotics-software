package us.ihmc.perception.gpuHeightMap;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.cuda.CUDAKernel;
import us.ihmc.perception.cuda.CUDAProgram;
import us.ihmc.perception.cuda.CUDAStreamManager;
import us.ihmc.perception.cuda.CUDATools;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;

import java.net.URL;

import static org.bytedeco.cuda.global.cudart.*;

public class RapidHeightMapExtractor
{
   private static final boolean PRINT_TIMING_FOR_KERNELS = false;
   static final int BLOCK_SIZE_XY = 32;
   private static final int MODE = 1; // 0 -> Ouster, 1 -> Realsense

   private final HeightMapParameters heightMapParameters;
   private final CUstream_st stream;
   private final CUDAProgram heightMapProgram;
   private final dim3 blockSize;

   // These are the mats required to extract the depth data
   private final GpuMat localMeanMap;
   private final GpuMat localVarianceMap;
   private final GpuMat localMotionVarianceMap;
   private final GpuMat localSampleCountMap;

   // These are the mats required to keep a global map
   private final GpuMat globalMeanMap;
   private final GpuMat globalVarianceMap;
   private final GpuMat previousGlobalMeanMap;
   private final GpuMat previousGlobalVarianceMap;
   private final GpuMat terrainCroppedHeightMap;
   private final GpuMat emptyGlobalHeightMap;

   private final CUDAKernel updateKernel;
   private final CUDAKernel translateKernel;
   private final CUDAKernel registerKernel;
   private final CUDAKernel terrainCroppingKernel;
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
   private int centerIndexTerrain;
   private int cellsPerAxisGlobal;
   private int cellsPerAxisTerrain;

   private final RigidBodyTransform previousSensorToWorld = new RigidBodyTransform();
   private int previousCellX;
   private int previousCellY;
   private int resetOffset;

   public RapidHeightMapExtractor(HeightMapParameters heightMapParameters)
   {
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
         terrainCroppingKernel = heightMapProgram.loadKernel("terrainCroppingHeightMapKernel");
         planOffsetKernel = heightMapProgram.loadKernel("planOffsetKernel");
         emptyRegisterKernel = heightMapProgram.loadKernel("heightMapEmptyRegistrationKernel");

         updateKernel.enableKernelTimings(PRINT_TIMING_FOR_KERNELS);
         translateKernel.enableKernelTimings(PRINT_TIMING_FOR_KERNELS);
         registerKernel.enableKernelTimings(PRINT_TIMING_FOR_KERNELS);
         terrainCroppingKernel.enableKernelTimings(PRINT_TIMING_FOR_KERNELS);
         planOffsetKernel.enableKernelTimings(PRINT_TIMING_FOR_KERNELS);
         emptyRegisterKernel.enableKernelTimings(PRINT_TIMING_FOR_KERNELS);

         // Initialize matrices and images
         localMeanMap = new GpuMat(cellsPerAxisLocal, cellsPerAxisLocal, opencv_core.CV_32FC1);
         localVarianceMap = new GpuMat(cellsPerAxisLocal, cellsPerAxisLocal, opencv_core.CV_32FC1);
         localMotionVarianceMap = new GpuMat(cellsPerAxisLocal, cellsPerAxisLocal, opencv_core.CV_32FC1);
         localSampleCountMap = new GpuMat(cellsPerAxisLocal, cellsPerAxisLocal, opencv_core.CV_16UC1);

         globalMeanMap = new GpuMat(cellsPerAxisGlobal, cellsPerAxisGlobal, opencv_core.CV_32FC1);
         globalVarianceMap = new GpuMat(cellsPerAxisGlobal, cellsPerAxisGlobal, opencv_core.CV_32FC1);
         previousGlobalMeanMap = new GpuMat(cellsPerAxisGlobal, cellsPerAxisGlobal, opencv_core.CV_32FC1);
         previousGlobalVarianceMap = new GpuMat(cellsPerAxisGlobal, cellsPerAxisGlobal, opencv_core.CV_32FC1);
         terrainCroppedHeightMap = new GpuMat(cellsPerAxisTerrain, cellsPerAxisTerrain, opencv_core.CV_32FC1);
         emptyGlobalHeightMap = new GpuMat(cellsPerAxisGlobal, cellsPerAxisGlobal, opencv_core.CV_16UC1);

         // Initialize transformation pointers
         groundToSensorTransformHostPointer = new FloatPointer(16);
         groundToSensorTransformDevicePointer = new FloatPointer();

         sensorToGroundTransformHostPointer = new FloatPointer(16);
         sensorToGroundTransformDevicePointer = new FloatPointer();

         zUpCameraToWorldAlignedGroundHostPointer = new FloatPointer(16);
         ZUpCameraToWorldAlignedGroundDevicePointer = new FloatPointer();

         parametersHostPointer = new FloatPointer(27);
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

      centerIndexGlobal = HeightMapTools.computeCenterIndex(heightMapParameters.getGlobalWidthInMeters(), heightMapParameters.getCellSizeInMeters());
      cellsPerAxisGlobal = 2 * centerIndexGlobal + 1;

      centerIndexTerrain = HeightMapTools.computeCenterIndex(heightMapParameters.getTerrainWidthInMeters(), heightMapParameters.getCellSizeInMeters());
      cellsPerAxisTerrain = 2 * centerIndexTerrain + 1;
   }

   public void reset(double footHeight)
   {
      reset(footHeight, 0);
   }

   public void reset(double footHeight, int loweredValue)
   {
      resetOffset = (int) ((footHeight + heightMapParameters.getHeightOffset()) * heightMapParameters.getHeightScaleFactor());
      resetOffset -= loweredValue;

      globalMeanMap.setTo(new Scalar(resetOffset));
      emptyGlobalHeightMap.setTo(new Scalar(resetOffset));
   }

   public void update(GpuMat latestDepthImageGPU,
                      CameraIntrinsics cameraIntrinsics,
                      RigidBodyTransform sensorToWorldTransform,
                      RigidBodyTransform sensorToGroundTransform,
                      RigidBodyTransformReadOnly groundToWorldTransform,
                      Point3D sensorOrigin,
                      double footHeight)
   {
      int error;

      // Populate parameter buffers with the necessary values
      float[] parametersArray = populateParameterArray(heightMapParameters, cameraIntrinsics, footHeight);
      parametersHostPointer.put(parametersArray);
      CUDATools.mallocAsync(parametersDevicePointer, parametersArray.length, stream);
      CUDATools.memcpyAsync(parametersDevicePointer, parametersHostPointer, parametersArray.length, stream);

      RigidBodyTransform groundToWorldNoRotation = new RigidBodyTransform(groundToWorldTransform);
      // Remove rotation from transformation
      groundToWorldNoRotation.getRotation().setIdentity();

      RigidBodyTransform worldToGroundNoRotation = new RigidBodyTransform(groundToWorldNoRotation);
      // Invert translation-only transform
      worldToGroundNoRotation.invert();

      // This transformation only has rotation
      RigidBodyTransform groundToWorldAlignedGround = new RigidBodyTransform(worldToGroundNoRotation);
      groundToWorldAlignedGround.multiply(groundToWorldTransform);

      groundToWorldAlignedGround.get(worldToGroundTransformArray);
      zUpCameraToWorldAlignedGroundHostPointer.put(worldToGroundTransformArray);
      CUDATools.mallocAsync(ZUpCameraToWorldAlignedGroundDevicePointer, worldToGroundTransformArray.length, stream);
      CUDATools.memcpyAsync(ZUpCameraToWorldAlignedGroundDevicePointer, zUpCameraToWorldAlignedGroundHostPointer, worldToGroundTransformArray.length, stream);
      checkCUDAError();

      // --------- Run the update kernel ---------
      {
         // Compute "speed" of the camera
         RigidBodyTransform previousToCurrentSensorOrigin = new RigidBodyTransform(previousSensorToWorld);
         previousToCurrentSensorOrigin.invert();
         previousToCurrentSensorOrigin.multiply(sensorToWorldTransform);

         Vector3DBasics translation = previousToCurrentSensorOrigin.getTranslation();
         double linearMotionMagnitude = translation.norm();

         RotationMatrixBasics rotation = previousToCurrentSensorOrigin.getRotation();
         AxisAngle axisAngle = new AxisAngle(rotation);
         double angularMotionMagnitude = Math.abs(axisAngle.getAngle());

         // Compute the inverse transforms for later use
         RigidBodyTransform groundToSensorTransform = new RigidBodyTransform(sensorToGroundTransform);
         groundToSensorTransform.invert();

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
         updateKernel.withPointer(globalMeanMap.data()).withLong(globalMeanMap.step());
         updateKernel.withPointer(localMeanMap.data()).withLong(localMeanMap.step());
         updateKernel.withPointer(localVarianceMap.data()).withLong(localVarianceMap.step());
         updateKernel.withPointer(localMotionVarianceMap.data()).withLong(localMotionVarianceMap.step());
         updateKernel.withPointer(localSampleCountMap.data()).withLong(localSampleCountMap.step());
         updateKernel.withPointer(parametersDevicePointer);
         updateKernel.withPointer(sensorToGroundTransformDevicePointer);
         updateKernel.withPointer(groundToSensorTransformDevicePointer);
         updateKernel.withPointer(ZUpCameraToWorldAlignedGroundDevicePointer);
         updateKernel.withFloat((float) linearMotionMagnitude).withFloat((float) angularMotionMagnitude);
         updateKernel.withFloat(resetOffset);

         updateKernel.run(stream, updateKernelGridDim, blockSize, 0);

         updateKernelGridDim.close();
         cudaFreeAsync(sensorToGroundTransformDevicePointer, stream);
         cudaFreeAsync(groundToSensorTransformDevicePointer, stream);
         checkCUDAError();
      }

      // ---------- Run the translate kernel ---------
      {
         int currentCellX = (int) Math.round(sensorOrigin.getX32() / heightMapParameters.getCellSizeInMeters());
         int currentCellY = (int) Math.round(sensorOrigin.getY32() / heightMapParameters.getCellSizeInMeters());

         // This means we have moved more then 2cm. So each cell should shift to one of its neighboring cells
         if (currentCellX != previousCellX || currentCellY != previousCellY)
         {
            // We will be updating the global height map with the applied translation of the data
            globalMeanMap.copyTo(previousGlobalMeanMap);
            globalVarianceMap.copyTo(previousGlobalVarianceMap);

            int translateKernelGridSizeXY = (cellsPerAxisGlobal + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
            dim3 translateKernelGridDim = new dim3(translateKernelGridSizeXY, translateKernelGridSizeXY, 1);

            int shiftX = currentCellX - previousCellX;
            int shiftY = currentCellY - previousCellY;

            translateKernel.withPointer(previousGlobalMeanMap.data()).withLong(previousGlobalMeanMap.step());
            translateKernel.withPointer(previousGlobalVarianceMap.data()).withLong(previousGlobalVarianceMap.step());
            translateKernel.withPointer(globalMeanMap.data()).withLong(globalMeanMap.step());
            translateKernel.withPointer(globalVarianceMap.data()).withLong(globalVarianceMap.step());
            translateKernel.withInt(shiftX).withInt(shiftY);
            translateKernel.withPointer(parametersDevicePointer);
            translateKernel.withInt(resetOffset);

            translateKernel.run(stream, translateKernelGridDim, blockSize, 0);

            translateKernelGridDim.close();
            checkCUDAError();

            previousCellX = currentCellX;
            previousCellY = currentCellY;
         }
      }

      // ---------- Run the registration kernel ----------
      {
         int registerKernelGridSizeXY = (cellsPerAxisGlobal + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
         dim3 registerKernelGridDim = new dim3(registerKernelGridSizeXY, registerKernelGridSizeXY, 1);

         registerKernel.withPointer(localMeanMap.data()).withLong(localMeanMap.step());
         registerKernel.withPointer(localVarianceMap.data()).withLong(localVarianceMap.step());
         registerKernel.withPointer(localMotionVarianceMap.data()).withLong(localMotionVarianceMap.step());
         registerKernel.withPointer(localSampleCountMap.data()).withLong(localSampleCountMap.step());
         registerKernel.withPointer(globalMeanMap.data()).withLong(globalMeanMap.step());
         registerKernel.withPointer(globalVarianceMap.data()).withLong(globalVarianceMap.step());
         registerKernel.withPointer(ZUpCameraToWorldAlignedGroundDevicePointer);
         registerKernel.withPointer(parametersDevicePointer);
         registerKernel.withFloat(resetOffset);

         registerKernel.run(stream, registerKernelGridDim, blockSize, 0);

         registerKernelGridDim.close();
         checkCUDAError();
      }

      Mat test = new Mat();
      globalMeanMap.download(test);
      PerceptionDebugTools.printMat("s", test, 10);

      // ---------- Run the Terrain cropping kernel ----------
      {
         int terrainKernelGridSizeXY = (cellsPerAxisTerrain + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
         dim3 terrainKernelGridDim = new dim3(terrainKernelGridSizeXY, terrainKernelGridSizeXY, 1);

         terrainCroppingKernel.withPointer(globalMeanMap.data()).withLong(globalMeanMap.step());
         terrainCroppingKernel.withPointer(terrainCroppedHeightMap.data()).withLong(terrainCroppedHeightMap.step());
         terrainCroppingKernel.withInt(centerIndexTerrain);
         terrainCroppingKernel.withPointer(parametersDevicePointer);

         terrainCroppingKernel.run(stream, terrainKernelGridDim, blockSize, 0);

         terrainKernelGridDim.close();
         checkCUDAError();
      }

      // All that memory we allocated on the GPU, need to free that up now
      cudaFreeAsync(parametersDevicePointer, stream);
      cudaFreeAsync(ZUpCameraToWorldAlignedGroundDevicePointer, stream);
      error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);

      // Save sensorOrigin as previous for next update
      previousSensorToWorld.set(sensorToWorldTransform);
   }

   public void updateHeightOffset(RigidBodyTransform groundToWorldTransform, float z, CameraIntrinsics cameraIntrinsics, double footHeight)
   {
      int error;

      // Populate parameter buffers with the necessary values
      float[] parametersArray = populateParameterArray(heightMapParameters, cameraIntrinsics, footHeight);
      parametersHostPointer.put(parametersArray);
      CUDATools.mallocAsync(parametersDevicePointer, parametersArray.length, stream);
      CUDATools.memcpyAsync(parametersDevicePointer, parametersHostPointer, parametersArray.length, stream);

      // ---------- Run the registration kernel for an empty global height map ----------
      {
         RigidBodyTransform groundToWorldNoRotation = new RigidBodyTransform(groundToWorldTransform);
         // Remove rotation from transformation
         groundToWorldNoRotation.getRotation().setIdentity();

         RigidBodyTransform worldToGroundNoRotation = new RigidBodyTransform(groundToWorldNoRotation);
         // Invert translation-only transform
         worldToGroundNoRotation.invert();

         // This transformation only has rotation
         RigidBodyTransform groundToWorldAlignedGround = new RigidBodyTransform(worldToGroundNoRotation);
         groundToWorldAlignedGround.multiply(groundToWorldTransform);

         groundToWorldAlignedGround.get(worldToGroundTransformArray);
         zUpCameraToWorldAlignedGroundHostPointer.put(worldToGroundTransformArray);
         CUDATools.mallocAsync(ZUpCameraToWorldAlignedGroundDevicePointer, worldToGroundTransformArray.length, stream);
         CUDATools.memcpyAsync(ZUpCameraToWorldAlignedGroundDevicePointer,
                               zUpCameraToWorldAlignedGroundHostPointer,
                               worldToGroundTransformArray.length,
                               stream);
         checkCUDAError();

         int emptyRegistrationGridSizeXY = (cellsPerAxisGlobal + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
         dim3 registerKernelGridDim = new dim3(emptyRegistrationGridSizeXY, emptyRegistrationGridSizeXY, 1);

         // Need to reset the empty global map before using it so when its filled it starts with all "zero" values
         emptyGlobalHeightMap.setTo(new Scalar(resetOffset));

         emptyRegisterKernel.withPointer(localMeanMap.data()).withLong(localMeanMap.step());
         emptyRegisterKernel.withPointer(emptyGlobalHeightMap.data()).withLong(emptyGlobalHeightMap.step());
         emptyRegisterKernel.withPointer(ZUpCameraToWorldAlignedGroundDevicePointer);
         emptyRegisterKernel.withPointer(parametersDevicePointer);
         emptyRegisterKernel.withFloat(resetOffset);

         emptyRegisterKernel.run(stream, registerKernelGridDim, blockSize, 0);

         registerKernelGridDim.close();
         checkCUDAError();
      }

      // ---------- Run the plan offset kernel ----------
      {
         int planOffsetGridSizeXY = (cellsPerAxisGlobal + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
         dim3 planOffsetKernelGridDim = new dim3(planOffsetGridSizeXY, planOffsetGridSizeXY, 1);

         // Run the plan offset kernel
         planOffsetKernel.withPointer(globalMeanMap.data()).withLong(globalMeanMap.step());
         planOffsetKernel.withPointer(emptyGlobalHeightMap.data()).withLong(emptyGlobalHeightMap.step());
         planOffsetKernel.withFloat(z).withInt(globalMeanMap.rows()).withInt(globalMeanMap.cols());
         planOffsetKernel.withFloat(resetOffset);

         planOffsetKernel.run(stream, planOffsetKernelGridDim, blockSize, 0);

         planOffsetKernelGridDim.close();
         checkCUDAError();
      }

      // Synchronize the stream so the cpu has the data when this method returns
      error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);
   }

   public float[] populateParameterArray(HeightMapParameters parameters, CameraIntrinsics cameraIntrinsics, double groundHeightGuess)
   {
      return new float[] {(float) parameters.getCellSizeInMeters(),
                          (float) centerIndexLocal,
                          (float) cameraIntrinsics.getHeight(),
                          (float) cameraIntrinsics.getWidth(),
                          (float) MODE,
                          (float) cameraIntrinsics.getCx(),
                          (float) cameraIntrinsics.getCy(),
                          (float) cameraIntrinsics.getFx(),
                          (float) cameraIntrinsics.getFy(),
                          (float) centerIndexGlobal,
                          (float) heightMapParameters.getLocalWidthInMeters() / 2,
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
                          (float) parameters.getKalmanFilterPredictionNoise(),
                          (float) parameters.getAdditionalTranslationalVarianceAdded(),
                          (float) parameters.getVariancePerMeter(),
                          (float) parameters.getVariancePerTranslationSpeed(),
                          (float) parameters.getVariancePerRotationSpeed(),
                          (float) parameters.getSearchSkipSize(),
                          (float) groundHeightGuess};
   }

   public void destroy()
   {
      heightMapProgram.close();
      blockSize.close();

      updateKernel.close();
      translateKernel.close();
      registerKernel.close();
      planOffsetKernel.close();
      emptyRegisterKernel.close();

      // Clean up each resource
      deallocateFloatPointer(groundToSensorTransformHostPointer, groundToSensorTransformDevicePointer, stream);
      deallocateFloatPointer(sensorToGroundTransformHostPointer, sensorToGroundTransformDevicePointer, stream);
      deallocateFloatPointer(zUpCameraToWorldAlignedGroundHostPointer, ZUpCameraToWorldAlignedGroundDevicePointer, stream);
      deallocateFloatPointer(parametersHostPointer, parametersDevicePointer, stream);

      localMeanMap.close();
      localVarianceMap.close();
      localMotionVarianceMap.close();
      localSampleCountMap.close();

      globalMeanMap.close();
      globalVarianceMap.close();
      previousGlobalMeanMap.close();
      previousGlobalVarianceMap.close();
      terrainCroppedHeightMap.close();
      emptyGlobalHeightMap.close();

      // At the end we have to destroy the stream to release the memory
      CUDAStreamManager.releaseStream(stream);
   }

   /**
    * If we are debugging the kernels with {@link RapidHeightMapExtractor#PRINT_TIMING_FOR_KERNELS} then we want to synchronize the GPU
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

   private void deallocateFloatPointer(FloatPointer hostPointer, Pointer devicePointer, CUstream_st stream)
   {
      hostPointer.close();
      devicePointer.close();
      cudaFreeAsync(devicePointer, stream);
   }

   public GpuMat getHeightMap()
   {
      return globalMeanMap.clone();
   }

   public GpuMat getTerrainHeightMap()
   {
      return terrainCroppedHeightMap.clone();
   }
}
