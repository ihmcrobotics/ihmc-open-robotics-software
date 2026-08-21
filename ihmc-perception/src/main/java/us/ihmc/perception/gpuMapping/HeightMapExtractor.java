package us.ihmc.perception.gpuMapping;

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
import us.ihmc.sensors.CameraIntrinsics;
import us.ihmc.perception.cuda.CUDAKernel;
import us.ihmc.perception.cuda.CUDAProgram;
import us.ihmc.perception.cuda.CUDAStreamManager;
import us.ihmc.perception.cuda.CUDATools;

import java.net.URL;

import static org.bytedeco.cuda.global.cudart.*;

public class HeightMapExtractor
{
   private static final boolean PRINT_TIMING_FOR_KERNELS = false;
   /**
    * The choice of 16 here is to utilize more SMs (Multi Processors) on the GPU.
    * This was chosen based on GPU profiling and significantly effects performance.
    */
   private static final int BLOCK_SIZE_XY = 8;
   /**
    * Sentinel written to a global-map variance cell to mark it as holding no real data.
    * Must match the {@code INVALID_CELL_VARIANCE} constant in HeightMapExtractor.cu.
    */
   private static final float INVALID_CELL_VARIANCE = -1.0f;
   /** [N, sum_rqx, sum_rqy, sum_rq2, sum_rx, sum_ry, sum_rz, sum_yawRhs], see icpCorrespondenceKernel. */
   private static final int ICP_ACCUMULATOR_SIZE = 8;

   private final HeightMapParameters heightMapParameters;
   private final CUstream_st stream;
   private final CUDAProgram heightMapProgram;
   private final dim3 blockSize;

   // These are the mats required to extract the depth data
   private final GpuMat tempSumMap;
   private final GpuMat tempCountMap;
   private final GpuMat tempSumOfSquaresMap;
   private final GpuMat tempMotionVarianceMap;
   private final GpuMat localMeanMap;
   private final GpuMat localVarianceMap;
   private final GpuMat localMotionVarianceMap;

   // These are the mats required to keep a global map
   private final GpuMat globalMeanMap;
   private final GpuMat globalVarianceMap;
   private final GpuMat previousGlobalMeanMap;
   private final GpuMat previousGlobalVarianceMap;
   private final GpuMat emptyGlobalHeightMap;

   private final CUDAKernel updateTempMapsKernel;
   private final CUDAKernel localMapKernel;
   private final CUDAKernel translateKernel;
   private final CUDAKernel icpCorrespondenceKernel;
   private final CUDAKernel icpApplyCorrectionKernel;
   private final CUDAKernel registerKernel;
   private final CUDAKernel planOffsetKernel;
   private final CUDAKernel emptyRegisterKernel;

   private final FloatPointer icpAccumulatorHostPointer;
   private final FloatPointer icpAccumulatorDevicePointer;

   private final float[] groundToWorldNoRotationTransformArray = new float[16];
   private final float[] sensorToWorldAlignedGroundTransformArray = new float[16];

   private final FloatPointer sensorToWorldAlignedGroundTransformHostPointer;
   private final FloatPointer sensorToWorldAlignedGroundTransformDevicePointer;

   private final FloatPointer groundToWorldTranslationHostPointer;
   private final FloatPointer groundToWorldTranslationDevicePointer;

   private final FloatPointer parametersHostPointer;
   private final FloatPointer parametersDevicePointer;

   private final int localCenterIndex;
   private final int globalCenterIndex;
   private final int localCellsPerAxis;
   private final int globalCellsPerAxis;

   private final RigidBodyTransform previousSensorToWorld = new RigidBodyTransform();
   private int previousCellX;
   private int previousCellY;
   private float resetOffset;

   private final HeightMapData heightMapData;
   private final Point3D heightMapCenterPoint = new Point3D();

   public HeightMapExtractor(HeightMapParameters heightMapParameters)
   {
      this.heightMapParameters = heightMapParameters;
      stream = CUDAStreamManager.getStream();

      // Load header and main file
      URL heightMapUtilsHeaderPath = getClass().getResource("HeightMapUtils.cuh");
      URL mathUtilsHeaderPath = getClass().getResource("/us/ihmc/perception/cuda/MathUtils.cuh");
      URL kernelPath = getClass().getResource("HeightMapExtractor.cu");

      localCenterIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getLocalWidthInMeters(), heightMapParameters.getCellSize());
      localCellsPerAxis = 2 * localCenterIndex + 1;
      globalCenterIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getGlobalWidthInMeters(), heightMapParameters.getCellSize());
      globalCellsPerAxis = 2 * globalCenterIndex + 1;

      blockSize = new dim3(BLOCK_SIZE_XY, BLOCK_SIZE_XY, 1);

      try
      {
         heightMapProgram = new CUDAProgram(kernelPath, heightMapUtilsHeaderPath, mathUtilsHeaderPath);

         updateTempMapsKernel = heightMapProgram.loadKernel("heightMapUpdateDataKernel");
         localMapKernel = heightMapProgram.loadKernel("computeLocalMap");
         translateKernel = heightMapProgram.loadKernel("translateHeightMapKernel");
         icpCorrespondenceKernel = heightMapProgram.loadKernel("icpCorrespondenceKernel");
         icpApplyCorrectionKernel = heightMapProgram.loadKernel("icpApplyCorrectionKernel");
         registerKernel = heightMapProgram.loadKernel("heightMapRegistrationKernel");
         planOffsetKernel = heightMapProgram.loadKernel("planOffsetKernel");
         emptyRegisterKernel = heightMapProgram.loadKernel("heightMapEmptyRegistrationKernel");

         updateTempMapsKernel.enableKernelTimings(PRINT_TIMING_FOR_KERNELS);
         localMapKernel.enableKernelTimings(PRINT_TIMING_FOR_KERNELS);
         translateKernel.enableKernelTimings(PRINT_TIMING_FOR_KERNELS);
         icpCorrespondenceKernel.enableKernelTimings(PRINT_TIMING_FOR_KERNELS);
         icpApplyCorrectionKernel.enableKernelTimings(PRINT_TIMING_FOR_KERNELS);
         registerKernel.enableKernelTimings(PRINT_TIMING_FOR_KERNELS);
         planOffsetKernel.enableKernelTimings(PRINT_TIMING_FOR_KERNELS);
         emptyRegisterKernel.enableKernelTimings(PRINT_TIMING_FOR_KERNELS);

         tempSumMap = new GpuMat(localCellsPerAxis, localCellsPerAxis, opencv_core.CV_32FC1);
         tempCountMap = new GpuMat(localCellsPerAxis, localCellsPerAxis, opencv_core.CV_32SC1);
         tempSumOfSquaresMap = new GpuMat(localCellsPerAxis, localCellsPerAxis, opencv_core.CV_32FC1);
         tempMotionVarianceMap = new GpuMat(localCellsPerAxis, localCellsPerAxis, opencv_core.CV_32FC1);

         // Initialize matrices and images
         localMeanMap = new GpuMat(localCellsPerAxis, localCellsPerAxis, opencv_core.CV_32FC1);
         localVarianceMap = new GpuMat(localCellsPerAxis, localCellsPerAxis, opencv_core.CV_32FC1);
         localMotionVarianceMap = new GpuMat(localCellsPerAxis, localCellsPerAxis, opencv_core.CV_32FC1);

         globalMeanMap = new GpuMat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_32FC1);
         globalVarianceMap = new GpuMat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_32FC1);
         previousGlobalMeanMap = new GpuMat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_32FC1);
         previousGlobalVarianceMap = new GpuMat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_32FC1);
         emptyGlobalHeightMap = new GpuMat(globalCellsPerAxis, globalCellsPerAxis, opencv_core.CV_32FC1);

         // Initialize transformation pointers
         sensorToWorldAlignedGroundTransformHostPointer = new FloatPointer(16);
         sensorToWorldAlignedGroundTransformDevicePointer = new FloatPointer();

         groundToWorldTranslationHostPointer = new FloatPointer(16);
         groundToWorldTranslationDevicePointer = new FloatPointer();

         parametersHostPointer = new FloatPointer(24);
         parametersDevicePointer = new FloatPointer();

         icpAccumulatorHostPointer = new FloatPointer(ICP_ACCUMULATOR_SIZE);
         icpAccumulatorDevicePointer = new FloatPointer();
         CUDATools.mallocAsync(icpAccumulatorDevicePointer, ICP_ACCUMULATOR_SIZE, stream);
      }
      catch (Exception e)
      {
         throw new RuntimeException(e);
      }

      heightMapData = new HeightMapData(heightMapParameters.getCellSize(), heightMapParameters.getGlobalWidthInMeters(), 0.0, 0.0);
   }

   public void reset(double footHeight)
   {
      reset(footHeight, 0);
   }

   public void reset(double footHeight, float loweredValue)
   {
      resetOffset = (float) footHeight;
      resetOffset -= loweredValue;

      globalMeanMap.setTo(new Scalar(resetOffset));
      globalVarianceMap.setTo(new Scalar(INVALID_CELL_VARIANCE));
   }

   public void update(GpuMat latestDepthImageGPU,
                      CameraIntrinsics cameraIntrinsics,
                      RigidBodyTransformReadOnly sensorToWorldTransform,
                      RigidBodyTransformReadOnly sensorToGroundTransform,
                      RigidBodyTransformReadOnly groundToWorldTransform,
                      float zDriftInMeters,
                      Point3D heightMapCenter,
                      double footHeight)
   {
      int error;

      // Populate parameter buffers with the necessary values
      float[] parametersArray = populateParameterArray(heightMapParameters, cameraIntrinsics, footHeight);
      parametersHostPointer.put(parametersArray);
      CUDATools.mallocAsync(parametersDevicePointer, parametersArray.length, stream);
      CUDATools.memcpyAsync(parametersDevicePointer, parametersHostPointer, parametersArray.length, stream);

      // Step 1: Remove rotation from transformation
      RigidBodyTransform groundToWorldNoRotation = new RigidBodyTransform(groundToWorldTransform);
      groundToWorldNoRotation.getRotation().setIdentity();

      // Step 2: Invert translation-only transform
      RigidBodyTransform worldToGroundNoRotation = new RigidBodyTransform(groundToWorldNoRotation);
      worldToGroundNoRotation.invert();

      // Step 3: Multiply with full ground->world to keep rotation, giving aligned ground
      RigidBodyTransform sensorToWorldAlignedGround = new RigidBodyTransform(worldToGroundNoRotation);
      sensorToWorldAlignedGround.multiply(groundToWorldTransform);

      // Step 4: Apply sensor->ground transform
      sensorToWorldAlignedGround.multiply(sensorToGroundTransform);

      groundToWorldNoRotation.get(groundToWorldNoRotationTransformArray);
      groundToWorldTranslationHostPointer.put(groundToWorldNoRotationTransformArray);
      CUDATools.mallocAsync(groundToWorldTranslationDevicePointer, groundToWorldNoRotationTransformArray.length, stream);
      CUDATools.memcpyAsync(groundToWorldTranslationDevicePointer, groundToWorldTranslationHostPointer, groundToWorldNoRotationTransformArray.length, stream);      checkCUDAError();
      checkCUDAError();

      // ---------- Run the translate kernel ---------
      // This is the first thing we need to do, we are going to compare the newest local data to the global data.
      // It won't line up if we don't first translate the global map to the latest translation
      {
         int currentCellX = (int) Math.round(heightMapCenter.getX32() / heightMapParameters.getCellSize());
         int currentCellY = (int) Math.round(heightMapCenter.getY32() / heightMapParameters.getCellSize());

         // This means we have moved more than 2cm. So each cell should shift to one of its neighboring cells
         if (currentCellX != previousCellX || currentCellY != previousCellY)
         {
            // We will be updating the global height map with the applied translation of the data
            globalMeanMap.copyTo(previousGlobalMeanMap);
            globalVarianceMap.copyTo(previousGlobalVarianceMap);

            int translateKernelGridSizeXY = (globalCellsPerAxis + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
            dim3 translateKernelGridDim = new dim3(translateKernelGridSizeXY, translateKernelGridSizeXY, 1);

            int shiftX = currentCellX - previousCellX;
            int shiftY = currentCellY - previousCellY;

            translateKernel.withPointer(previousGlobalMeanMap.data()).withLong(previousGlobalMeanMap.step());
            translateKernel.withPointer(previousGlobalVarianceMap.data()).withLong(previousGlobalVarianceMap.step());
            translateKernel.withPointer(globalMeanMap.data()).withLong(globalMeanMap.step());
            translateKernel.withPointer(globalVarianceMap.data()).withLong(globalVarianceMap.step());
            translateKernel.withInt(shiftX).withInt(shiftY);
            translateKernel.withPointer(parametersDevicePointer);
            translateKernel.withFloat(resetOffset);

            translateKernel.run(stream, translateKernelGridDim, blockSize, 0);

            translateKernelGridDim.close();
            checkCUDAError();

            previousCellX = currentCellX;
            previousCellY = currentCellY;
         }
      }

      // --------- Run the temp and local kernel ---------
      // Now we have all the data in the same "location" so we create our local height map
      // We have to split up the kernels first, the first one fills all the maps with data, the second one actually makes the local map
      {
         // Compute "speed" of the point
         RigidBodyTransform previousToCurrentSensorOrigin = new RigidBodyTransform(previousSensorToWorld);
         previousToCurrentSensorOrigin.invert();
         previousToCurrentSensorOrigin.multiply(sensorToWorldTransform);

         Vector3DBasics translation = previousToCurrentSensorOrigin.getTranslation();
         float linearMotionMagnitude = (float) translation.norm();

         RotationMatrixBasics rotation = previousToCurrentSensorOrigin.getRotation();
         AxisAngle axisAngle = new AxisAngle(rotation);
         float angularMotionMagnitude = (float) Math.abs(axisAngle.getAngle());

         sensorToWorldAlignedGround.get(this.sensorToWorldAlignedGroundTransformArray);
         sensorToWorldAlignedGroundTransformHostPointer.put(this.sensorToWorldAlignedGroundTransformArray);
         CUDATools.mallocAsync(sensorToWorldAlignedGroundTransformDevicePointer, this.sensorToWorldAlignedGroundTransformArray.length, stream);
         CUDATools.memcpyAsync(sensorToWorldAlignedGroundTransformDevicePointer,
                               sensorToWorldAlignedGroundTransformHostPointer,
                               this.sensorToWorldAlignedGroundTransformArray.length,
                               stream);

         // Clearing all the temp maps so they are ready for the next update call
         cudaMemset2DAsync(tempSumMap.data(), tempSumMap.step(), 0, (long) tempSumMap.cols() * Float.BYTES, tempSumMap.rows());
         cudaMemset2DAsync(tempCountMap.data(), tempCountMap.step(), 0, (long) tempCountMap.cols() * Integer.BYTES, tempCountMap.rows());
         cudaMemset2DAsync(tempSumOfSquaresMap.data(),
                           tempSumOfSquaresMap.step(),
                           0,
                           (long) tempSumOfSquaresMap.cols() * Float.BYTES,
                           tempSumOfSquaresMap.rows());
         cudaMemset2DAsync(tempMotionVarianceMap.data(),
                           tempMotionVarianceMap.step(),
                           0,
                           (long) tempMotionVarianceMap.cols() * Float.BYTES,
                           tempMotionVarianceMap.rows());

         int gridDimX = (latestDepthImageGPU.cols() + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
         int gridDimY = (latestDepthImageGPU.rows() + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
         dim3 updateTempMapsDim = new dim3(gridDimX, gridDimY, 1);

         updateTempMapsKernel.withPointer(latestDepthImageGPU.data()).withLong(latestDepthImageGPU.step());
         updateTempMapsKernel.withPointer(tempSumMap.data()).withLong(tempSumMap.step());
         updateTempMapsKernel.withPointer(tempCountMap.data()).withLong(tempCountMap.step());
         updateTempMapsKernel.withPointer(tempSumOfSquaresMap.data()).withLong(tempSumOfSquaresMap.step());
         updateTempMapsKernel.withPointer(tempMotionVarianceMap.data()).withLong(tempMotionVarianceMap.step());
         updateTempMapsKernel.withPointer(parametersDevicePointer);
         updateTempMapsKernel.withPointer(sensorToWorldAlignedGroundTransformDevicePointer);
         updateTempMapsKernel.withFloat(linearMotionMagnitude);
         updateTempMapsKernel.withFloat(angularMotionMagnitude);

         updateTempMapsKernel.run(stream, updateTempMapsDim, blockSize, 0);

         // That is the end of kernel one, now onto the local kernel

         localMapKernel.withPointer(tempSumMap.data()).withLong(tempSumMap.step());
         localMapKernel.withPointer(tempCountMap.data()).withLong(tempCountMap.step());
         localMapKernel.withPointer(tempSumOfSquaresMap.data()).withLong(tempSumOfSquaresMap.step());
         localMapKernel.withPointer(tempMotionVarianceMap.data()).withLong(tempMotionVarianceMap.step());
         localMapKernel.withPointer(localMeanMap.data()).withLong(localMeanMap.step());
         localMapKernel.withPointer(localVarianceMap.data()).withLong(localVarianceMap.step());
         localMapKernel.withPointer(localMotionVarianceMap.data()).withLong(localMotionVarianceMap.step());
         localMapKernel.withPointer(parametersDevicePointer);

         int computeLocalKernelGridXY = (localCellsPerAxis + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
         dim3 localKernelDim = new dim3(computeLocalKernelGridXY, computeLocalKernelGridXY, 1);

         localMapKernel.run(stream, localKernelDim, blockSize, 0);

         updateTempMapsDim.close();
         localKernelDim.close();
         cudaFreeAsync(sensorToWorldAlignedGroundTransformDevicePointer, stream);
         checkCUDAError();
      }

      // ---------- Run the ICP drift-correction step ----------
      // The state estimator drifts in ways the translate kernel's known integer shift doesn't capture.
      // Find the rigid (x, y, z, yaw) transform that best aligns the old global map with this frame's
      // local data, iterating a few times, then apply it to the global map before registration.
      {
         int searchRadiusCells = Math.max(1, (int) Math.ceil(heightMapParameters.getIcpMaxHorizontalDrift() / heightMapParameters.getCellSize()));
         int maxIterations = heightMapParameters.getIcpMaxIterations();
         int minCorrespondenceCount = heightMapParameters.getIcpMinCorrespondenceCount();
         boolean rotationEnabled = heightMapParameters.getIcpRotationEnabled();
         double convergenceZMeters = heightMapParameters.getIcpConvergenceZMeters();
         double convergenceYawRadians = Math.toRadians(heightMapParameters.getIcpConvergenceYawDegrees());
         double convergenceXYMeters = heightMapParameters.getCellSize();

         float totalTx = 0.0f;
         float totalTy = 0.0f;
         float totalTz = 0.0f;
         float totalYaw = 0.0f;
         boolean appliedAnyCorrection = false;

         int icpCorrespondenceGridSizeXY = (localCellsPerAxis + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
         dim3 icpCorrespondenceGridDim = new dim3(icpCorrespondenceGridSizeXY, icpCorrespondenceGridSizeXY, 1);

         for (int iteration = 0; iteration < maxIterations; iteration++)
         {
            cudaMemsetAsync(icpAccumulatorDevicePointer, 0, (long) ICP_ACCUMULATOR_SIZE * Float.BYTES, stream);

            icpCorrespondenceKernel.withPointer(localMeanMap.data()).withLong(localMeanMap.step());
            icpCorrespondenceKernel.withPointer(globalMeanMap.data()).withLong(globalMeanMap.step());
            icpCorrespondenceKernel.withPointer(globalVarianceMap.data()).withLong(globalVarianceMap.step());
            icpCorrespondenceKernel.withFloat(heightMapCenter.getX32());
            icpCorrespondenceKernel.withFloat(heightMapCenter.getY32());
            icpCorrespondenceKernel.withPointer(groundToWorldTranslationDevicePointer);
            icpCorrespondenceKernel.withFloat(totalTx).withFloat(totalTy).withFloat(totalTz).withFloat(totalYaw);
            icpCorrespondenceKernel.withInt(searchRadiusCells);
            icpCorrespondenceKernel.withPointer(icpAccumulatorDevicePointer);
            icpCorrespondenceKernel.withPointer(parametersDevicePointer);

            icpCorrespondenceKernel.run(stream, icpCorrespondenceGridDim, blockSize, 0);

            CUDATools.memcpyAsync(icpAccumulatorHostPointer, icpAccumulatorDevicePointer, ICP_ACCUMULATOR_SIZE, stream);
            error = cudaStreamSynchronize(stream);
            CUDATools.checkCUDAError(error);

            float correspondenceCount = icpAccumulatorHostPointer.get(0);
            if (correspondenceCount < minCorrespondenceCount)
               break;

            float sumRqx = icpAccumulatorHostPointer.get(1);
            float sumRqy = icpAccumulatorHostPointer.get(2);
            float sumRq2 = icpAccumulatorHostPointer.get(3);
            float sumRx = icpAccumulatorHostPointer.get(4);
            float sumRy = icpAccumulatorHostPointer.get(5);
            float sumRz = icpAccumulatorHostPointer.get(6);
            float sumYawRhs = icpAccumulatorHostPointer.get(7);

            float dtx;
            float dty;
            float dtz = -sumRz / correspondenceCount;
            float dyaw = 0.0f;

            if (rotationEnabled)
            {
               double[][] normalMatrix = {{correspondenceCount, 0.0, -sumRqy}, {0.0, correspondenceCount, sumRqx}, {-sumRqy, sumRqx, sumRq2}};
               double[] rhs = {-sumRx, -sumRy, sumYawRhs};
               double[] solution = new double[3];

               if (solve3x3(normalMatrix, rhs, solution))
               {
                  dtx = (float) solution[0];
                  dty = (float) solution[1];
                  dyaw = (float) solution[2];
               }
               else
               {
                  dtx = -sumRx / correspondenceCount;
                  dty = -sumRy / correspondenceCount;
               }
            }
            else
            {
               dtx = -sumRx / correspondenceCount;
               dty = -sumRy / correspondenceCount;
            }

            totalTx += dtx;
            totalTy += dty;
            totalTz += dtz;
            totalYaw += dyaw;
            appliedAnyCorrection = true;

            boolean converged = Math.abs(dtz) < convergenceZMeters && Math.sqrt((double) dtx * dtx + (double) dty * dty) < convergenceXYMeters
                              && (!rotationEnabled || Math.abs(dyaw) < convergenceYawRadians);

            if (converged)
               break;
         }

         icpCorrespondenceGridDim.close();

         if (appliedAnyCorrection)
         {
            globalMeanMap.copyTo(previousGlobalMeanMap);
            globalVarianceMap.copyTo(previousGlobalVarianceMap);

            int icpApplyGridSizeXY = (globalCellsPerAxis + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
            dim3 icpApplyGridDim = new dim3(icpApplyGridSizeXY, icpApplyGridSizeXY, 1);

            icpApplyCorrectionKernel.withPointer(previousGlobalMeanMap.data()).withLong(previousGlobalMeanMap.step());
            icpApplyCorrectionKernel.withPointer(previousGlobalVarianceMap.data()).withLong(previousGlobalVarianceMap.step());
            icpApplyCorrectionKernel.withPointer(globalMeanMap.data()).withLong(globalMeanMap.step());
            icpApplyCorrectionKernel.withPointer(globalVarianceMap.data()).withLong(globalVarianceMap.step());
            icpApplyCorrectionKernel.withFloat(heightMapCenter.getX32());
            icpApplyCorrectionKernel.withFloat(heightMapCenter.getY32());
            icpApplyCorrectionKernel.withFloat(totalTx).withFloat(totalTy).withFloat(totalTz).withFloat(totalYaw);
            icpApplyCorrectionKernel.withPointer(parametersDevicePointer);
            icpApplyCorrectionKernel.withFloat(resetOffset);

            icpApplyCorrectionKernel.run(stream, icpApplyGridDim, blockSize, 0);

            icpApplyGridDim.close();
            checkCUDAError();
         }
      }

      // ---------- Run the registration kernel ----------
      // Ok so now we've got our local map, lets put that onto the global map
      {
         int registerKernelGridSizeXY = (localCellsPerAxis + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
         dim3 registerKernelGridDim = new dim3(registerKernelGridSizeXY, registerKernelGridSizeXY, 1);

         registerKernel.withPointer(localMeanMap.data()).withLong(localMeanMap.step());
         registerKernel.withPointer(localVarianceMap.data()).withLong(localVarianceMap.step());
         registerKernel.withPointer(localMotionVarianceMap.data()).withLong(localMotionVarianceMap.step());
         registerKernel.withPointer(globalMeanMap.data()).withLong(globalMeanMap.step());
         registerKernel.withPointer(globalVarianceMap.data()).withLong(globalVarianceMap.step());
         registerKernel.withFloat(heightMapCenter.getX32());
         registerKernel.withFloat(heightMapCenter.getY32());
         registerKernel.withPointer(groundToWorldTranslationDevicePointer);
         registerKernel.withPointer(parametersDevicePointer);

         registerKernel.run(stream, registerKernelGridDim, blockSize, 0);

         registerKernelGridDim.close();
         checkCUDAError();
      }

      float zeroValueForEmptySpaces = 0.0f;
      // ---------- Run the registration kernel for an empty global height map ----------
      {
         int emptyRegistrationGridSizeXY = (globalCellsPerAxis + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
         dim3 registerKernelGridDim = new dim3(emptyRegistrationGridSizeXY, emptyRegistrationGridSizeXY, 1);

         // Need to reset the empty global map before using it so when its filled it starts with all "zero" values
         emptyGlobalHeightMap.setTo(new Scalar(zeroValueForEmptySpaces));

         emptyRegisterKernel.withPointer(localMeanMap.data()).withLong(localMeanMap.step());
         emptyRegisterKernel.withPointer(emptyGlobalHeightMap.data()).withLong(emptyGlobalHeightMap.step());
         emptyRegisterKernel.withPointer(groundToWorldTranslationDevicePointer);
         emptyRegisterKernel.withPointer(parametersDevicePointer);

         emptyRegisterKernel.run(stream, registerKernelGridDim, blockSize, 0);

         registerKernelGridDim.close();
         checkCUDAError();
      }

      // ---------- Run the plan offset kernel ----------
      {
         int planOffsetGridSizeXY = (globalCellsPerAxis + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
         dim3 planOffsetKernelGridDim = new dim3(planOffsetGridSizeXY, planOffsetGridSizeXY, 1);

         // Run the plan offset kernel
         planOffsetKernel.withPointer(globalMeanMap.data()).withLong(globalMeanMap.step());
         planOffsetKernel.withPointer(emptyGlobalHeightMap.data()).withLong(emptyGlobalHeightMap.step());
         planOffsetKernel.withFloat(zDriftInMeters);
         planOffsetKernel.withFloat(zeroValueForEmptySpaces);
         planOffsetKernel.withPointer(parametersDevicePointer);

         planOffsetKernel.run(stream, planOffsetKernelGridDim, blockSize, 0);

         planOffsetKernelGridDim.close();
         checkCUDAError();
      }

      // All that memory we allocated on the GPU, need to free that up now
      cudaFreeAsync(parametersDevicePointer, stream);
      cudaFreeAsync(groundToWorldTranslationDevicePointer, stream);
      error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);

      // The center of this map should be centered in the world grid
      // The sensor origin isn't always at the center of a grid point, in fact it's often not in the center
      double currentCellX = (int) Math.round(heightMapCenter.getX32() / heightMapParameters.getCellSize()) * heightMapParameters.getCellSize();
      double currentCellY = (int) Math.round(heightMapCenter.getY32() / heightMapParameters.getCellSize()) * heightMapParameters.getCellSize();
      heightMapCenterPoint.set(currentCellX, currentCellY, 0.0);

      // Finished GPU kernels, let pack this into the height map data object
      Mat hostHeightMap = new Mat();
      globalMeanMap.download(hostHeightMap);
      HeightMapTools.convertToHeightMapData(hostHeightMap,
                                            heightMapData,
                                            heightMapCenterPoint,
                                            (float) heightMapParameters.getGlobalWidthInMeters(),
                                            (float) heightMapParameters.getCellSize());

      // Save sensorOrigin as previous for next update
      previousSensorToWorld.set(sensorToWorldTransform);
   }

   public float[] populateParameterArray(HeightMapParameters parameters, CameraIntrinsics cameraIntrinsics, double groundHeightGuess)
   {
      return new float[] {(float) parameters.getCellSize(),
                          (float) localCenterIndex,
                          (float) globalCenterIndex,
                          (float) localCellsPerAxis,
                          (float) globalCellsPerAxis,
                          (float) cameraIntrinsics.getHeight(),
                          (float) cameraIntrinsics.getWidth(),
                          (float) cameraIntrinsics.getCx(),
                          (float) cameraIntrinsics.getCy(),
                          (float) cameraIntrinsics.getFx(),
                          (float) cameraIntrinsics.getFy(),
                          (float) parameters.getMinHeightRegistration(),
                          (float) parameters.getMaxHeightRegistration(),
                          (float) parameters.getMinClampHeight(),
                          (float) parameters.getMaxClampHeight(),
                          (float) parameters.getKalmanFilterPredictionNoise(),
                          (float) parameters.getAdditionalTranslationalVarianceAdded(),
                          (float) parameters.getVariancePerMeter(),
                          (float) parameters.getVariancePerTranslationSpeed(),
                          (float) parameters.getVariancePerRotationSpeed(),
                          (float) groundHeightGuess,
                          (float) parameters.getMinDepthToAccept(),
                          (float) parameters.getIcpOutlierDistanceThreshold(),
                          (float) parameters.getIcpVariancePerMeterOfCorrection()};
   }

   public HeightMapData getHeightMapData()
   {
      return heightMapData;
   }

   public void destroy()
   {
      heightMapProgram.close();
      blockSize.close();

      updateTempMapsKernel.close();
      localMeanMap.close();
      translateKernel.close();
      icpCorrespondenceKernel.close();
      icpApplyCorrectionKernel.close();
      registerKernel.close();
      planOffsetKernel.close();
      emptyRegisterKernel.close();

      // Clean up each resource
      deallocateFloatPointer(sensorToWorldAlignedGroundTransformHostPointer, sensorToWorldAlignedGroundTransformDevicePointer, stream);
      deallocateFloatPointer(groundToWorldTranslationHostPointer, groundToWorldTranslationDevicePointer, stream);
      deallocateFloatPointer(parametersHostPointer, parametersDevicePointer, stream);
      deallocateFloatPointer(icpAccumulatorHostPointer, icpAccumulatorDevicePointer, stream);

      tempSumMap.close();
      tempCountMap.close();
      tempSumOfSquaresMap.close();
      tempMotionVarianceMap.close();

      localMeanMap.close();
      localVarianceMap.close();
      localMotionVarianceMap.close();

      globalMeanMap.close();
      globalVarianceMap.close();
      previousGlobalMeanMap.close();
      previousGlobalVarianceMap.close();
      emptyGlobalHeightMap.close();

      // At the end we have to destroy the stream to release the memory
      CUDAStreamManager.releaseStream(stream);
   }

   /**
    * If we are debugging the kernels with {@link HeightMapExtractor#PRINT_TIMING_FOR_KERNELS} then we want to synchronize the GPU
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

   /**
    * Solves the 3x3 linear system {@code matrix * solutionToPack = rhs} via Cramer's rule.
    *
    * @return false if the matrix is singular, in which case solutionToPack is left untouched.
    */
   private static boolean solve3x3(double[][] matrix, double[] rhs, double[] solutionToPack)
   {
      double determinant = determinant3x3(matrix);
      if (Math.abs(determinant) < 1e-9)
         return false;

      for (int column = 0; column < 3; column++)
      {
         double[][] substituted = {matrix[0].clone(), matrix[1].clone(), matrix[2].clone()};
         for (int row = 0; row < 3; row++)
            substituted[row][column] = rhs[row];

         solutionToPack[column] = determinant3x3(substituted) / determinant;
      }

      return true;
   }

   private static double determinant3x3(double[][] matrix)
   {
      return matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
           - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
           + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);
   }

   public GpuMat getHeightMap()
   {
      return globalMeanMap;
   }
}
