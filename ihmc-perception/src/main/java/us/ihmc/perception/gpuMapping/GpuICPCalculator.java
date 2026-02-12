package us.ihmc.perception.gpuMapping;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.perception.cuda.CUDAKernel;
import us.ihmc.perception.cuda.CUDAProgram;
import us.ihmc.perception.cuda.CUDAStreamManager;
import us.ihmc.perception.cuda.CUDATools;
import us.ihmc.perception.gpuMapping.HeightMapTools.FlattenedHeightMap;
import us.ihmc.perception.tools.PerceptionDebugTools;

import java.net.URL;

import static org.bytedeco.cuda.global.cudart.*;

public class GpuICPCalculator
{
   private static final boolean PRINT_TIMING_FOR_KERNELS = false;
   private static final int BLOCK_SIZE_XY = 8;
   private final CUstream_st stream;
   private final dim3 blockSize;

   private final HeightMapParameters heightMapParameters;

   private final CUDAProgram heightMapICPProgram;
   private final CUDAKernel nearestNeighborsKernel;
   private final CUDAKernel transformPointsKernel;

   private final DMatrixRMaj totalAccumulatedErrorTransform;
   private final DMatrixRMaj latestPointCloudErrorTransform;
   private final int localCenterIndex;
   private final int localCellsPerAxis;
   private final int globalCenterIndex;
   private final int globalCellsPerAxis;

   private final FloatPointer parametersHostPointer;
   private final FloatPointer parametersDevicePointer;

   public GpuICPCalculator(HeightMapParameters heightMapParameters)
   {
      this.heightMapParameters = heightMapParameters;
      stream = CUDAStreamManager.getStream();

      // Load header and main file
      URL heightMapUtilsHeaderPath = getClass().getResource("HeightMapUtils.cuh");
      URL mathUtilsHeaderPath = getClass().getResource("/us/ihmc/perception/cuda/MathUtils.cuh");
      URL kernelPath = getClass().getResource("GpuICPCalculator.cu");

      totalAccumulatedErrorTransform = CommonOps_DDRM.identity(4);
      latestPointCloudErrorTransform = CommonOps_DDRM.identity(4);

      localCenterIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getLocalWidthInMeters(), heightMapParameters.getCellSize());
      localCellsPerAxis = 2 * localCenterIndex + 1;
      globalCenterIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getGlobalWidthInMeters(), heightMapParameters.getCellSize());
      globalCellsPerAxis = 2 * globalCenterIndex + 1;

      try
      {
         heightMapICPProgram = new CUDAProgram(kernelPath, heightMapUtilsHeaderPath, mathUtilsHeaderPath);

         nearestNeighborsKernel = heightMapICPProgram.loadKernel("v2");
         transformPointsKernel = heightMapICPProgram.loadKernel("transformPointsKernel");

         nearestNeighborsKernel.enableKernelTimings(PRINT_TIMING_FOR_KERNELS);
         transformPointsKernel.enableKernelTimings(PRINT_TIMING_FOR_KERNELS);

         parametersHostPointer = new FloatPointer(6);
         parametersDevicePointer = new FloatPointer();

         blockSize = new dim3(BLOCK_SIZE_XY, BLOCK_SIZE_XY, 1);
      }
      catch (Exception e)
      {
         throw new RuntimeException(e);
      }
   }

   public void computeICPErrorTransform(GpuMat localMap,
                                        GpuMat globalMap,
                                        Point3D localMapCenter,
                                        Point3D globalMapCenter,
                                        FloatPointer transformLocalToGlobalFromOdometry)
   {

//      RigidBodyTransform correctedGlobalTransform = new RigidBodyTransform();
//      correctedGlobalTransform.set(totalAccumulatedErrorTransform);
//      correctedGlobalTransform.multiply(transformLocalToGlobalFromOdometry);

//      double correctedX = correctedGlobalTransform.getTranslationX();
//      double correctedY = correctedGlobalTransform.getTranslationY();
//      double correctedZ = correctedGlobalTransform.getTranslationZ();

      // TODO need to use the total accumulated error transform rather then just the odometry
//      FlattenedHeightMap flattenedLocalMap = HeightMapTools.flattenHeightMapToXYZ(localMap,
//                                                                                  transformLocalToGlobalFromOdometry.getTranslationX(),
//                                                                                  transformLocalToGlobalFromOdometry.getTranslationY(),
//                                                                                  transformLocalToGlobalFromOdometry.getTranslationZ(),
//                                                                                  localCenterIndex,
//                                                                                  (float) heightMapParameters.getCellSize(),
//                                                                                  0.0f);
//      FlattenedHeightMap flattenedGlobalMap = HeightMapTools.flattenHeightMapToXYZ(globalMap,
//                                                                                   globalMapCenter.getX32(),
//                                                                                   globalMapCenter.getY32(),
//                                                                                   globalMapCenter.getZ32(),
//                                                                                   globalCenterIndex,
//                                                                                   (float) heightMapParameters.getCellSize(),
//                                                                                   0.0f);

//      if (flattenedLocalMap.pointCount() == 0 || flattenedGlobalMap.pointCount() == 0)
//         return;

      computeICPFromPointClouds(localMap, globalMap, localMapCenter, globalMapCenter, transformLocalToGlobalFromOdometry);

//      computeICPFromPointClouds(flattenedLocalMap.data(), flattenedLocalMap.pointCount(), flattenedGlobalMap.data(), flattenedGlobalMap.pointCount());

//      flattenedLocalMap.data().close();
//      flattenedGlobalMap.data().close();
   }

   public void computeICPFromPointClouds(GpuMat localMap, GpuMat globalMap, Point3D localCenter, Point3D globalCenter, FloatPointer localToGlobalTransform)
   {
//      Mat what = new Mat();
//      globalMap.download(what);
//      PerceptionDebugTools.printMat("s", what, 10);
      double translationThreshold = heightMapParameters.getIcpConvergence();
      float xCorrectedDrift = 0.0f;
      float yCorrectedDrift = 0.0f;
      float zCorrectedDrift = 0.0f;

      // Start with an Identity matrix (no movement)
      latestPointCloudErrorTransform.set(CommonOps_DDRM.identity(4));

      float[] parametersArray = populateParameterArray(heightMapParameters);
      parametersHostPointer.put(parametersArray);
      CUDATools.mallocAsync(parametersDevicePointer, parametersArray.length, stream);
      CUDATools.memcpyAsync(parametersDevicePointer, parametersHostPointer, parametersArray.length, stream);

      IntPointer gpuLocalKeys = new IntPointer();
      IntPointer gpuGlobalKeys = new IntPointer();
      CUDATools.mallocAsync(gpuLocalKeys, (long) localCellsPerAxis * localCellsPerAxis, stream);
      CUDATools.mallocAsync(gpuGlobalKeys, (long) localCellsPerAxis * localCellsPerAxis, stream);

      FloatPointer gpuDistances = new FloatPointer();
      CUDATools.mallocAsync(gpuDistances, (long) localCellsPerAxis * localCellsPerAxis, stream);

      IntPointer gpuValidCounter = new IntPointer();
      CUDATools.mallocAsync(gpuValidCounter, 1, stream);

      int gridSize = (localCellsPerAxis + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
      dim3 gridDim = new dim3(gridSize, gridSize, 1);

      float[] transformHostArray = new float[16];
      FloatPointer transformHostPointer = new FloatPointer(transformHostArray);
      DMatrixRMaj currentTransform = new DMatrixRMaj(4, 4);
      DMatrixRMaj updatedTransform = new DMatrixRMaj(4, 4);

      int error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);

      for (int i = 0; i < heightMapParameters.getIcpMaxIterations(); i++)
      {
         cudaMemsetAsync(gpuValidCounter, 0, Integer.BYTES, stream);

         nearestNeighborsKernel.withPointer(localMap.data()).withLong(localMap.step());
         nearestNeighborsKernel.withPointer(globalMap.data()).withLong(globalMap.step());
         nearestNeighborsKernel.withPointer(gpuLocalKeys);
         nearestNeighborsKernel.withPointer(gpuGlobalKeys);
         nearestNeighborsKernel.withPointer(gpuDistances);
         nearestNeighborsKernel.withPointer(gpuValidCounter);
         nearestNeighborsKernel.withPointer(localToGlobalTransform);
         nearestNeighborsKernel.withFloat(globalCenter.getX32());
         nearestNeighborsKernel.withFloat(globalCenter.getY32());
         nearestNeighborsKernel.withPointer(parametersDevicePointer);
         nearestNeighborsKernel.withFloat(zCorrectedDrift);

         nearestNeighborsKernel.run(stream, gridDim, blockSize, 0);

         error = cudaStreamSynchronize(stream);
         CUDATools.checkCUDAError(error);

         IntPointer cpuValidCounter = new IntPointer(1);
         CUDATools.memcpyAsync(cpuValidCounter, gpuValidCounter, 1, stream);
         cudaStreamSynchronize(stream);

         IntPointer cpuLocalKeys = new IntPointer((long) localCellsPerAxis * localCellsPerAxis);
         CUDATools.memcpyAsync(cpuLocalKeys, gpuLocalKeys, (long) localCellsPerAxis * localCellsPerAxis, stream);
         IntPointer cpuGlobalKeys = new IntPointer((long) localCellsPerAxis * localCellsPerAxis);
         CUDATools.memcpyAsync(cpuGlobalKeys, gpuGlobalKeys, (long) localCellsPerAxis * localCellsPerAxis, stream);
         cudaStreamSynchronize(stream);

         // Number of valid correspondences
         int validCount = cpuValidCounter.get(0);

         if (validCount <= 0)
         {
            System.out.println("No valid correspondences.");
            return;
         }

         // Compute center indices from map dimensions
         int localCenterIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getLocalWidthInMeters(), heightMapParameters.getCellSize());

         int globalCenterIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getGlobalWidthInMeters(), heightMapParameters.getCellSize());

         double resolution = heightMapParameters.getCellSize();

         // Allocate aligned point clouds (Nx3)
         float[] localCloud = new float[validCount * 3];
         float[] globalCloud = new float[validCount * 3];

         Mat cpuLocalMap = new Mat();
         localMap.download(cpuLocalMap);
         Mat cpuGlobalMap = new Mat();
         globalMap.download(cpuGlobalMap);
         cudaStreamSynchronize(stream);

//         PerceptionDebugTools.printMat("l", cpuLocalMap, 20);
//         PerceptionDebugTools.printMat("g", cpuGlobalMap, 20);


         for (int j = 0; j < validCount; j++)
         {
            int localKey = cpuLocalKeys.get(j);
            int globalKey = cpuGlobalKeys.get(j);

            double localX = HeightMapTools.keyToXCoordinate(localKey, 0.0, resolution, localCenterIndex);
            double localY = HeightMapTools.keyToYCoordinate(localKey, 0.0, resolution, localCenterIndex);
            double globalX = HeightMapTools.keyToXCoordinate(globalKey, globalCenter.getX(), resolution, globalCenterIndex);
            double globalY = HeightMapTools.keyToYCoordinate(globalKey, globalCenter.getY(), resolution, globalCenterIndex);
            float localZ = cpuLocalMap.data().getFloat((long) localKey * Float.BYTES);
            float globalZ = cpuGlobalMap.data().getFloat((long) globalKey * Float.BYTES);

            globalX -= xCorrectedDrift;
            globalY -= yCorrectedDrift;
            globalZ -= zCorrectedDrift;

            int base = j * 3;

            localCloud[base] = (float) localX;
            localCloud[base + 1] = (float) localY;
            localCloud[base + 2] = localZ;

            globalCloud[base] = (float) globalX;
            globalCloud[base + 1] = (float) globalY;
            globalCloud[base + 2] = globalZ;
         }

         FloatPointer filteredLocal = new FloatPointer(localCloud);
         FloatPointer filteredGlobal = new FloatPointer(globalCloud);
         DMatrixRMaj incrementalTransform = HeightMapTools.computeTransformSVD(filteredLocal, filteredGlobal, validCount);

         // ALPHA FILTERING (e.g., take only 20% of the calculated move)
         double alpha = 1.0;//heightMapParameters.getIcpAlphaFilter();
         incrementalTransform.set(0, 3, incrementalTransform.get(0, 3) * alpha); // dx
         incrementalTransform.set(1, 3, incrementalTransform.get(1, 3) * alpha); // dy
         incrementalTransform.set(2, 3, incrementalTransform.get(2, 3) * alpha); // dz

         DMatrixRMaj combined = new DMatrixRMaj(4, 4);
         CommonOps_DDRM.mult(incrementalTransform, latestPointCloudErrorTransform, combined);
         latestPointCloudErrorTransform.set(combined);

         // Extract translation vector length (Euclidean distance)
         double dx = incrementalTransform.get(0, 3);
         double dy = incrementalTransform.get(1, 3);
         double dz = incrementalTransform.get(2, 3);
         double moveDist = Math.sqrt(dx * dx + dy * dy + dz * dz);

         xCorrectedDrift += (float) dx;
         yCorrectedDrift += (float) dy;
         zCorrectedDrift += (float) dz;


         System.out.println("Iteration " + i);
         System.out.println("  Valid correspondences: " + validCount);
         System.out.println("  Incremental dx: " + incrementalTransform.get(0, 3));
         System.out.println("  Incremental dy: " + incrementalTransform.get(1, 3));
         System.out.println("  Incremental dz: " + incrementalTransform.get(2, 3));
         System.out.println("  Move distance: " + moveDist);

         // -----------------------------
         // Update localToGlobalTransform
         // -----------------------------

         // Copy current transform from GPU → CPU
         CUDATools.memcpyAsync(transformHostPointer, localToGlobalTransform, 16, stream);
         cudaStreamSynchronize(stream);
         transformHostPointer.get(transformHostArray);

         // Convert float[16] → DMatrixRMaj (row-major)
         for (int r = 0; r < 4; r++)
         {
            for (int c = 0; c < 4; c++)
            {
               currentTransform.set(r, c, transformHostArray[r * 4 + c]);
            }
         }

         // Compose: T_new = ΔT * T_old
         CommonOps_DDRM.mult(incrementalTransform, currentTransform, updatedTransform);

         // Convert back to float[16]
         for (int r = 0; r < 4; r++)
         {
            for (int c = 0; c < 4; c++)
            {
               transformHostArray[r * 4 + c] = (float) updatedTransform.get(r, c);
            }
         }

         // Copy updated transform CPU → GPU
         transformHostPointer.put(transformHostArray);
         CUDATools.memcpyAsync(localToGlobalTransform, transformHostPointer, 16, stream);
         cudaStreamSynchronize(stream);

         if (moveDist < translationThreshold)
         {
            break; // Stop if we moved less then the threshold
         }

         CUDATools.checkCUDAError(error);
      }

      cudaFreeAsync(gpuDistances, stream);

      // Track the total drift we have accumulated while running ICP
      DMatrixRMaj tempTotalAccumulatedErrorTransform = new DMatrixRMaj(4, 4);
      CommonOps_DDRM.mult(totalAccumulatedErrorTransform, latestPointCloudErrorTransform, tempTotalAccumulatedErrorTransform);
      totalAccumulatedErrorTransform.set(tempTotalAccumulatedErrorTransform);
   }

   public Vector3D getLatestPointCloudErrorTransform()
   {
      Vector3D result = new Vector3D();
      result.set(latestPointCloudErrorTransform.get(0, 3), latestPointCloudErrorTransform.get(1, 3), latestPointCloudErrorTransform.get(2, 3));
      return result;
   }

   /**
    * Updates the 4x4 transformation matrix on the GPU with the calculated translation residuals.
    * * @param matrixPtr The pointer to the 4x4 float matrix on the GPU
    *
    * @param dx The translation correction in X
    * @param dy The translation correction in Y
    * @param dz The translation correction in Z
    */
   public void applyCorrectionToTransform(FloatPointer matrixPtr, double dx, double dy, double dz, CUstream_st stream)
   {
      // Create a temporary host array to hold the 16 elements of a 4x4 matrix
      float[] hostMatrix = new float[16];
      FloatPointer hostPointer = new FloatPointer(hostMatrix);

      // Copy the current matrix from GPU to CPU
      CUDATools.memcpyAsync(hostPointer, matrixPtr, 16, stream);
      cudaStreamSynchronize(stream);
      hostPointer.get(hostMatrix);

      // In a row-major 4x4 matrix, translation is at indices 3, 7, and 11.
      // [ R R R Tx ]  -> Index 3
      // [ R R R Ty ]  -> Index 7
      // [ R R R Tz ]  -> Index 11
      // [ 0 0 0 1  ]
      hostMatrix[3] += (float) dx;
      hostMatrix[7] += (float) dy;
      hostMatrix[11] += (float) dz;

      // Copy the updated matrix back to the GPU
      hostPointer.put(hostMatrix);
      CUDATools.memcpyAsync(matrixPtr, hostPointer, 16, stream);
      hostPointer.close();
   }

   public float[] populateParameterArray(HeightMapParameters parameters)
   {
      return new float[] {(float) parameters.getCellSize(),
                          (float) localCenterIndex,
                          (float) globalCenterIndex,
                          (float) localCellsPerAxis,
                          (float) globalCellsPerAxis,
                          (float) heightMapParameters.getIcpSearchRadius()};
   }

   public void close()
   {
      nearestNeighborsKernel.close();
      transformPointsKernel.close();
      blockSize.close();
      heightMapICPProgram.close();
      CUDAStreamManager.releaseStream(stream);
   }

   public CUstream_st getStream()
   {
      return stream;
   }
}
