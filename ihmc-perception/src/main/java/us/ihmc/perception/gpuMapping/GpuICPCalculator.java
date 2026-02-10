package us.ihmc.perception.gpuMapping;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.perception.cuda.CUDAKernel;
import us.ihmc.perception.cuda.CUDAProgram;
import us.ihmc.perception.cuda.CUDAStreamManager;
import us.ihmc.perception.cuda.CUDATools;
import us.ihmc.perception.gpuMapping.HeightMapTools.FlattenedHeightMap;

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

      try
      {
         heightMapICPProgram = new CUDAProgram(kernelPath, heightMapUtilsHeaderPath, mathUtilsHeaderPath);

         nearestNeighborsKernel = heightMapICPProgram.loadKernel("findNearestNeighborsKernel");
         transformPointsKernel = heightMapICPProgram.loadKernel("transformPointsKernel");

         nearestNeighborsKernel.enableKernelTimings(PRINT_TIMING_FOR_KERNELS);
         transformPointsKernel.enableKernelTimings(PRINT_TIMING_FOR_KERNELS);

         blockSize = new dim3(BLOCK_SIZE_XY, 1, 1);
      }
      catch (Exception e)
      {
         throw new RuntimeException(e);
      }
   }

   public void computeICPErrorTransform(GpuMat localMap,
                                        GpuMat globalMap,
                                        Point3DReadOnly localMapCenter,
                                        Point3DReadOnly globalMapCenter,
                                        int localCenterIndex,
                                        int globalCenterIndex,
                                        RigidBodyTransform transformLocalToGlobalFromOdometry)
   {

      RigidBodyTransform correctedGlobalTransform = new RigidBodyTransform();
      correctedGlobalTransform.set(totalAccumulatedErrorTransform);
      correctedGlobalTransform.multiply(transformLocalToGlobalFromOdometry);

      double correctedX = correctedGlobalTransform.getTranslationX();
      double correctedY = correctedGlobalTransform.getTranslationY();
      double correctedZ = correctedGlobalTransform.getTranslationZ();

      // TODO need to use the total accumulated error transform rather then just the odometry
      FlattenedHeightMap flattenedLocalMap = HeightMapTools.flattenHeightMapToXYZ(localMap,
                                                                                  transformLocalToGlobalFromOdometry.getTranslationX(),
                                                                                  transformLocalToGlobalFromOdometry.getTranslationY(),
                                                                                  transformLocalToGlobalFromOdometry.getTranslationZ(),
                                                                                  localCenterIndex,
                                                                                  (float) heightMapParameters.getCellSize(),
                                                                                  0.0f);
      FlattenedHeightMap flattenedGlobalMap = HeightMapTools.flattenHeightMapToXYZ(globalMap,
                                                                                   globalMapCenter.getX32(),
                                                                                   globalMapCenter.getY32(),
                                                                                   globalMapCenter.getZ32(),
                                                                                   globalCenterIndex,
                                                                                   (float) heightMapParameters.getCellSize(),
                                                                                   0.0f);

      if (flattenedLocalMap.pointCount() == 0 || flattenedGlobalMap.pointCount() == 0)
         return;

      computeICPFromPointClouds(flattenedLocalMap.data(), flattenedLocalMap.pointCount(), flattenedGlobalMap.data(), flattenedGlobalMap.pointCount());

      flattenedLocalMap.data().close();
      flattenedGlobalMap.data().close();
   }

   public void computeICPFromPointClouds(FloatPointer cpuLocalDataPointer, int localPoints, FloatPointer cpuGlobalDataPointer, int globalPoints)
   {
      int localFloats = localPoints * 3;
      int globalFloats = globalPoints * 3;

      FloatPointer gpuLocalDataPointer = new FloatPointer();
      FloatPointer gpuGlobalDataPointer = new FloatPointer();

      CUDATools.mallocAsync(gpuLocalDataPointer, cpuLocalDataPointer.limit(), stream);
      CUDATools.memcpyAsync(gpuLocalDataPointer, cpuLocalDataPointer, cpuLocalDataPointer.limit(), stream);

      CUDATools.mallocAsync(gpuGlobalDataPointer, cpuGlobalDataPointer.limit(), stream);
      CUDATools.memcpyAsync(gpuGlobalDataPointer, cpuGlobalDataPointer, cpuGlobalDataPointer.limit(), stream);

      // Allocate result buffers on GPU
      IntPointer gpuCorrespondences = new IntPointer();
      FloatPointer gpuDistances = new FloatPointer();

      // Allocate CPU buffers for downloading data
      FloatPointer cpuLocalTransformed = new FloatPointer(localFloats);
      IntPointer cpuCorrespondences = new IntPointer(localPoints);
      FloatPointer cpuDistances = new FloatPointer(localPoints);

      CUDATools.mallocAsync(gpuCorrespondences, localPoints, stream);
      CUDATools.mallocAsync(gpuDistances, localPoints, stream);

      double translationThreshold = heightMapParameters.getIcpConvergenceThreshold();

      // Start with an Identity matrix (no movement)
      latestPointCloudErrorTransform.set(CommonOps_DDRM.identity(4));

      int gridSize = (localPoints + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
      dim3 gridDim = new dim3(gridSize, 1, 1);

      for (int i = 0; i < heightMapParameters.getIcpMaxIterations(); i++)
      {
         // Use the GPU pointers we made earlier
         nearestNeighborsKernel.withPointer(gpuLocalDataPointer);
         nearestNeighborsKernel.withPointer(gpuGlobalDataPointer);
         nearestNeighborsKernel.withPointer(gpuCorrespondences);
         nearestNeighborsKernel.withPointer(gpuDistances);

         // Note: Pass the number of points, not the number of floats
         nearestNeighborsKernel.withInt(localPoints);
         nearestNeighborsKernel.withInt(globalPoints);

         nearestNeighborsKernel.run(stream, gridDim, blockSize, 0);

         int error = cudaStreamSynchronize(stream);
         CUDATools.checkCUDAError(error);

         // Download data from GPU to CPU
         CUDATools.memcpyAsync(cpuLocalTransformed, gpuLocalDataPointer, localFloats, stream);
         CUDATools.memcpyAsync(cpuCorrespondences, gpuCorrespondences, localPoints, stream);
         CUDATools.memcpyAsync(cpuDistances, gpuDistances, localPoints, stream);

         float maxDistance = 1.0f;
         int validCount = 0;

         FloatPointer filteredLocal = new FloatPointer(localFloats);
         IntPointer filteredCorrespondences = new IntPointer(localPoints);

         // This loop is going through the local points, and filtered any data out that doesn't make sense
         // The result is the filtered local data, and the filtered correspondence
         for (int k = 0; k < localPoints; k++)
         {
            float d = cpuDistances.get(k);

            if (d > maxDistance || Float.isNaN(d) || Float.isInfinite(d))
               continue; // Reject correspondence

            // Copy local point (XYZ)
            int dstBase = validCount * 3;
            int srcBase = k * 3;
            filteredLocal.put(dstBase + 0, cpuLocalTransformed.get(srcBase + 0));
            filteredLocal.put(dstBase + 1, cpuLocalTransformed.get(srcBase + 1));
            filteredLocal.put(dstBase + 2, cpuLocalTransformed.get(srcBase + 2));

            // Copy correspondence index
            filteredCorrespondences.put(validCount, cpuCorrespondences.get(k));

            validCount++;
         }

         if (validCount < 10)
            break;

         // Compute transformation using SVD
         DMatrixRMaj incrementalTransform = HeightMapTools.computeTransformSVD(filteredLocal, cpuGlobalDataPointer, filteredCorrespondences, validCount);

         filteredLocal.close();
         filteredCorrespondences.close();

         DMatrixRMaj combined = new DMatrixRMaj(4, 4);
         CommonOps_DDRM.mult(incrementalTransform, latestPointCloudErrorTransform, combined);
         latestPointCloudErrorTransform.set(combined);

         // Extract translation vector length (Euclidean distance)
         double dx = incrementalTransform.get(0, 3);
         double dy = incrementalTransform.get(1, 3);
         double dz = incrementalTransform.get(2, 3);
         double moveDist = Math.sqrt(dx * dx + dy * dy + dz * dz);

         System.out.println("Iteration " + i);
         System.out.println("  Valid correspondences: " + validCount);
         System.out.println("  Incremental dx: " + incrementalTransform.get(0, 3));
         System.out.println("  Incremental dy: " + incrementalTransform.get(1, 3));
         System.out.println("  Incremental dz: " + incrementalTransform.get(2, 3));
         System.out.println("  Move distance: " + moveDist);

         if (moveDist < translationThreshold)
         {
            break; // Stop if we moved less then the threshold
         }

         // Flatten and convert from Double to Float
         // DMatrixRMaj stores data in a field called '.data'
         float[] h_matrix = new float[16];
         for (int j = 0; j < incrementalTransform.data.length; j++)
         {
            h_matrix[j] = (float) incrementalTransform.data[j];
         }

         // Wrap in a Host Pointer
         FloatPointer hostMatrixPtr = new FloatPointer(h_matrix);
         FloatPointer deviceMatrixPtr = new FloatPointer();
         CUDATools.mallocAsync(deviceMatrixPtr, 16, stream);
         CUDATools.memcpyAsync(deviceMatrixPtr, hostMatrixPtr, 16, stream);

         // Update the Points on the GPU
         int transformGridSize = (validCount + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
         dim3 transformGridDim = new dim3(transformGridSize, 1, 1);

         transformPointsKernel.withPointer(gpuLocalDataPointer);
         transformPointsKernel.withPointer(deviceMatrixPtr);
         transformPointsKernel.withInt(localPoints);

         transformPointsKernel.run(stream, transformGridDim, blockSize, 0);

         hostMatrixPtr.close();
         deviceMatrixPtr.close();
         cudaFreeAsync(deviceMatrixPtr, stream);

         CUDATools.checkCUDAError(error);
      }

      cpuLocalTransformed.close();
      cpuCorrespondences.close();
      cpuDistances.close();

      cudaFreeAsync(gpuLocalDataPointer, stream);
      cudaFreeAsync(gpuGlobalDataPointer, stream);
      cudaFreeAsync(gpuCorrespondences, stream);
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

   public void close()
   {
      nearestNeighborsKernel.close();
      transformPointsKernel.close();
      blockSize.close();
      heightMapICPProgram.close();
      CUDAStreamManager.releaseStream(stream);
   }
}
