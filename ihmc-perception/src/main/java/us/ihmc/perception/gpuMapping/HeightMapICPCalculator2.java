package us.ihmc.perception.gpuMapping;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.perception.cuda.CUDAKernel;
import us.ihmc.perception.cuda.CUDAProgram;
import us.ihmc.perception.cuda.CUDAStreamManager;
import us.ihmc.perception.cuda.CUDATools;
import us.ihmc.perception.gpuMapping.HeightMapTools.FlattenedHeightMap;

import java.net.URL;

import static org.bytedeco.cuda.global.cudart.*;

public class HeightMapICPCalculator2
{
   private static final boolean PRINT_TIMING_FOR_KERNELS = false;
   private static final int BLOCK_SIZE_XY = 8;

   private final int localCenterIndex;
   private final int globalCenterIndex;

   private final HeightMapParameters heightMapParameters;
//   private final CUstream_st stream;

   private final CUDAProgram heightMapICPProgram;

   private final CUDAKernel nearestNeighborsKernel;
   private final CUDAKernel transformPointsKernel;
   private final dim3 blockSize;
   private DMatrixRMaj totalErrorTransform;
   private CUstream_st transformStream = new CUstream_st();
   private CUstream_st nearStream = new CUstream_st();

   public HeightMapICPCalculator2(HeightMapParameters heightMapParameters)
   {
      this.heightMapParameters = heightMapParameters;
//      this.stream = CUDAStreamManager.getStream();

      // Load header and main file
      URL heightMapUtilsHeaderPath = getClass().getResource("HeightMapUtils.cuh");
      URL mathUtilsHeaderPath = getClass().getResource("/us/ihmc/perception/cuda/MathUtils.cuh");
      URL kernelPath = getClass().getResource("HeightMapICPFilter2.cu");

      localCenterIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getLocalWidthInMeters(), heightMapParameters.getCellSize());
      globalCenterIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getGlobalWidthInMeters(), heightMapParameters.getCellSize());

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

   public void update(GpuMat localMap, GpuMat globalMap, Point3DReadOnly localMapCenter, Point3DReadOnly globalMapCenter)
   {
      FlattenedHeightMap flattenedLocalMap = HeightMapTools.flattenHeightMapToXYZ(localMap,
                                                                                  localMapCenter.getX32(),
                                                                                  localMapCenter.getY32(),
                                                                                  localCenterIndex,
                                                                                  (float) heightMapParameters.getCellSize(),
                                                                                  0.0f);
      FlattenedHeightMap flattenedGlobalMap = HeightMapTools.flattenHeightMapToXYZ(globalMap,
                                                                                   globalMapCenter.getX32(),
                                                                                   globalMapCenter.getY32(),
                                                                                   globalCenterIndex,
                                                                                   (float) heightMapParameters.getCellSize(),
                                                                                   0.0f);

      updateInternal(flattenedLocalMap.data(), flattenedGlobalMap.pointCount(), flattenedGlobalMap.data(), flattenedGlobalMap.pointCount());
   }

   public void updateInternal(FloatPointer cpuLocalDataPointer, int localPoints, FloatPointer cpuGlobalDataPointer, int globalPoints)
   {
      int localFloats = localPoints * 3;
      int globalFloats = globalPoints * 3;

      FloatPointer gpuLocalDataPointer = new FloatPointer();
      FloatPointer gpuGlobalDataPointer = new FloatPointer();

      cudaMalloc(gpuLocalDataPointer, cpuLocalDataPointer.limit() * Float.BYTES);
      cudaMemcpy(gpuLocalDataPointer, cpuLocalDataPointer, cpuLocalDataPointer.limit() * Float.BYTES, cudaMemcpyHostToDevice);
      cudaMalloc(gpuGlobalDataPointer, cpuGlobalDataPointer.limit() * Float.BYTES);
      cudaMemcpy(gpuGlobalDataPointer, cpuGlobalDataPointer, cpuGlobalDataPointer.limit() * Float.BYTES, cudaMemcpyHostToDevice);

      // Allocate result buffers on GPU
      IntPointer gpuCorrespondences = new IntPointer();
      FloatPointer gpuDistances = new FloatPointer();

      // Allocate CPU buffers for downloading data
      FloatPointer cpuLocalTransformed = new FloatPointer(localFloats);
      FloatPointer cpuGlobalMap = new FloatPointer(globalFloats);
      IntPointer cpuCorrespondences = new IntPointer(localPoints);
      FloatPointer cpuDistances = new FloatPointer(localPoints);

      cudaMemcpy(cpuGlobalMap, gpuGlobalDataPointer, (long) globalFloats * Float.BYTES, cudaMemcpyDeviceToHost);
      cudaMalloc(gpuCorrespondences, (long) localPoints * Integer.BYTES);
      cudaMalloc(gpuDistances, (long) localPoints * Float.BYTES);

      double translationThreshold = heightMapParameters.getIcpConvergenceThreshold();

      // --- INITIALIZATION (Before the loop) ---
      // Start with an Identity matrix (no movement)
      totalErrorTransform = CommonOps_DDRM.identity(4);

      int gridSize = (localPoints + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
      dim3 gridDim = new dim3(gridSize, 1, 1);

      for (int i = 0; i < heightMapParameters.getIcpMaxIterations(); i++)
      {
         // Use the GPU pointers we grabbed in step 1
         nearestNeighborsKernel.withPointer(gpuLocalDataPointer);
         nearestNeighborsKernel.withPointer(gpuGlobalDataPointer);
         nearestNeighborsKernel.withPointer(gpuCorrespondences);
         nearestNeighborsKernel.withPointer(gpuDistances);

         // IMPORTANT: Pass the number of POINTS, not the number of floats
         nearestNeighborsKernel.withInt(localPoints);
         nearestNeighborsKernel.withInt(globalPoints);

         nearestNeighborsKernel.run(nearStream, gridDim, blockSize, 0);

         int error = cudaStreamSynchronize(nearStream);
         CUDATools.checkCUDAError(error);

         // Step 2: Download data from GPU to CPU
         cudaMemcpy(cpuLocalTransformed, gpuLocalDataPointer, (long) localFloats * Float.BYTES, cudaMemcpyDeviceToHost);
         cudaMemcpy(cpuCorrespondences, gpuCorrespondences, (long) localPoints * Integer.BYTES, cudaMemcpyDeviceToHost);
         cudaMemcpy(cpuDistances, gpuDistances, (long) localPoints * Float.BYTES, cudaMemcpyDeviceToHost);

         float maxDistance = 2.5f;
         int validCount = 0;

         FloatPointer filteredLocal = new FloatPointer(localFloats);
         IntPointer filteredCorrespondences = new IntPointer(localPoints);

         for (int k = 0; k < localPoints; k++)
         {
            float d = cpuDistances.get(k);

            if (d > maxDistance || Float.isNaN(d) || Float.isInfinite(d))
               continue; // reject correspondence

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

         // Step 3: Compute transformation using SVD
         DMatrixRMaj incrementalTransform = HeightMapTools.computeTransformSVD(filteredLocal, cpuGlobalMap, filteredCorrespondences, validCount);

         filteredLocal.close();
         filteredCorrespondences.close();

         DMatrixRMaj combined = new DMatrixRMaj(4, 4);
         CommonOps_DDRM.mult(incrementalTransform, totalErrorTransform, combined);
         totalErrorTransform.set(combined);

         // Extract translation vector length (Euclidean distance)
         double dx = incrementalTransform.get(0, 3);
         double dy = incrementalTransform.get(1, 3);
         double dz = incrementalTransform.get(2, 3);
         double moveDist = Math.sqrt(dx * dx + dy * dy + dz * dz);

         System.out.println("Iteration " + i);
         System.out.println("  Valid correspondences: " + validCount);
         System.out.println("  Incremental dx: " + incrementalTransform.get(0, 3));
         System.out.println("  Incremental dy: " + incrementalTransform.get(1, 3));
         System.out.println("  Total dx so far: " + totalErrorTransform.get(0, 3));
         System.out.println("  Move distance: " + moveDist);

         if (moveDist < translationThreshold)
         {
            break; // Stop if we moved less than 1mm
         }

         // Assuming incrementalTransform is 4x4 or 3x4
         float[] h_matrix = new float[16];

         // 1. Flatten and convert from Double to Float
         // DMatrixRMaj stores data in a field called '.data'
         for (int j = 0; j < incrementalTransform.data.length; j++)
         {
            h_matrix[j] = (float) incrementalTransform.data[j];
         }

         // 2. Wrap in a Host Pointer
         FloatPointer hostMatrixPtr = new FloatPointer(h_matrix);
         FloatPointer deviceMatrixPtr = new FloatPointer();

         // 4. Copy to GPU
         cudaMalloc(deviceMatrixPtr, 16 * Float.BYTES);
         cudaMemcpy(deviceMatrixPtr, hostMatrixPtr, 16 * Float.BYTES, cudaMemcpyHostToDevice);

         // 3. Update the Points on the GPU

         int transformGridSize = (validCount + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
         dim3 transformGridDim = new dim3(transformGridSize, 1, 1);
         // We pass the R and t we just found into a new kernel
         transformPointsKernel.withPointer(gpuLocalDataPointer);
         transformPointsKernel.withPointer(deviceMatrixPtr);
         transformPointsKernel.withInt(localPoints);

         transformPointsKernel.run(transformStream, transformGridDim, blockSize, 0);

         hostMatrixPtr.close();
         deviceMatrixPtr.close();
         cudaFree(deviceMatrixPtr);

         CUDATools.checkCUDAError(error);
      }

      cpuLocalTransformed.close();
      cpuGlobalMap.close();
      cpuCorrespondences.close();
      cpuDistances.close();

      cudaFree(gpuLocalDataPointer);
      cudaFree(gpuGlobalDataPointer);
      cudaFree(gpuCorrespondences);
      cudaFree(gpuDistances);
   }

   public Vector3D getTotalErrorTransform()
   {
      Vector3D result = new Vector3D();
      result.set(totalErrorTransform.get(0, 3), totalErrorTransform.get(1, 3), totalErrorTransform.get(2, 3));
      return result;
   }

   public void close()
   {
      nearestNeighborsKernel.close();
      transformPointsKernel.close();
      blockSize.close();
      heightMapICPProgram.close();
   }
}
