package us.ihmc.perception.gpuMapping;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.perception.cuda.CUDAKernel;
import us.ihmc.perception.cuda.CUDAProgram;
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
   private final int localCellsPerAxis;
   private final int globalCellsPerAxis;

   private final HeightMapParameters heightMapParameters;
   private final CUstream_st stream;

   private final CUDAProgram heightMapICPProgram;

   private final CUDAKernel nearestNeighborsKernel;
   private final CUDAKernel transformPointsKernel;
   private final dim3 blockSize;
   private DMatrixRMaj totalErrorTransform;

   public HeightMapICPCalculator2(HeightMapParameters heightMapParameters, CUstream_st stream)
   {
      this.heightMapParameters = heightMapParameters;
      this.stream = stream;

      // Load header and main file
      URL heightMapUtilsHeaderPath = getClass().getResource("HeightMapUtils.cuh");
      URL mathUtilsHeaderPath = getClass().getResource("/us/ihmc/perception/cuda/MathUtils.cuh");
      URL kernelPath = getClass().getResource("HeightMapICPFilter2.cu");

      localCenterIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getLocalWidthInMeters(), heightMapParameters.getCellSize());
      localCellsPerAxis = 2 * localCenterIndex + 1;
      globalCenterIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getGlobalWidthInMeters(), heightMapParameters.getCellSize());
      globalCellsPerAxis = 2 * globalCenterIndex + 1;

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

      FloatPointer gpuLocalDataPointer = new FloatPointer(cpuLocalDataPointer.limit() * Float.BYTES);
      FloatPointer gpuGlobalDataPointer = new FloatPointer(cpuGlobalDataPointer.limit() * Float.BYTES);

      CUDATools.mallocAsync(gpuLocalDataPointer, cpuLocalDataPointer.limit() * Float.BYTES, stream);
      CUDATools.memcpyAsync(gpuLocalDataPointer, cpuLocalDataPointer, cpuLocalDataPointer.limit() * Float.BYTES, stream);
      CUDATools.mallocAsync(gpuGlobalDataPointer, cpuGlobalDataPointer.limit() * Float.BYTES, stream);
      CUDATools.memcpyAsync(gpuGlobalDataPointer, cpuGlobalDataPointer, cpuGlobalDataPointer.limit() * Float.BYTES, stream);

      // Allocate result buffers on GPU
      IntPointer gpuCorrespondences = new IntPointer();
      FloatPointer gpuDistances = new FloatPointer();

      // Allocate CPU buffers for downloading data
      FloatPointer cpuLocalTransformed = new FloatPointer(localFloats);
      FloatPointer cpuGlobalMap = new FloatPointer(globalFloats);
      IntPointer cpuCorrespondences = new IntPointer(localPoints);
      FloatPointer cpuDistances = new FloatPointer(localPoints);

      cudaMemcpy(cpuGlobalMap, gpuGlobalDataPointer, (long) globalFloats * Float.BYTES, cudaMemcpyDeviceToHost);
      CUDATools.mallocAsync(gpuCorrespondences, (long) localPoints * Integer.BYTES, stream);
      CUDATools.mallocAsync(gpuDistances, (long) localPoints * Float.BYTES, stream);

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

         nearestNeighborsKernel.run(stream, gridDim, blockSize, 0);

         int error = cudaStreamSynchronize(stream);
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

         // 3. Allocate Device Memory (if not already done)
         // Make sure to allocate BYTES: 16 * 4 = 64 bytes
         if (deviceMatrixPtr.isNull())
         {
            CUDATools.mallocAsync(deviceMatrixPtr, 16 * Float.BYTES, stream);
         }

         // 4. Copy to GPU
         CUDATools.memcpyAsync(deviceMatrixPtr, hostMatrixPtr, 16 * Float.BYTES, stream);

         // 3. Update the Points on the GPU

         int transformGridSize = (validCount + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
         dim3 transformGridDim = new dim3(transformGridSize, 1, 1);
         // We pass the R and t we just found into a new kernel
         transformPointsKernel.withPointer(gpuLocalDataPointer);
         transformPointsKernel.withPointer(deviceMatrixPtr);
         transformPointsKernel.withInt(localPoints);

         transformPointsKernel.run(stream, transformGridDim, blockSize, 0);

         error = cudaStreamSynchronize(stream);
         CUDATools.checkCUDAError(error);

         hostMatrixPtr.close();
         deviceMatrixPtr.close();
      }
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
