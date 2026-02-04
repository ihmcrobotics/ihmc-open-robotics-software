package us.ihmc.perception.gpuMapping;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.indexer.FloatIndexer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.log.LogTools;
import us.ihmc.perception.cuda.CUDAKernel;
import us.ihmc.perception.cuda.CUDAProgram;
import us.ihmc.perception.cuda.CUDAStreamManager;
import us.ihmc.perception.cuda.CUDATools;

import java.net.URL;

import static org.bytedeco.cuda.global.cudart.*;

public class HeightMapICPCalculator
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
   private final dim3 blockSize;

   private final CUDAKernel iceCorrespondenceKernel;
   private final GpuMat vectorMapGPU;

   private final FloatPointer parametersHostPointer;
   private final FloatPointer parametersDevicePointer;
   private final FloatPointer groundToWorldTranslationDevicePointerCopy;

   private Vector3D correctedTransformTranslationOnly;

   public HeightMapICPCalculator(HeightMapParameters heightMapParameters, CUstream_st stream)
   {
      this.heightMapParameters = heightMapParameters;
      this.stream = stream;

      // Load header and main file
      URL heightMapUtilsHeaderPath = getClass().getResource("HeightMapUtils.cuh");
      URL mathUtilsHeaderPath = getClass().getResource("/us/ihmc/perception/cuda/MathUtils.cuh");
      URL kernelPath = getClass().getResource("HeightMapICPFilter.cu");

      localCenterIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getLocalWidthInMeters(), heightMapParameters.getCellSize());
      localCellsPerAxis = 2 * localCenterIndex + 1;
      globalCenterIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getGlobalWidthInMeters(), heightMapParameters.getCellSize());
      globalCellsPerAxis = 2 * globalCenterIndex + 1;
      blockSize = new dim3(BLOCK_SIZE_XY, BLOCK_SIZE_XY, 1);

      try
      {
         heightMapICPProgram = new CUDAProgram(kernelPath, heightMapUtilsHeaderPath, mathUtilsHeaderPath);

         iceCorrespondenceKernel = heightMapICPProgram.loadKernel("heightMapICPKernel");
         iceCorrespondenceKernel.enableKernelTimings(PRINT_TIMING_FOR_KERNELS);

         vectorMapGPU = new GpuMat(localCellsPerAxis, localCellsPerAxis, opencv_core.CV_32FC3);

         parametersHostPointer = new FloatPointer(6);
         parametersDevicePointer = new FloatPointer();

         groundToWorldTranslationDevicePointerCopy = new FloatPointer();
      }
      catch (Exception e)
      {
         throw new RuntimeException(e);
      }
   }

   public void update(GpuMat localMap, GpuMat globalMap, FloatPointer groundToWorldTranslationDevicePointer, Point3D globalMapCenter)
   {
      int maxIterations = heightMapParameters.getIcpMaxIterations();
      double convergenceThreshold = heightMapParameters.getIcpConvergenceThreshold();

      float[] parametersArray = populateParameterArray(heightMapParameters);
      parametersHostPointer.put(parametersArray);
      CUDATools.mallocAsync(parametersDevicePointer, parametersArray.length, stream);
      CUDATools.memcpyAsync(parametersDevicePointer, parametersHostPointer, parametersArray.length, stream);

      CUDATools.mallocAsync(groundToWorldTranslationDevicePointerCopy, 16, stream);
      int error = cudaMemcpyAsync(groundToWorldTranslationDevicePointerCopy,
                                  groundToWorldTranslationDevicePointer,
                                  16 * Float.BYTES,
                                  cudaMemcpyDeviceToDevice,
                                  stream);
      CUDATools.checkCUDAError(error);

      int gridDimX = (localMap.rows() + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
      int gridDimY = (localMap.cols() + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
      dim3 icpCorrespondenceDim = new dim3(gridDimX, gridDimY, 1);
      Mat cpuVectorMap = new Mat();

      int i;
      double totalShift = 0.0;
      for (i = 0; i < maxIterations; i++)
      {
         // We run the icp kernel with the latest transform we have, when things start this will be the transform we are given, on the next pass it will be updated.
         iceCorrespondenceKernel.withPointer(localMap.data()).withLong(localMap.step());
         iceCorrespondenceKernel.withPointer(globalMap.data()).withLong(globalMap.step());
         iceCorrespondenceKernel.withPointer(groundToWorldTranslationDevicePointerCopy);
         iceCorrespondenceKernel.withFloat(globalMapCenter.getX32());
         iceCorrespondenceKernel.withFloat(globalMapCenter.getY32());
         iceCorrespondenceKernel.withPointer(vectorMapGPU.data()).withLong(vectorMapGPU.step());
         iceCorrespondenceKernel.withPointer(parametersDevicePointer);

         iceCorrespondenceKernel.run(stream, icpCorrespondenceDim, blockSize, 0);

         error = cudaStreamSynchronize(stream);
         CUDATools.checkCUDAError(error);

         // Before we use things from the gpu we have to synchronize to make sure the gpu has finished
         vectorMapGPU.download(cpuVectorMap);
         FloatIndexer indexer = cpuVectorMap.createIndexer();

         double sumX = 0;
         double sumY = 0;
         double sumZ = 0;
         long validCellCount = 0;

         // This gets the mean vector form the vector map
         for (int y = 0; y < cpuVectorMap.rows(); y++)
         {
            for (int x = 0; x < cpuVectorMap.cols(); x++)
            {
               // The vectorMap has 3 axis (x, y, z)
               float vx = indexer.get(y, x, 0);
               float vy = indexer.get(y, x, 1);
               float vz = indexer.get(y, x, 2);

               // Check for NaN (points that didn't find a global match)
               if (!Float.isNaN(vx) && !Float.isNaN(vy) && !Float.isNaN(vz))
               {
                  sumX += vx;
                  sumY += vy;
                  sumZ += vz;
                  validCellCount++;
               }
            }
         }

         // Don't wanna forget to free the memory!
         indexer.release();

         if (validCellCount == 0)
         {
            LogTools.info("Iteration " + i + ": No correspondences found!");
            break;
         }

         double dx = sumX / validCellCount;
         double dy = sumY / validCellCount;
         double dz = sumZ / validCellCount;

         LogTools.info("Mean vector on " + i + "th iteration: (" + dx + ", " + dy + ", " + dz + ")");

         double alpha = 1.0;
         applyCorrectionToTransform(groundToWorldTranslationDevicePointerCopy, alpha * dx, alpha * dy, alpha * dz, stream);

         // Check for convergence
         totalShift = Math.sqrt(dx * dx + dy * dy + dz * dz);
         if (totalShift < convergenceThreshold)
         {
            break;
         }
      }

      // Finished our GPU kernels, close the objects related to that
      icpCorrespondenceDim.close();
      cpuVectorMap.close();

      // After the loop, check if we exited due to max iterations
      if (i == maxIterations)
      {
         LogTools.info("Warning: Reached max iterations (" + maxIterations + ") without convergence. Last totalShift = " + totalShift);
      }

      // Copy the final corrected matrix from GPU to CPU
      float[] finalCorrectedTransform = new float[16];
      FloatPointer cpuFinalCorrectedTransformPointer = new FloatPointer(finalCorrectedTransform);
      CUDATools.memcpyAsync(cpuFinalCorrectedTransformPointer, groundToWorldTranslationDevicePointerCopy, 16, stream);
      cudaStreamSynchronize(stream);
      cpuFinalCorrectedTransformPointer.get(finalCorrectedTransform);

      correctedTransformTranslationOnly = new Vector3D(finalCorrectedTransform[3], finalCorrectedTransform[7], finalCorrectedTransform[11]);

      cpuFinalCorrectedTransformPointer.close();
   }

   public float[] populateParameterArray(HeightMapParameters parameters)
   {
      return new float[] {(float) parameters.getCellSize(),
                          (float) localCenterIndex,
                          (float) globalCenterIndex,
                          (float) localCellsPerAxis,
                          (float) globalCellsPerAxis,
                          10.0f};
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
   }

   public Vector3D getCorrectedTransform()
   {
      return correctedTransformTranslationOnly;
   }

   public void destroy()
   {
      parametersHostPointer.close();
      parametersDevicePointer.close();
      groundToWorldTranslationDevicePointerCopy.close();
      cudaFreeAsync(parametersHostPointer, stream);
      cudaFreeAsync(groundToWorldTranslationDevicePointerCopy, stream);

      blockSize.close();
      vectorMapGPU.close();
      iceCorrespondenceKernel.close();
      heightMapICPProgram.close();
      CUDAStreamManager.releaseStream(stream);
   }
}
