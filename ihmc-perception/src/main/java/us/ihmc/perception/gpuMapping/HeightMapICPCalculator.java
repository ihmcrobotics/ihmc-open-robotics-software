package us.ihmc.perception.gpuMapping;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.perception.cuda.CUDAKernel;
import us.ihmc.perception.cuda.CUDAProgram;
import us.ihmc.perception.cuda.CUDATools;

import java.net.URL;

import static org.bytedeco.cuda.global.cudart.*;

public class HeightMapICPCalculator
{
   private static final boolean PRINT_TIMING_FOR_KERNELS = false;
   /**
    * The choice of 16 here is to utilize more SMs (Multi Processors) on the GPU.
    * This was chosen based on GPU profiling and significantly effects performance.
    */
   private static final int BLOCK_SIZE_XY = 8;

   private final int localCenterIndex;
   private final int localCellsPerAxis;

   private final HeightMapParameters heightMapParameters;
   private final CUstream_st stream;

   private final CUDAProgram heightMapICPProgram;
   private final dim3 blockSize;

   private final CUDAKernel icpCoorespondenceKernel;
   private final GpuMat vectorMap;

   private final FloatPointer parametersHostPointer;
   private final FloatPointer parametersDevicePointer;
   private Vector3D meanVector3D;
   private float[] finalMatrix;

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
      blockSize = new dim3(BLOCK_SIZE_XY, BLOCK_SIZE_XY, 1);

      try
      {
         heightMapICPProgram = new CUDAProgram(kernelPath, heightMapUtilsHeaderPath, mathUtilsHeaderPath);

         icpCoorespondenceKernel = heightMapICPProgram.loadKernel("heightMapICPKernel");
         icpCoorespondenceKernel.enableKernelTimings(PRINT_TIMING_FOR_KERNELS);

         vectorMap = new GpuMat(localCellsPerAxis, localCellsPerAxis, opencv_core.CV_32FC3);

         parametersHostPointer = new FloatPointer(13);
         parametersDevicePointer = new FloatPointer();
      }
      catch (Exception e)
      {
         throw new RuntimeException(e);
      }
   }

   public void update(GpuMat localMeanMap, GpuMat globalMeanMap, FloatPointer groundToWorldTranslationDevicePointer, Point3D heightMapCenter)
   {
      int maxIterations = 10;
      double convergenceThreshold = 0.001;

      FloatPointer groundToWorldTranslationDevicePointerCopy = new FloatPointer();

      // 1. Allocate memory for the copy (16 floats for a 4x4 matrix)
      CUDATools.mallocAsync(groundToWorldTranslationDevicePointerCopy, 16, stream);

      // 2. Perform the GPU-to-GPU copy
      // cudaMemcpyDeviceToDevice is the standard way to clone data on the card
      int error = cudaMemcpyAsync(groundToWorldTranslationDevicePointerCopy,
                                  groundToWorldTranslationDevicePointer,
                                  16 * Float.BYTES,
                                  cudaMemcpyDeviceToDevice,
                                  stream);
      CUDATools.checkCUDAError(error);

      float[] parametersArray = populateParameterArray(heightMapParameters);
      parametersHostPointer.put(parametersArray);
      CUDATools.mallocAsync(parametersDevicePointer, parametersArray.length, stream);
      CUDATools.memcpyAsync(parametersDevicePointer, parametersHostPointer, parametersArray.length, stream);

      for (int i = 0; i < maxIterations; i++)
      {
         int gridDimX = (localMeanMap.rows() + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
         int gridDimY = (localMeanMap.cols() + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
         dim3 icpCorrespondenceDim = new dim3(gridDimX, gridDimY, 1);

         icpCoorespondenceKernel.withPointer(localMeanMap.data()).withLong(localMeanMap.step());
         icpCoorespondenceKernel.withPointer(globalMeanMap.data()).withLong(globalMeanMap.step());
         icpCoorespondenceKernel.withPointer(groundToWorldTranslationDevicePointerCopy);
         icpCoorespondenceKernel.withFloat(heightMapCenter.getX32());
         icpCoorespondenceKernel.withFloat(heightMapCenter.getY32());
         icpCoorespondenceKernel.withPointer(vectorMap.data()).withLong(vectorMap.step());
         icpCoorespondenceKernel.withPointer(parametersDevicePointer);

         icpCoorespondenceKernel.run(stream, icpCorrespondenceDim, blockSize, 0);

         icpCorrespondenceDim.close();

         error = cudaStreamSynchronize(stream);
         CUDATools.checkCUDAError(error);

         Mat cpuVectorMap = new Mat();
         vectorMap.download(cpuVectorMap);

         // 1. Create a mask where localMap has height (is not 0.0)
         Mat mask = new Mat();
         Mat cpuLocalMean = new Mat();
         localMeanMap.download(cpuLocalMean);
         opencv_core.compare(cpuLocalMean, new Mat(new Scalar(0.0)), mask, opencv_core.CMP_NE);

         // 2. Compute mean only for the pillar area
         Scalar meanVector = opencv_core.mean(cpuVectorMap, mask);

         double dx = meanVector.get(0);
         double dy = meanVector.get(1);
         double dz = meanVector.get(2);

         applyCorrectionToGpuMatrix(groundToWorldTranslationDevicePointerCopy, dx, dy, dz, stream);

         // 5. Check for convergence
         double totalShift = Math.sqrt(dx * dx + dy * dy + dz * dz);
         if (totalShift < convergenceThreshold)
         {
            break;
         }
      }

      finalMatrix = new float[16];
      FloatPointer hostPointer = new FloatPointer(finalMatrix);

      // Copy the final corrected matrix from GPU to CPU
      CUDATools.memcpyAsync(hostPointer, groundToWorldTranslationDevicePointerCopy, 16, stream);
      cudaStreamSynchronize(stream);
      hostPointer.get(finalMatrix);

      meanVector3D = new Vector3D(finalMatrix[3], finalMatrix[7], finalMatrix[11]);
   }

   public float[] populateParameterArray(HeightMapParameters parameters)
   {
      return new float[] {(float) parameters.getCellSize(), (float) localCenterIndex, (float) localCellsPerAxis, 5.0f};
   }

   /**
    * Updates the 4x4 transformation matrix on the GPU with the calculated translation residuals.
    * * @param matrixPtr The pointer to the 4x4 float matrix on the GPU
    *
    * @param dx The translation correction in X
    * @param dy The translation correction in Y
    * @param dz The translation correction in Z
    */
   private void applyCorrectionToGpuMatrix(FloatPointer matrixPtr, double dx, double dy, double dz, CUstream_st stream)
   {
      // 1. Create a temporary host array to hold the 16 elements of a 4x4 matrix
      float[] hostMatrix = new float[16];
      FloatPointer hostPointer = new FloatPointer(hostMatrix);

      // 2. Copy the current matrix from GPU to CPU
      CUDATools.memcpyAsync(hostPointer, matrixPtr, 16, stream);
      cudaStreamSynchronize(stream);
      hostPointer.get(hostMatrix);

      // 3. Apply the translation correction.
      // In a row-major 4x4 matrix, translation is typically at indices 3, 7, and 11.
      // [ R R R Tx ]  -> Index 3
      // [ R R R Ty ]  -> Index 7
      // [ R R R Tz ]  -> Index 11
      // [ 0 0 0 1  ]
      hostMatrix[3] += (float) dx;
      hostMatrix[7] += (float) dy;
      hostMatrix[11] += (float) dz;

      // 4. Copy the updated matrix back to the GPU
      hostPointer.put(hostMatrix);
      CUDATools.memcpyAsync(matrixPtr, hostPointer, 16, stream);
   }

   public float[] getFinalMatrix()
   {
      return finalMatrix;
   }

   public Vector3D getVectorMap()
   {
      return meanVector3D;
   }

   public void destroy()
   {
      blockSize.close();
      heightMapICPProgram.close();
   }
}
