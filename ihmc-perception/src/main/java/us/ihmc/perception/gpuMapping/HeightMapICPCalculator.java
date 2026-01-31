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

import static org.bytedeco.cuda.global.cudart.cudaStreamSynchronize;

public class HeightMapICPCalculator
{
   private static final boolean PRINT_TIMING_FOR_KERNELS = false;
   /**
    * The choice of 16 here is to utilize more SMs (Multi Processors) on the GPU.
    * This was chosen based on GPU profiling and significantly effects performance.
    */
   private static final int BLOCK_SIZE_XY = 8;

   private final int centerIndex;
   private final int cellsPerAxis;

   private final HeightMapParameters heightMapParameters;
   private final CUstream_st stream;

   private final CUDAProgram heightMapICPProgram;
   private final dim3 blockSize;

   private final CUDAKernel icpCoorespondenceKernel;
   private final GpuMat vectorMap;

   private final FloatPointer parametersHostPointer;
   private final FloatPointer parametersDevicePointer;
   private Vector3D meanVector3D;

   public HeightMapICPCalculator(HeightMapParameters heightMapParameters, CUstream_st stream)
   {
      this.heightMapParameters = heightMapParameters;
      this.stream = stream;

      // Load header and main file
      URL heightMapUtilsHeaderPath = getClass().getResource("HeightMapUtils.cuh");
      URL mathUtilsHeaderPath = getClass().getResource("/us/ihmc/perception/cuda/MathUtils.cuh");
      URL kernelPath = getClass().getResource("HeightMapICPFilter.cu");

      centerIndex = HeightMapTools.computeCenterIndex(heightMapParameters.getWidthInMeters(), heightMapParameters.getCellSize());
      cellsPerAxis = 2 * centerIndex + 1;
      blockSize = new dim3(BLOCK_SIZE_XY, BLOCK_SIZE_XY, 1);

      try
      {
         heightMapICPProgram = new CUDAProgram(kernelPath, heightMapUtilsHeaderPath, mathUtilsHeaderPath);

         icpCoorespondenceKernel = heightMapICPProgram.loadKernel("heightMapICPKernel");
         icpCoorespondenceKernel.enableKernelTimings(PRINT_TIMING_FOR_KERNELS);

         vectorMap = new GpuMat(cellsPerAxis, cellsPerAxis, opencv_core.CV_32FC3);

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
      float[] parametersArray = populateParameterArray(heightMapParameters);
      parametersHostPointer.put(parametersArray);
      CUDATools.mallocAsync(parametersDevicePointer, parametersArray.length, stream);
      CUDATools.memcpyAsync(parametersDevicePointer, parametersHostPointer, parametersArray.length, stream);

      int gridDimX = (localMeanMap.rows() + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
      int gridDimY = (localMeanMap.cols() + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
      dim3 icpCorrespondenceDim = new dim3(gridDimX, gridDimY, 1);

      icpCoorespondenceKernel.withPointer(localMeanMap.data()).withLong(localMeanMap.step());
      icpCoorespondenceKernel.withPointer(globalMeanMap.data()).withLong(globalMeanMap.step());
      icpCoorespondenceKernel.withPointer(groundToWorldTranslationDevicePointer);
      icpCoorespondenceKernel.withFloat(heightMapCenter.getX32());
      icpCoorespondenceKernel.withFloat(heightMapCenter.getY32());
      icpCoorespondenceKernel.withPointer(vectorMap.data()).withLong(vectorMap.step());
      icpCoorespondenceKernel.withPointer(parametersDevicePointer);

      icpCoorespondenceKernel.run(stream, icpCorrespondenceDim, blockSize, 0);

      icpCorrespondenceDim.close();

      int error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);

      Mat what = new Mat();
      vectorMap.download(what);

      Scalar meanVector = opencv_core.mean(what);

      double meanX = meanVector.get(0);
      double meanY = meanVector.get(1);
      double meanZ = meanVector.get(2);

      meanVector3D = new Vector3D(meanX, meanY, meanZ);
   }

   public float[] populateParameterArray(HeightMapParameters parameters)
   {
      return new float[] {(float) parameters.getCellSize(),
                          (float) centerIndex,
                          (float) cellsPerAxis,
                          (float) parameters.getMinHeightRegistration(),
                          (float) parameters.getMaxHeightRegistration(),
                          (float) parameters.getMinClampHeight(),
                          (float) parameters.getMaxClampHeight(),
                          (float) parameters.getKalmanFilterPredictionNoise(),
                          (float) parameters.getAdditionalTranslationalVarianceAdded(),
                          (float) parameters.getVariancePerMeter(),
                          (float) parameters.getVariancePerTranslationSpeed(),
                          (float) parameters.getVariancePerRotationSpeed(),
                          (float) parameters.getMinDepthToAccept()};
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
