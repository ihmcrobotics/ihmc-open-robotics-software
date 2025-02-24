package us.ihmc.perception.cuda.examples;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.cudaExtent;
import org.bytedeco.cuda.cudart.cudaPitchedPtr;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import us.ihmc.perception.cuda.CUDAKernel;
import us.ihmc.perception.cuda.CUDAProgram;
import us.ihmc.perception.cuda.CUDAStreamManager;
import us.ihmc.perception.cuda.CUDATools;
import us.ihmc.perception.tools.PerceptionDebugTools;

import java.net.URL;

import static org.bytedeco.cuda.global.cudart.*;

public class CUDAExampleKernel3D
{
   /**
    * This is an example of how to pass in a 3d array into the CUDA kernel.
    * Each index will have values in each layer.
    * This averages each index's layer and returns the result in a {@link GpuMat}.
    */
   public CUDAExampleKernel3D() throws Exception
   {
      // Create a 2x2 with 5 layers
      int rows = 2;
      int cols = 2;
      int layers = 4;

      CUstream_st stream = CUDAStreamManager.getStream();

      URL programPath = getClass().getResource("matrix_3d.cu");
      String kernelName = "matrix_3d_example";
      CUDAProgram program = new CUDAProgram(programPath);
      CUDAKernel kernel = program.loadKernel(kernelName);

      // Allocate enough memory for all layers
      cudaPitchedPtr pointerTo3DArray = new cudaPitchedPtr();
      cudaExtent extent = make_cudaExtent(cols * Short.BYTES, rows, layers);
      int error = cudaMalloc3D(pointerTo3DArray, extent);
      CUDATools.checkCUDAError(error);

      for (int i = 0; i < layers; i++)
      {
         // To get the average to be a whole number do (i * 2 + 2)
         Mat cpuData = new Mat(rows, cols, opencv_core.CV_16UC1, new Scalar(i * 2 + 2));

         // Allocate memory for each layer, creating a 3d array
         cudaMemcpy2D(pointerTo3DArray.ptr().position(i * pointerTo3DArray.pitch() * pointerTo3DArray.ysize()),
                      pointerTo3DArray.pitch(),
                      cpuData.data(),
                      cpuData.step(),
                      pointerTo3DArray.xsize(),
                      pointerTo3DArray.ysize(),
                      cudaMemcpyDefault);

         // Note we can't close the GpuMat here cause we need to access the data later in the program, so add it to a list, and close the list at the end
         cpuData.close();
      }

      GpuMat result = new GpuMat(rows, cols, opencv_core.CV_16UC1);

      kernel.withPointer(pointerTo3DArray.ptr()).withLong(pointerTo3DArray.pitch());
      kernel.withPointer(result.data()).withLong(result.step());
      kernel.withLong(pointerTo3DArray.pitch() * rows);
      kernel.withInt(rows);
      kernel.withInt(cols);
      kernel.withInt(layers);

      dim3 gridDim = new dim3();
      dim3 blockDim = new dim3(cols, rows, 1);
      kernel.run(stream, gridDim, blockDim, 0);

      error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);

      // Print the result to make sure we get what we expect
      // In this example we go through 2,4,6,8 so we expect the average to be 5
      Mat cpuResult = new Mat();
      result.download(cpuResult);
      PerceptionDebugTools.printMat("Result", cpuResult, 1);

      kernel.close();
      program.close();
      gridDim.close();
      blockDim.close();

      CUDAStreamManager.releaseStream(stream);
   }

   public static void main(String[] args) throws Exception
   {
      new CUDAExampleKernel3D();
   }
}
