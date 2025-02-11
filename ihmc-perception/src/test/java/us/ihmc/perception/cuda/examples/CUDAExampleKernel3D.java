package us.ihmc.perception.cuda.examples;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.cuda.global.cudart;
import org.bytedeco.javacpp.ShortPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import us.ihmc.perception.cuda.CUDAKernel;
import us.ihmc.perception.cuda.CUDAProgram;
import us.ihmc.perception.cuda.CUDAStreamManager;
import us.ihmc.perception.cuda.CUDATools;

import java.net.URL;
import java.util.ArrayList;
import java.util.List;

import static org.bytedeco.cuda.global.cudart.*;

public class CUDAExampleKernel3D
{
   public CUDAExampleKernel3D() throws Exception
   {
      int width = 2;
      int height = 2;
      int layers = 2;

      CUstream_st stream = CUDAStreamManager.getStream();

      GpuMat gpuMatExample = new GpuMat(height, width, opencv_core.CV_16UC1);
      long pitchA = gpuMatExample.step(); // CUDA pitch (in bytes)

      // Allocate enough memory for all layers
      ShortPointer heightMapHistory = new ShortPointer();
      cudaMalloc(heightMapHistory, layers * pitchA * height);
      int error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);

      URL programPath = getClass().getResource("matrix_3d.cu");
      String kernelName = "matrix_3d_example";


      Mat cpuData1 = new Mat(height, width, opencv_core.CV_16UC1);
      cpuData1.ptr(0, 0, 1);
      Mat cpuData2 = new Mat(height, width, opencv_core.CV_16UC1);
      cpuData2.ptr(0, 0, 2);

      List<Mat> heightMaps = new ArrayList<>();
      heightMaps.add(cpuData1);
      heightMaps.add(cpuData2);

      List<GpuMat> gpuHeightMaps = new ArrayList<>();

      for (int i = 0; i < layers; i++)
      {
         Mat cpuData = new Mat(height, width, opencv_core.CV_16UC1, new Scalar(i + 1));
//         cpuData.setTo(new Scalar(i + 1)); // Example: Fill with 1 for layer 0, 2 for layer 1

         GpuMat gpuData = new GpuMat();
         gpuData.upload(cpuData);

         // Copy this layer into the allocated memory
         cudaMemcpy2D(heightMapHistory.position(i * pitchA * height), pitchA,
                      gpuData.data(), gpuData.step(),
                      width * Short.BYTES, height,
                      cudaMemcpyHostToDevice);
      }

      CUDAProgram program = new CUDAProgram(programPath);
      CUDAKernel kernel = program.loadKernel(kernelName);

      kernel.withPointer(heightMapHistory);
      kernel.withLong(pitchA);
      kernel.withLong(pitchA * height);  // layerSize = pitchA * height
      kernel.withInt(height);
      kernel.withInt(width);
      kernel.withInt(layers);

      dim3 gridDim = new dim3(); // The same thing as ( new dim3(1, 1, 1); )
      dim3 blockDim = new dim3(width, height, 1);
      kernel.run(stream, gridDim, blockDim, 0);

      error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);

      cudaStreamSynchronize(stream);

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
