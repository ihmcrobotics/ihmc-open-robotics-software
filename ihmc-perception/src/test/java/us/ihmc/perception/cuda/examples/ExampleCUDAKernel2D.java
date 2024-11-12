package us.ihmc.perception.cuda.examples;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.cuda.global.cudart;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import us.ihmc.perception.cuda.CUDAKernel;
import us.ihmc.perception.cuda.CUDAProgram;
import us.ihmc.perception.cuda.CUDAStreamManager;

import java.net.URISyntaxException;
import java.nio.file.Path;
import java.util.Objects;

public class ExampleCUDAKernel2D
{
   public ExampleCUDAKernel2D() throws URISyntaxException
   {
      CUstream_st stream = CUDAStreamManager.getStream();

      int width = 3;
      int height = 5;

      Mat matA = new Mat(height, width, opencv_core.CV_16UC1, new Scalar(3));
      Mat matB = new Mat(height, width, opencv_core.CV_16UC1, new Scalar(5));
      Mat cpuResult = new Mat();

      GpuMat gpuMatA = new GpuMat();
      GpuMat gpuMatB = new GpuMat();
      GpuMat gpuResult = new GpuMat(matA.size(), matA.type());

      gpuMatA.upload(matA);
      gpuMatB.upload(matB);

      Path programPath = Path.of(Objects.requireNonNull(getClass().getResource("matrix_element_wise_addition.cu")).toURI());
      CUDAProgram elementWiseAdditionProgram = new CUDAProgram(programPath);
      CUDAKernel additionKernel = elementWiseAdditionProgram.loadKernel("element_wise_add");

      additionKernel.withPointer(gpuMatA.data()).withPointer(gpuMatB.data()).withPointer(gpuResult.data()).withInt(gpuResult.cols()).withInt(gpuResult.rows());
      additionKernel.run(stream, new dim3(), new dim3(), 0);

      cudart.cudaStreamSynchronize(stream);

      gpuResult.download(cpuResult);

      for (int i = 0; i < cpuResult.cols(); ++i)
      {
         for (int j = 0; j < cpuResult.rows(); ++j)
         {
            System.out.println(cpuResult.data().getShort(i * cpuResult.cols() + j));
         }

         System.out.println();
      }

      CUDAStreamManager.releaseStream(stream);
   }

   public static void main(String[] args) throws URISyntaxException
   {
      new ExampleCUDAKernel2D();
   }
}
