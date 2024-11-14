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
   public ExampleCUDAKernel2D()
   {
      Path programPath = Path.of(Objects.requireNonNull(getClass().getResource("matrix_element_wise_addition.cu")).getPath());
      CUstream_st stream = CUDAStreamManager.getStream();

      int width = (int) Math.sqrt(4096.0);
      int height = (int) Math.sqrt(4096.0);

      try (Mat matA = new Mat(height, width, opencv_core.CV_16UC1, new Scalar(3));
           Mat matB = new Mat(height, width, opencv_core.CV_16UC1, new Scalar(5));
           Mat cpuResult = new Mat();

           GpuMat gpuMatA = new GpuMat();
           GpuMat gpuMatB = new GpuMat();
           GpuMat gpuResult = new GpuMat(matA.size(), matA.type());

           CUDAProgram elementWiseAdditionProgram = new CUDAProgram(programPath);
           CUDAKernel additionKernel = elementWiseAdditionProgram.loadKernel("element_wise_add"))
      {
         gpuMatA.upload(matA);
         gpuMatB.upload(matB);

         additionKernel.withPointer(gpuMatA.data()).withLong(gpuMatA.step())
                       .withPointer(gpuMatB.data()).withLong(gpuMatB.step())
                       .withPointer(gpuResult.data()).withLong(gpuResult.step())
                       .withInt(gpuResult.rows()).withInt(gpuResult.cols())
                       .run(stream, new dim3(), new dim3(width / 2, height / 2, 1), 0);

         cudart.cudaStreamSynchronize(stream);

         gpuResult.download(cpuResult);

         for (int i = 0; i < cpuResult.rows(); ++i)
         {
            for (int j = 0; j < cpuResult.cols(); ++j)
            {
               System.out.print(cpuResult.row(i).col(j).data().getShort() + " ");
            }

            System.out.println();
         }
      }

      CUDAStreamManager.releaseStream(stream);
   }

   public static void main(String[] args)
   {
      new ExampleCUDAKernel2D();
   }
}
