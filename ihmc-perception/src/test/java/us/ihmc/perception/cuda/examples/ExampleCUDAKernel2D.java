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

import java.net.URL;

/**
 * This is an example of using a more complex CUDA kernel that takes in an OpenCV {@link Mat} object. We do this often for kernels, so this example walks you
 * through how it would work for a simple case.
 * This uses more of the standard approach then the {@link ExampleCUDAKernel} did because we use the {@link CUDAStreamManager} and we are creating objects
 * in the try-with-resources statement.
 */
public class ExampleCUDAKernel2D
{
   public ExampleCUDAKernel2D()
   {
      // We load the kernel from resources; this is a good place to store kernels to separate them from the Java classes
      URL programPath = getClass().getResource("matrix_element_wise_addition.cu");
      // Create the stream
      CUstream_st stream = CUDAStreamManager.getStream();

      // Here we want to maximize the number of threads we can, this helps optimize the kernel for runtime.
      // By the rules of CUDA, a block cannot have more than a certain number of threads.
      // Older gpus have a limit of 512, while newer gpus have a limit of 1024.
      // So since we are using a matrix, we are going to have a height and a width, so those squared can't be more the 1024.
      // Here we expect a grid of 3x3
      int height = (int) Math.sqrt(9.0);
      int width = (int) Math.sqrt(9.0);

      // The following commented code below will fail because it will try to create more than 1024 threads within one block, try it...
      //      int height = (int) Math.sqrt(1024.0) + 1;
      //      int width = (int) Math.sqrt(1024.0) + 1;

      // Note this name DOES have to match the name of the CUDA kernel you want to run, that means the name of the method.
      //  It's not guaranteed to be the name of the file either if the ( .cu ) file contains more than one method
      String kernelName = "element_wise_add";

      // This is where we create all of our objects that will be used in the kernel, the following formatting will be gross because of all the comments
      try (
            // This creates the inputs to the kernel, as well as the cpu result we expect to get back and use in Java
            Mat cpuMatA = new Mat(height, width, opencv_core.CV_16UC1, new Scalar(3));
            Mat cpuMatB = new Mat(height, width, opencv_core.CV_16UC1, new Scalar(5));
            Mat cpuResult = new Mat();

            // This creates objects that will be allocated on the gpu, these will be used in the kernel since that all happens on the gpu
            GpuMat gpuMatA = new GpuMat();
            GpuMat gpuMatB = new GpuMat();
            // We need to know the size (width and height) to allocate the right amount on one gpu,
            // the type() returns the data type (e.g., 16 bit unsigned short)
            GpuMat gpuResult = new GpuMat(cpuMatA.size(), cpuMatA.type());

            // Create the program and kernel to manage a lot of the over head for you, like creating and destroying. Let these classes handle it.
            CUDAProgram cudaProgram = new CUDAProgram(programPath);
            CUDAKernel cudaKernel = cudaProgram.loadKernel(kernelName))
      {
         // This method has nice Javadoc, but for clarity it allocates memory onto the gpu
         gpuMatA.upload(cpuMatA);
         gpuMatB.upload(cpuMatB);

         // Ok, let's set up the kernel with all the data
         // Our kernel expects a few matrices, and the pitch (step) of those matrices
         cudaKernel.withPointer(gpuMatA.data()).withLong(gpuMatA.step());
         cudaKernel.withPointer(gpuMatB.data()).withLong(gpuMatB.step());
         cudaKernel.withPointer(gpuResult.data()).withLong(gpuResult.step());
         cudaKernel.withInt(gpuResult.rows()).withInt(gpuResult.cols());

         // We need to worry about closing these variables
         dim3 gridDim = new dim3(); // The same thing as ( new dim3(1, 1, 1); )
         dim3 blockDim = new dim3(width, height, 1);

         // Runs the kernel with the desired grid and block sizes
         cudaKernel.run(stream, gridDim, blockDim, 0);

         // Synchronize the stream
         // This call waits until all asynchronous functions being executed on this stream finish.
         // We have to call this to ensure that the kernel finished, and the result is ready to be downloaded onto the CPU
         cudart.cudaStreamSynchronize(stream);

         // This is where we are pulling the result from the gpu. The download packs the variable being passed in
         gpuResult.download(cpuResult);

         printResult(cpuResult);

         // Remember to close these!
         gridDim.close();
         blockDim.close();
      }

      CUDAStreamManager.releaseStream(stream);
   }

   /**
    * This prints the result to the terminal, helpful for the user to see what's going on
    *
    * @param cpuResult the results that has been filled from the gpu after the kernel has run
    */
   private static void printResult(Mat cpuResult)
   {
      for (int i = 0; i < cpuResult.rows(); ++i)
      {
         for (int j = 0; j < cpuResult.cols(); ++j)
         {
            // Don't want to make it to easy to get the data though :)
            System.out.print(cpuResult.row(i).col(j).data().getShort() + " ");
         }

         System.out.println();
      }
   }

   public static void main(String[] args)
   {
      new ExampleCUDAKernel2D();
   }
}
