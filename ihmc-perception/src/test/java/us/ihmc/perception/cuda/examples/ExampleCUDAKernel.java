package us.ihmc.perception.cuda.examples;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.cuda.global.cudart;
import org.bytedeco.javacpp.FloatPointer;
import us.ihmc.log.LogTools;
import us.ihmc.perception.cuda.CUDAKernel;
import us.ihmc.perception.cuda.CUDAProgram;

import static org.bytedeco.cuda.global.cudart.*;

/**
 * This is a simple example of a kernel that adds two arrays together. The ways in which kernels can be run vary such that you can create all the JavaCPP Pointers
 * inside a try-with-resources. The user can create more threads on the GPU. This example attempted to keep things really simple and avoided most of that overhead.
 */
public class ExampleCUDAKernel
{
   private static final String KERNEL_TO_ADD_THE_VALUES_OF_TWO_ARRAYS = """
         extern "C"
         
         __global__
         void add_arrays(int n, float *x, float *y)
         {
            int index = blockIdx.x * blockDim.x + threadIdx.x;
            int stride = blockDim.x * gridDim.x;
            for (int i = index; i < n; i += stride)
               y[i] = x[i] + y[i];
         }
         """;

   // Even if you don't create a stream, CUDA will create a default one for you, so we make one ourselves to handle things better
   public ExampleCUDAKernel()
   {
      // Note this name does NOT have to match the name of the program, however for readability its ideal to have them match.
      String userFriendlyNameOfProgram = "userFriendlyNameOfProgram";
      // Note this name DOES have to match the name of the CUDA kernel you want to run
      String kernelName = "add_arrays";

      // We create a stream to synchronize the method calls that happen on the GPU.
      CUstream_st stream = new CUstream_st();
      // Allocates the memory for the stream and makes sure the GPU knows about the stream
      cudart.cudaStreamCreate(stream);

      // The CUDAProgram is going to hold the kernel code
      CUDAProgram program = new CUDAProgram(userFriendlyNameOfProgram, KERNEL_TO_ADD_THE_VALUES_OF_TWO_ARRAYS);
      CUDAKernel kernel = program.loadKernel(kernelName);

      // Primitive types can be passed directly into the kernel class.
      // However, because we use it in different places, we have made this a variable
      int arraySize = 5;

      // Allocating memory for an array and populating it with values in the constructor.
      // The values stored in these variables will be passed to the kernel
      FloatPointer cpuArrayX = new FloatPointer(1.0f, 2.0f, 3.0f, 4.0f, 5.0f);
      FloatPointer cpuArrayY = new FloatPointer(5.0f, 4.0f, 3.0f, 2.0f, 1.0f);

      // These will be pointers to the gpu memory, where we will upload the data too.
      FloatPointer gpuArrayX = new FloatPointer();
      FloatPointer gpuArrayY = new FloatPointer();

      // Allocate memory on the gpu, to allocate the right size we need to get the sizeof the datatype being passed to the gpu
      cudaMallocAsync(gpuArrayX, (long) gpuArrayX.sizeof() * arraySize, stream);
      cudaMallocAsync(gpuArrayY, (long) gpuArrayY.sizeof() * arraySize, stream);

      // This variable is specific to CUDA, docs can be found online
      // Feel free to try this link as well: https://docs.nvidia.com/cuda/cuda-runtime-api/group__CUDART__MEMORY.html
      int cudaDefaultValue = cudaMemcpyDefault;

      // Copy the cpu data into the gpu
      // (cpuArrayX.sizeof() * arraySize) in this case saying (byteSizeOfFloat * numberOfFloats)
      cudaMemcpyAsync(gpuArrayX, cpuArrayX, (long) cpuArrayX.sizeof() * arraySize, cudaDefaultValue, stream);
      cudaMemcpyAsync(gpuArrayY, cpuArrayY, (long) cpuArrayY.sizeof() * arraySize, cudaDefaultValue, stream);

      // Now we are ready to run the kernel, we need to pass in the correct parameters
      // The method call of the kernel looks like this: (void add_arrays(int n, float *x, float *y)) so it needs an (int, float pointer, float pointer)
      // This runs on the GPU, so we need to pass in the data that is stored on the GPU (except for primitive types)
      kernel.withInt(arraySize).withPointer(gpuArrayX).withPointer(gpuArrayY);

      // In this example, our array's have 5 values, so if we wanted to run on 5 threads for the blockSize we would do: (new dim3(5,1,1))
      kernel.run(stream, new dim3(), new dim3(), 0);

      // At this point the kernel may have run or is running on the GPU, when it finishes we need to copy the result back to the CPU
      // The kernel packs the result in the y array (y[i] = x[i] + y[i];) so we want to get those values on the CPU
      // We are asking the GPU do copy the data to the CPU when its ready, there isn't a guarantee that this happens now
      cudaMemcpyAsync(cpuArrayY, gpuArrayY, (long) gpuArrayY.sizeof() * arraySize, cudaDefaultValue, stream);

      // Synchronize the stream
      // This call waits until all asynchronous functions being executed on this stream finish.
      // We have to call this to ensure that the above memcpy finished, and we have data back in Java land
      cudaStreamSynchronize(stream);

      // Free the memory on the GPU now that we are done with it. The data in on the CPU so we don't need it anymore
      cudaFreeAsync(gpuArrayX, stream);
      cudaFreeAsync(gpuArrayY, stream);

      // Copy array Y to Java land, so we can get the data
      float[] javaArrayY = new float[arraySize];
      cpuArrayY.get(javaArrayY);
      LogTools.info("Results: {}", javaArrayY);

      // Since we didn't create these pointers in a try-with-resources, we have to close everything correctly
      program.close();
      kernel.close();

      cpuArrayX.close();
      cpuArrayY.close();
      gpuArrayX.close();
      gpuArrayY.close();

      // At the end we have to destroy the stream to release the memory
      cudart.cudaStreamDestroy(stream);
      stream.close();
   }

   public static void main(String[] args)
   {
      new ExampleCUDAKernel();
   }
}
