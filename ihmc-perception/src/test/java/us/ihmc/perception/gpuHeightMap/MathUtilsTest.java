package us.ihmc.perception.gpuHeightMap;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.FloatPointer;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.perception.cuda.CUDAKernel;
import us.ihmc.perception.cuda.CUDAProgram;
import us.ihmc.perception.cuda.CUDAStreamManager;

import java.net.URL;

import static org.bytedeco.cuda.global.cudart.*;
import static org.junit.jupiter.api.Assertions.*;

public class MathUtilsTest
{
   @Test
   public void testDotProductCUDA()
   {
      URL programPath = getClass().getClassLoader().getResource("us/ihmc/perception/gpuHeightMap/MathUtilsTest.cu");
      URL headerPath = getClass().getClassLoader().getResource("us/ihmc/perception/gpuHeightMap/MathUtils.cuh");

      CUstream_st stream = CUDAStreamManager.getStream();
      CUDAProgram program = new CUDAProgram(programPath, headerPath);
      CUDAKernel kernel = program.loadKernel("test_math_utils_dot_product");

      FloatPointer gpuResultPointer = new FloatPointer(1);
      FloatPointer cpuResultPointer = new FloatPointer(1);

      cudaMallocAsync(gpuResultPointer, (long) cpuResultPointer.sizeof() * (cpuResultPointer.limit() + 1), stream);

      // Set up the vectors that will be used in the dot product
      Vector3D vectorA = new Vector3D();
      vectorA.setX(1.0);
      vectorA.setY(2.0);
      vectorA.setZ(3.0);

      Vector3D vectorB = new Vector3D();
      vectorB.setX(4.0);
      vectorB.setY(5.0);
      vectorB.setZ(6.0);

      kernel.withFloat((float) vectorA.getX()).withFloat((float) vectorA.getY()).withFloat((float) vectorA.getZ());
      kernel.withFloat((float) vectorB.getX()).withFloat((float) vectorB.getY()).withFloat((float) vectorB.getZ());
      kernel.withPointer(gpuResultPointer);

      kernel.run(stream, new dim3(), new dim3(), 0);

      cudaMemcpyAsync(cpuResultPointer, gpuResultPointer,gpuResultPointer.sizeof() * (gpuResultPointer.limit() + 1), cudaMemcpyDefault, stream);

      cudaStreamSynchronize(stream);

      float[] resultFromGKernel = new float[1];
      cpuResultPointer.get(resultFromGKernel);

      cudaFreeAsync(gpuResultPointer, stream);

      double expectedDotProduct = (float) vectorA.dot(vectorB);
      assertEquals(expectedDotProduct, resultFromGKernel[0]);

      cpuResultPointer.close();
      gpuResultPointer.close();

      kernel.close();
      program.close();
      CUDAStreamManager.releaseStream(stream);
   }
}

