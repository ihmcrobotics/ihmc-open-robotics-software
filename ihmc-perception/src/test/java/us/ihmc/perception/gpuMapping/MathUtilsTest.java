package us.ihmc.perception.gpuMapping;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.FloatPointer;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.perception.cuda.CUDAKernel;
import us.ihmc.perception.cuda.CUDAProgram;
import us.ihmc.perception.cuda.CUDAStreamManager;
import us.ihmc.perception.cuda.CUDATools;
import us.ihmc.robotics.geometry.LeastSquaresZPlaneFitter;

import java.net.URL;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.bytedeco.cuda.global.cudart.*;
import static org.junit.jupiter.api.Assertions.*;

public class MathUtilsTest
{
   @Test
   public void testDotProductCUDA() throws Exception
   {
      URL programPath = getClass().getClassLoader().getResource("us/ihmc/perception/gpuMapping/MathUtilsTest.cu");
      URL headerPath = getClass().getClassLoader().getResource("us/ihmc/perception/cuda/MathUtils.cuh");

      CUstream_st stream = CUDAStreamManager.getStream();
      CUDAProgram program = new CUDAProgram(programPath, headerPath);
      CUDAKernel kernel = program.loadKernel("test_math_utils_dot_product");

      FloatPointer gpuResultPointer = new FloatPointer(1);
      FloatPointer cpuResultPointer = new FloatPointer(1);

      CUDATools.mallocAsync(gpuResultPointer, 1, stream);

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

      CUDATools.memcpyAsync(cpuResultPointer, gpuResultPointer, 1, stream);

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

   @Test
   public void testSolveForPlaneCoefficients() throws Exception
   {
      Random random = new Random(32900);

      URL programPath = getClass().getClassLoader().getResource("us/ihmc/perception/gpuMapping/MathUtilsTest.cu");
      URL headerPath = getClass().getClassLoader().getResource("us/ihmc/perception/cuda/MathUtils.cuh");

      CUstream_st stream = CUDAStreamManager.getStream();
      CUDAProgram program = new CUDAProgram(programPath, headerPath);
      CUDAKernel kernel = program.loadKernel("test_best_fit_plane");

      int minPoints = 3;
      int maxPoints = 15;

      // cpu --> gpu
      FloatPointer gpuPointsPointer = new FloatPointer();
      FloatPointer cpuPointsPointer = new FloatPointer(3 * maxPoints);
      CUDATools.mallocAsync(gpuPointsPointer, 3 * maxPoints, stream);

      // gpu --> cpu
      FloatPointer pointSolutionPointer = new FloatPointer();
      FloatPointer normalSolutionPointer = new FloatPointer();
      FloatPointer squaredErrorPointer = new FloatPointer();

      cudaMallocHost(pointSolutionPointer, 3);
      cudaMallocHost(normalSolutionPointer, 3);
      cudaMallocHost(squaredErrorPointer, 1);

      LeastSquaresZPlaneFitter planeFitter = new LeastSquaresZPlaneFitter();
      Plane3D expectedPlane = new Plane3D();

      int numTests = 30;
      for (int i = 0; i < numTests; i++)
      {
         int numPoints = minPoints + random.nextInt(maxPoints - minPoints + 1);
         List<Point3D> points = new ArrayList<>();

         for (int j = 0; j < numPoints; j++)
         {
            points.add(EuclidCoreRandomTools.nextPoint3D(random, 5.0));
         }

         double expectedSquaredError = planeFitter.fitPlaneToPoints(points, expectedPlane);

         float[] pointsArray = new float[3 * maxPoints];
         for (int j = 0; j < points.size(); j++)
         {
            points.get(j).get(3 * j, pointsArray);
         }

         cpuPointsPointer.put(pointsArray);

         CUDATools.memcpyAsync(gpuPointsPointer, cpuPointsPointer, pointsArray.length, stream);

         // run kernel
         kernel.withPointer(gpuPointsPointer);
         kernel.withInt(points.size());
         kernel.withPointer(pointSolutionPointer);
         kernel.withPointer(normalSolutionPointer);
         kernel.withPointer(squaredErrorPointer);
         kernel.withInt(i);

         kernel.run(stream, new dim3(), new dim3(), 0);
         cudaStreamSynchronize(stream);

         Plane3D computedPlane = new Plane3D();
         computedPlane.getPoint().set(pointSolutionPointer.get(0), pointSolutionPointer.get(1), pointSolutionPointer.get(2));
         computedPlane.getNormal().set(normalSolutionPointer.get(0), normalSolutionPointer.get(1), normalSolutionPointer.get(2));

         Assertions.assertTrue(expectedPlane.epsilonEquals(computedPlane, 1.0e-5));
         Assertions.assertTrue(Math.abs(expectedSquaredError - squaredErrorPointer.get(0)) < 1.0e-4);
      }

      cudaFreeAsync(gpuPointsPointer, stream);
      cudaFreeHost(pointSolutionPointer);
      cudaFreeHost(normalSolutionPointer);
      cudaFreeHost(squaredErrorPointer);

      gpuPointsPointer.close();
      pointSolutionPointer.close();
      normalSolutionPointer.close();
      squaredErrorPointer.close();

      kernel.close();
      program.close();
      CUDAStreamManager.releaseStream(stream);
   }

   @Test
   void testClampCUDA() throws Exception
   {
      float value = 7;
      float minValue = 1;
      float maxValue = 5;

      URL programFile = getClass().getClassLoader().getResource("us/ihmc/perception/gpuMapping/MathUtilsTest.cu");
      URL headerFile = getClass().getClassLoader().getResource("us/ihmc/perception/cuda/MathUtils.cuh");

      CUstream_st stream = CUDAStreamManager.getStream();
      CUDAProgram program = new CUDAProgram(programFile, headerFile);
      CUDAKernel kernel = program.loadKernel("test_math_utils_clamp");

      FloatPointer gpuResultPointer = new FloatPointer(1);
      FloatPointer cpuResultPointer = new FloatPointer(1);

      cudaMallocAsync(gpuResultPointer, (long) cpuResultPointer.sizeof() * (cpuResultPointer.limit() + 1), stream);

      kernel.withFloat(value).withFloat(minValue).withFloat(maxValue);
      kernel.withPointer(gpuResultPointer);

      kernel.run(stream, new dim3(), new dim3(), 0);

      cudaMemcpyAsync(cpuResultPointer, gpuResultPointer, gpuResultPointer.sizeof() * (gpuResultPointer.limit() + 1), cudaMemcpyDefault, stream);

      cudaStreamSynchronize(stream);

      float[] resultFromGKernel = new float[1];
      cpuResultPointer.get(resultFromGKernel);

      cudaFreeAsync(gpuResultPointer, stream);

      int expectedValue = (int) maxValue;
      assertEquals(expectedValue, resultFromGKernel[0]);

      cpuResultPointer.close();
      gpuResultPointer.close();

      kernel.close();
      program.close();
      CUDAStreamManager.releaseStream(stream);
   }

   @Test
   public void testTransformPointCUDA() throws Exception
   {
      float[] cudaResult;
      cudaResult = runTransformPoint3d32_2OnCUDA();

      assertEquals(5.0f, cudaResult[0], 1e-6);
      assertEquals(7.0f, cudaResult[1], 1e-6);
      assertEquals(9.0f, cudaResult[2], 1e-6);
   }

   private float[] runTransformPoint3d32_2OnCUDA() throws Exception
   {
      URL programPath = getClass().getClassLoader().getResource("us/ihmc/perception/gpuMapping/MathUtilsTest.cu");
      URL headerPath = getClass().getClassLoader().getResource("us/ihmc/perception/cuda/MathUtils.cuh");

      CUstream_st stream = CUDAStreamManager.getStream();
      CUDAProgram program = new CUDAProgram(programPath, headerPath);
      CUDAKernel kernel = program.loadKernel("test_math_utils_transform_point");

      FloatPointer gpuResultPointer = new FloatPointer(3);
      FloatPointer cpuResultPointer = new FloatPointer(3);
      cudaMallocAsync(gpuResultPointer, gpuResultPointer.sizeof() * 3L, stream);

      float px = 1.0f, py = 2.0f, pz = 3.0f;
      float r0x = 1.0f, r0y = 0.0f, r0z = 0.0f;
      float r1x = 0.0f, r1y = 1.0f, r1z = 0.0f;
      float r2x = 0.0f, r2y = 0.0f, r2z = 1.0f;
      float tx = 4.0f, ty = 5.0f, tz = 6.0f;

      kernel.withFloat(px).withFloat(py).withFloat(pz);
      kernel.withFloat(r0x).withFloat(r0y).withFloat(r0z);
      kernel.withFloat(r1x).withFloat(r1y).withFloat(r1z);
      kernel.withFloat(r2x).withFloat(r2y).withFloat(r2z);
      kernel.withFloat(tx).withFloat(ty).withFloat(tz);
      kernel.withPointer(gpuResultPointer);

      kernel.run(stream, new dim3(), new dim3(), 0);

      cudaMemcpyAsync(cpuResultPointer, gpuResultPointer, gpuResultPointer.sizeof() * 3L, cudaMemcpyDefault, stream);
      cudaStreamSynchronize(stream);

      float[] resultFromGKernel = new float[3];
      cpuResultPointer.get(resultFromGKernel);

      cudaFreeAsync(gpuResultPointer, stream);

      cpuResultPointer.close();
      gpuResultPointer.close();
      kernel.close();
      program.close();
      CUDAStreamManager.releaseStream(stream);

      return resultFromGKernel;
   }

   // Test for CUDA MathUtils::transformPoint3D function. Compares results against Euclid methods.
   @Test
   public void testTransformPoint2CUDA() throws Exception
   {
      URL programPath = getClass().getClassLoader().getResource("us/ihmc/perception/gpuMapping/MathUtilsTest.cu");
      URL headerPath = getClass().getClassLoader().getResource("us/ihmc/perception/cuda/MathUtils.cuh");

      CUstream_st stream = CUDAStreamManager.getStream();
      try (CUDAProgram program = new CUDAProgram(programPath, headerPath);
           CUDAKernel kernel = program.loadKernel("test_math_utils_transform_point_2");
           FloatPointer transformCPUPointer = new FloatPointer(4L * 4L);
           FloatPointer transformGPUPointer = new FloatPointer();
           FloatPointer resultPoint = new FloatPointer())
      {
         Point3D point = new Point3D(1.0, 2.0, 3.0);
         RigidBodyTransform transform = new RigidBodyTransform(1.0, 0.0, 0.0, 4.0,
                                                               0.0, 1.0, 0.0, 5.0,
                                                               0.0, 0.0, 1.0, 6.0);
         float[] transformArray = new float[4 * 4];
         transform.get(transformArray);
         transformCPUPointer.put(transformArray);

         CUDATools.mallocAsync(transformGPUPointer, 4 * 4, stream);
         CUDATools.memcpyAsync(transformGPUPointer, transformCPUPointer, 4 * 4, stream);

         cudaMallocHost(resultPoint, 3L * resultPoint.sizeof());

         // Calculate transformation on GPU
         kernel.withFloat(point.getX32())
               .withFloat(point.getY32())
               .withFloat(point.getZ32())
               .withPointer(transformGPUPointer)
               .withPointer(resultPoint)
               .run(stream, new dim3(), new dim3(), 0);

         cudaStreamSynchronize(stream);

         // Calculate transform using Euclid library (on CPU)
         point.applyTransform(transform);

         // Ensure results match
         assertEquals(resultPoint.get(0), point.getX32(), 1E-6);
         assertEquals(resultPoint.get(1), point.getY32(), 1E-6);
         assertEquals(resultPoint.get(2), point.getZ32(), 1E-6);

         cudaFreeHost(resultPoint);
         cudaFreeAsync(transformGPUPointer, stream);
      }
      CUDAStreamManager.releaseStream(stream);
   }
}
