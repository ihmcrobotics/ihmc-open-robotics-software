package us.ihmc.perception.gpuMapping;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_mem;
import org.bytedeco.opencl._cl_program;
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
import us.ihmc.perception.opencl.OpenCLManager;
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

   @Test
   public void testTransformOpenCL() throws Exception
   {
      float[] openCLResult;
      openCLResult = runTransformPoint3d3d_2OnOpenCL();

      float[] cudaResult;
      cudaResult = runTransformPoint3d32_2OnCUDA();

      assertEquals(openCLResult[0], cudaResult[0], 1e-6);
      assertEquals(openCLResult[1], cudaResult[1], 1e-6);
      assertEquals(openCLResult[2], cudaResult[2], 1e-6);
   }

   @Test
   public void testTransformOpenCLMatchesCUDA()
   {
      float[] openCLResult;
      openCLResult = runTransformPoint3d3d_2OnOpenCL();

      assertEquals(5.0f, openCLResult[0], 1e-6);
      assertEquals(7.0f, openCLResult[1], 1e-6);
      assertEquals(9.0f, openCLResult[2], 1e-6);
   }

   private static float[] runTransformPoint3d3d_2OnOpenCL()
   {
      OpenCLManager openCLManager = new OpenCLManager();
      // The EuclidCommon.cl is included by default, so we don't need to worry about including the header
      _cl_program openCLProgram = openCLManager.loadProgram("EuclidCommonTest");
      _cl_kernel openCLKernel = openCLManager.createKernel(openCLProgram, "test_math_utils_transform_point");

      FloatPointer pxPointer = new FloatPointer(1);
      pxPointer.put(0, 1.0f);

      FloatPointer pyPointer = new FloatPointer(1);
      pyPointer.put(0, 2.0f);

      FloatPointer pzPointer = new FloatPointer(1);
      pzPointer.put(0, 3.0f);

      FloatPointer r0xPointer = new FloatPointer(1);
      r0xPointer.put(0, 1.0f);

      FloatPointer r0yPointer = new FloatPointer(1);
      r0yPointer.put(0, 0.0f);

      FloatPointer r0zPointer = new FloatPointer(1);
      r0zPointer.put(0, 0.0f);

      FloatPointer r1xPointer = new FloatPointer(1);
      r1xPointer.put(0, 0.0f);

      FloatPointer r1yPointer = new FloatPointer(1);
      r1yPointer.put(0, 1.0f);

      FloatPointer r1zPointer = new FloatPointer(1);
      r1zPointer.put(0, 0.0f);

      FloatPointer r2xPointer = new FloatPointer(1);
      r2xPointer.put(0, 0.0f);

      FloatPointer r2yPointer = new FloatPointer(1);
      r2yPointer.put(0, 0.0f);

      FloatPointer r2zPointer = new FloatPointer(1);
      r2zPointer.put(0, 1.0f);

      FloatPointer txPointer = new FloatPointer(1);
      txPointer.put(0, 4.0f);

      FloatPointer tyPointer = new FloatPointer(1);
      tyPointer.put(0, 5.0f);

      FloatPointer tzPointer = new FloatPointer(1);
      tzPointer.put(0, 6.0f);

      _cl_mem pxPointerObject = openCLManager.createBufferObject(4, pxPointer);
      _cl_mem pyPointerObject = openCLManager.createBufferObject(4, pyPointer);
      _cl_mem pzPointerObject = openCLManager.createBufferObject(4, pzPointer);
      _cl_mem r0xPointerObject = openCLManager.createBufferObject(4, r0xPointer);
      _cl_mem r0yPointerObject = openCLManager.createBufferObject(4, r0yPointer);
      _cl_mem r0zPointerObject = openCLManager.createBufferObject(4, r0zPointer);
      _cl_mem r1xPointerObject = openCLManager.createBufferObject(4, r1xPointer);
      _cl_mem r1yPointerObject = openCLManager.createBufferObject(4, r1yPointer);
      _cl_mem r1zPointerObject = openCLManager.createBufferObject(4, r1zPointer);
      _cl_mem r2xPointerObject = openCLManager.createBufferObject(4, r2xPointer);
      _cl_mem r2yPointerObject = openCLManager.createBufferObject(4, r2yPointer);
      _cl_mem r2zPointerObject = openCLManager.createBufferObject(4, r2zPointer);
      _cl_mem txPointerObject = openCLManager.createBufferObject(4, txPointer);
      _cl_mem tyPointerObject = openCLManager.createBufferObject(4, tyPointer);
      _cl_mem tzPointerObject = openCLManager.createBufferObject(4, tzPointer);

      FloatPointer resultPointer = new FloatPointer(3);
      _cl_mem resultPointerObject = openCLManager.createBufferObject(4 * 3, resultPointer);

      openCLManager.enqueueWriteBuffer(pxPointerObject, 4, pxPointer);
      openCLManager.enqueueWriteBuffer(pyPointerObject, 4, pyPointer);
      openCLManager.enqueueWriteBuffer(pzPointerObject, 4, pzPointer);
      openCLManager.enqueueWriteBuffer(r0xPointerObject, 4, r0xPointer);
      openCLManager.enqueueWriteBuffer(r0yPointerObject, 4, r0yPointer);
      openCLManager.enqueueWriteBuffer(r0zPointerObject, 4, r0zPointer);
      openCLManager.enqueueWriteBuffer(r1xPointerObject, 4, r1xPointer);
      openCLManager.enqueueWriteBuffer(r1yPointerObject, 4, r1yPointer);
      openCLManager.enqueueWriteBuffer(r1zPointerObject, 4, r1zPointer);
      openCLManager.enqueueWriteBuffer(r2xPointerObject, 4, r2xPointer);
      openCLManager.enqueueWriteBuffer(r2yPointerObject, 4, r2yPointer);
      openCLManager.enqueueWriteBuffer(r2zPointerObject, 4, r2zPointer);
      openCLManager.enqueueWriteBuffer(txPointerObject, 4, txPointer);
      openCLManager.enqueueWriteBuffer(tyPointerObject, 4, tyPointer);
      openCLManager.enqueueWriteBuffer(tzPointerObject, 4, tzPointer);

      openCLManager.setKernelArgument(openCLKernel, 0, pxPointerObject);
      openCLManager.setKernelArgument(openCLKernel, 1, pyPointerObject);
      openCLManager.setKernelArgument(openCLKernel, 2, pzPointerObject);
      openCLManager.setKernelArgument(openCLKernel, 3, r0xPointerObject);
      openCLManager.setKernelArgument(openCLKernel, 4, r0yPointerObject);
      openCLManager.setKernelArgument(openCLKernel, 5, r0zPointerObject);
      openCLManager.setKernelArgument(openCLKernel, 6, r1xPointerObject);
      openCLManager.setKernelArgument(openCLKernel, 7, r1yPointerObject);
      openCLManager.setKernelArgument(openCLKernel, 8, r1zPointerObject);
      openCLManager.setKernelArgument(openCLKernel, 9, r2xPointerObject);
      openCLManager.setKernelArgument(openCLKernel, 10, r2yPointerObject);
      openCLManager.setKernelArgument(openCLKernel, 11, r2zPointerObject);
      openCLManager.setKernelArgument(openCLKernel, 12, txPointerObject);
      openCLManager.setKernelArgument(openCLKernel, 13, tyPointerObject);
      openCLManager.setKernelArgument(openCLKernel, 14, tzPointerObject);

      openCLManager.setKernelArgument(openCLKernel, 15, resultPointerObject);

      openCLManager.execute1D(openCLKernel, 1);

      openCLManager.enqueueReadBuffer(resultPointerObject, 4 * 3, resultPointer);

      System.out.printf("Transformed Point: (%f, %f, %f)\n", resultPointer.get(0), resultPointer.get(1), resultPointer.get(2));

      openCLProgram.close();
      openCLManager.destroy();

      float[] returnData = new float[3];
      returnData[0] = resultPointer.get(0);
      returnData[1] = resultPointer.get(1);
      returnData[2] = resultPointer.get(2);

      pxPointer.close();
      pyPointer.close();
      pzPointer.close();
      r0xPointer.close();
      r0yPointer.close();
      r0zPointer.close();
      r1xPointer.close();
      r1yPointer.close();
      r1zPointer.close();
      r2xPointer.close();
      r2yPointer.close();
      r2zPointer.close();
      txPointer.close();
      tyPointer.close();
      tzPointer.close();
      pxPointerObject.close();
      pyPointerObject.close();
      pzPointerObject.close();
      r0xPointerObject.close();
      r0yPointerObject.close();
      r0zPointerObject.close();
      r1xPointerObject.close();
      r1yPointerObject.close();
      r1zPointerObject.close();
      r2xPointerObject.close();
      r2yPointerObject.close();
      r2zPointerObject.close();
      tzPointerObject.close();
      txPointerObject.close();
      tyPointerObject.close();
      tzPointerObject.close();

      return returnData;
   }
}
