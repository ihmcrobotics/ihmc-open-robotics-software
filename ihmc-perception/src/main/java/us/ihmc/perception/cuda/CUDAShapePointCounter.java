package us.ihmc.perception.cuda;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.IntPointer;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.perception.RawImage;

import java.net.URL;

import static org.bytedeco.cuda.global.cudart.*;

/**
 * Counts depth image points that fall within a world-frame sphere.
 */
public class CUDAShapePointCounter implements AutoCloseable
{
   private static final int BLOCK_SIZE_XY = 16;

   private final CUDAProgram program;
   private final CUDAKernel sphereKernel;
   private final CUDAKernel capsuleKernel;
   private final CUstream_st stream;

   private final RigidBodyTransform depthToWorldTransform = new RigidBodyTransform();
   private final float[] transformArray = new float[16];

   private final FloatPointer transformPointer = new FloatPointer();
   private final IntPointer countPointer = new IntPointer();

   private int error;

   public CUDAShapePointCounter()
   {
      URL shapePointCounterURL = getClass().getResource("ShapePointCounter.cu");
      URL utilsURL = getClass().getResource("Utils.cu");
      URL perceptionUtilsURL = getClass().getResource("PerceptionUtils.cu");
      URL mathUtilsURL = getClass().getResource("MathUtils.cuh");

      try
      {
         program = new CUDAProgram(shapePointCounterURL, utilsURL, perceptionUtilsURL, mathUtilsURL);
         sphereKernel = program.loadKernel("countPointsInSphere");
         capsuleKernel = program.loadKernel("countPointsInCapsule");
      }
      catch (Exception e)
      {
         throw new RuntimeException(e);
      }

      stream = CUDAStreamManager.getStream();

      error = cudaMallocHost(transformPointer, 16L * transformPointer.sizeof());
      CUDATools.checkCUDAError(error);
      error = cudaMallocHost(countPointer, countPointer.sizeof());
      CUDATools.checkCUDAError(error);
   }

   public long countPointsInSphere(RawImage depthImage, Tuple3DReadOnly sphereCenter, float sphereRadius)
   {
      if (depthImage.get() == null)
         return 0;

      depthToWorldTransform.set(depthImage.getTransformToWorld());
      depthToWorldTransform.get(transformArray);
      transformPointer.put(transformArray);
      countPointer.put(0);

      dim3 blockSize = new dim3(BLOCK_SIZE_XY, BLOCK_SIZE_XY, 1);
      int gridSizeX = (depthImage.getWidth() + BLOCK_SIZE_XY - 1) / (BLOCK_SIZE_XY * 2);
      int gridSizeY = (depthImage.getHeight() + BLOCK_SIZE_XY - 1) / (BLOCK_SIZE_XY * 2);
      dim3 gridSize = new dim3(gridSizeX, gridSizeY, 1);

      sphereKernel.withPointer(depthImage.getCUDADataPointer())
                  .withLong(depthImage.getGpuImageMat().step())
                  .withInt(depthImage.getWidth())
                  .withInt(depthImage.getHeight())
                  .withFloat(depthImage.getFocalLengthX())
                  .withFloat(depthImage.getFocalLengthY())
                  .withFloat(depthImage.getPrincipalPointX())
                  .withFloat(depthImage.getPrincipalPointY())
                  .withFloat(depthImage.getDepthDiscretization())
                  .withPointer(transformPointer)
                  .withFloat(sphereCenter.getX32())
                  .withFloat(sphereCenter.getY32())
                  .withFloat(sphereCenter.getZ32())
                  .withFloat(sphereRadius)
                  .withPointer(countPointer)
                  .run(stream, gridSize, blockSize, BLOCK_SIZE_XY * BLOCK_SIZE_XY * Integer.BYTES);

      error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);

      blockSize.close();
      gridSize.close();
      depthImage.release();

      return Integer.toUnsignedLong(countPointer.get());
   }

   public long countPointsInCapsule(RawImage depthImage, Tuple3DReadOnly pointA, Tuple3DReadOnly pointB, float radius)
   {
      if (depthImage.get() == null)
         return 0;

      depthToWorldTransform.set(depthImage.getTransformToWorld());
      depthToWorldTransform.get(transformArray);
      transformPointer.put(transformArray);
      countPointer.put(0);

      FloatPointer pointsHostPointer = new FloatPointer(pointA.getX32(), pointA.getY32(), pointA.getZ32(), pointB.getX32(), pointB.getY32(), pointB.getZ32());
      FloatPointer pointsDevicePointer = new FloatPointer();
      CUDATools.mallocAsync(pointsDevicePointer, 6, stream);
      CUDATools.memcpyAsync(pointsDevicePointer, pointsHostPointer, 6, stream);

      dim3 blockSize = new dim3(BLOCK_SIZE_XY, BLOCK_SIZE_XY, 1);
      int gridSizeX = (depthImage.getWidth() + BLOCK_SIZE_XY - 1) / (BLOCK_SIZE_XY * 2);
      int gridSizeY = (depthImage.getHeight() + BLOCK_SIZE_XY - 1) / (BLOCK_SIZE_XY * 2);
      dim3 gridSize = new dim3(gridSizeX, gridSizeY, 1);

      capsuleKernel.withPointer(depthImage.getCUDADataPointer())
                  .withLong(depthImage.getGpuImageMat().step())
                  .withInt(depthImage.getWidth())
                  .withInt(depthImage.getHeight())
                  .withFloat(depthImage.getFocalLengthX())
                  .withFloat(depthImage.getFocalLengthY())
                  .withFloat(depthImage.getPrincipalPointX())
                  .withFloat(depthImage.getPrincipalPointY())
                  .withFloat(depthImage.getDepthDiscretization())
                  .withPointer(transformPointer)
                  .withPointer(pointsDevicePointer)
                  .withFloat(radius)
                  .withPointer(countPointer)
                  .run(stream, gridSize, blockSize, BLOCK_SIZE_XY * BLOCK_SIZE_XY * Integer.BYTES);

      cudaFreeAsync(pointsDevicePointer, stream);

      error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);

      pointsDevicePointer.close();
      pointsHostPointer.close();
      blockSize.close();
      gridSize.close();
      depthImage.release();

      return Integer.toUnsignedLong(countPointer.get());
   }

   @Override
   public void close()
   {
      CUDATools.checkCUDAError(cudaFreeHost(transformPointer));
      CUDATools.checkCUDAError(cudaFreeHost(countPointer));

      sphereKernel.close();
      capsuleKernel.close();
      program.close();
      CUDAStreamManager.releaseStream(stream);
   }
}
