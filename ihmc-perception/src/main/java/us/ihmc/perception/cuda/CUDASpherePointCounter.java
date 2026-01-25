package us.ihmc.perception.cuda;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.IntPointer;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.perception.RawImage;

import java.net.URL;

import static org.bytedeco.cuda.global.cudart.*;

/**
 * Counts depth image points that fall within a world-frame sphere.
 */
public class CUDASpherePointCounter implements AutoCloseable
{
   private static final int BLOCK_SIZE_XY = 16;

   private final CUDAProgram program;
   private final CUDAKernel kernel;
   private final CUstream_st stream;

   private final RigidBodyTransform depthToWorldTransform = new RigidBodyTransform();
   private final float[] transformArray = new float[16];

   private final FloatPointer transformPointer = new FloatPointer();
   private final IntPointer countPointer = new IntPointer();

   private int error;

   public CUDASpherePointCounter()
   {
      URL shapePointCounterURL = getClass().getResource("ShapePointCounter.cu");
      URL utilsURL = getClass().getResource("Utils.cu");
      URL perceptionUtilsURL = getClass().getResource("PerceptionUtils.cu");
      URL mathUtilsURL = getClass().getResource("MathUtils.cuh");

      try
      {
         program = new CUDAProgram(shapePointCounterURL, utilsURL, perceptionUtilsURL, mathUtilsURL);
         kernel = program.loadKernel("countPointsInSphere");
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

   public int countPointsInSphere(RawImage depthImage, Point3D32 sphereCenter, float sphereRadius)
   {
      if (depthImage.get() == null)
         return 0;

      depthToWorldTransform.set(depthImage.getTransformToWorld());
      depthToWorldTransform.get(transformArray);
      transformPointer.put(transformArray);

      dim3 blockSize = new dim3(BLOCK_SIZE_XY, BLOCK_SIZE_XY, 1);
      int gridSizeX = (depthImage.getWidth() + BLOCK_SIZE_XY - 1) / (BLOCK_SIZE_XY * 2);
      int gridSizeY = (depthImage.getHeight() + BLOCK_SIZE_XY - 1) / (BLOCK_SIZE_XY * 2);
      dim3 gridSize = new dim3(gridSizeX, gridSizeY, 1);

      kernel.withPointer(depthImage.getCUDADataPointer())
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
            .run(stream, gridSize, blockSize, 0);

      error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);
      int count = countPointer.get();

      blockSize.close();
      gridSize.close();
      depthImage.release();

      return count;
   }

   @Override
   public void close()
   {
      CUDATools.checkCUDAError(cudaFreeHost(transformPointer));
      CUDATools.checkCUDAError(cudaFreeHost(countPointer));

      kernel.close();
      program.close();
      CUDAStreamManager.releaseStream(stream);
   }
}
