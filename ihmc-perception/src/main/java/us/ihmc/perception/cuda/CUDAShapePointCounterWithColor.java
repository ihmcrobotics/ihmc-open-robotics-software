package us.ihmc.perception.cuda;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.LongPointer;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.imageMessage.PixelFormat;

import java.net.URL;

import static org.bytedeco.cuda.global.cudart.*;

/**
 * Counts depth image points that fall within a world-frame shape and computes average RGB values.
 */
public class CUDAShapePointCounterWithColor implements AutoCloseable
{
   private static final int BLOCK_SIZE_XY = 16;

   private final CUDAProgram program;
   private final CUDAKernel sphereKernel;
   private final CUDAKernel capsuleKernel;
   private final CUstream_st stream;

   private final RigidBodyTransform depthToWorldTransform = new RigidBodyTransform();
   private final float[] transformArray = new float[16];

   private final FloatPointer transformPointer = new FloatPointer();
   private final LongPointer countPointer = new LongPointer();
   private final LongPointer redSumPointer = new LongPointer();
   private final LongPointer greenSumPointer = new LongPointer();
   private final LongPointer blueSumPointer = new LongPointer();

   private float averageRed = 0.0f;
   private float averageGreen = 0.0f;
   private float averageBlue = 0.0f;
   private float averageHue = 0.0f;
   private float averageSaturation = 0.0f;
   private float averageValue = 0.0f;

   private int error;

   public CUDAShapePointCounterWithColor()
   {
      URL shapePointCounterURL = getClass().getResource("ShapePointCounterWithColor.cu");
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
      error = cudaMallocHost(redSumPointer, redSumPointer.sizeof());
      CUDATools.checkCUDAError(error);
      error = cudaMallocHost(greenSumPointer, greenSumPointer.sizeof());
      CUDATools.checkCUDAError(error);
      error = cudaMallocHost(blueSumPointer, blueSumPointer.sizeof());
      CUDATools.checkCUDAError(error);
   }

   public long countPointsInSphere(RawImage depthImage, RawImage colorImage, Tuple3DReadOnly sphereCenter, float sphereRadius)
   {
      if (depthImage.get() == null)
         return 0;
      if (colorImage.get() == null)
      {
         depthImage.release();
         return 0;
      }

      resetSumsAndAverages();
      ColorAccessInfo colorAccessInfo = getColorAccessInfo(colorImage.getPixelFormat());
      if (colorAccessInfo == null
          || depthImage.getWidth() != colorImage.getWidth()
          || depthImage.getHeight() != colorImage.getHeight())
      {
         depthImage.release();
         colorImage.release();
         return 0;
      }

      depthToWorldTransform.set(depthImage.getTransformToWorld());
      depthToWorldTransform.get(transformArray);
      transformPointer.put(transformArray);

      dim3 blockSize = new dim3(BLOCK_SIZE_XY, BLOCK_SIZE_XY, 1);
      int gridSizeX = (depthImage.getWidth() + BLOCK_SIZE_XY - 1) / (BLOCK_SIZE_XY * 2);
      int gridSizeY = (depthImage.getHeight() + BLOCK_SIZE_XY - 1) / (BLOCK_SIZE_XY * 2);
      dim3 gridSize = new dim3(gridSizeX, gridSizeY, 1);

      sphereKernel.withPointer(depthImage.getCUDADataPointer())
                  .withLong(depthImage.getGpuImageMat().step())
                  .withPointer(colorImage.getCUDADataPointer())
                  .withLong(colorImage.getGpuImageMat().step())
                  .withInt(depthImage.getWidth())
                  .withInt(depthImage.getHeight())
                  .withInt(colorAccessInfo.channels())
                  .withInt(colorAccessInfo.redOffset())
                  .withInt(colorAccessInfo.greenOffset())
                  .withInt(colorAccessInfo.blueOffset())
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
                  .withPointer(redSumPointer)
                  .withPointer(greenSumPointer)
                  .withPointer(blueSumPointer)
                  .run(stream, gridSize, blockSize, BLOCK_SIZE_XY * BLOCK_SIZE_XY * Long.BYTES);

      error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);

      long count = countPointer.get();
      updateAverages(count);

      blockSize.close();
      gridSize.close();
      depthImage.release();
      colorImage.release();

      return count;
   }

   public long countPointsInCapsule(RawImage depthImage, RawImage colorImage, Tuple3DReadOnly pointA, Tuple3DReadOnly pointB, float radius)
   {
      if (depthImage.get() == null)
         return 0;
      if (colorImage.get() == null)
      {
         depthImage.release();
         return 0;
      }

      resetSumsAndAverages();
      ColorAccessInfo colorAccessInfo = getColorAccessInfo(colorImage.getPixelFormat());
      if (colorAccessInfo == null
          || depthImage.getWidth() != colorImage.getWidth()
          || depthImage.getHeight() != colorImage.getHeight())
      {
         depthImage.release();
         colorImage.release();
         return 0;
      }

      depthToWorldTransform.set(depthImage.getTransformToWorld());
      depthToWorldTransform.get(transformArray);
      transformPointer.put(transformArray);

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
                  .withPointer(colorImage.getCUDADataPointer())
                  .withLong(colorImage.getGpuImageMat().step())
                  .withInt(depthImage.getWidth())
                  .withInt(depthImage.getHeight())
                  .withInt(colorAccessInfo.channels())
                  .withInt(colorAccessInfo.redOffset())
                  .withInt(colorAccessInfo.greenOffset())
                  .withInt(colorAccessInfo.blueOffset())
                  .withFloat(depthImage.getFocalLengthX())
                  .withFloat(depthImage.getFocalLengthY())
                  .withFloat(depthImage.getPrincipalPointX())
                  .withFloat(depthImage.getPrincipalPointY())
                  .withFloat(depthImage.getDepthDiscretization())
                  .withPointer(transformPointer)
                  .withPointer(pointsDevicePointer)
                  .withFloat(radius)
                  .withPointer(countPointer)
                  .withPointer(redSumPointer)
                  .withPointer(greenSumPointer)
                  .withPointer(blueSumPointer)
                  .run(stream, gridSize, blockSize, BLOCK_SIZE_XY * BLOCK_SIZE_XY * Long.BYTES);

      cudaFreeAsync(pointsDevicePointer, stream);

      error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);

      long count = countPointer.get();
      updateAverages(count);

      pointsDevicePointer.close();
      pointsHostPointer.close();
      blockSize.close();
      gridSize.close();
      depthImage.release();
      colorImage.release();

      return count;
   }

   public float getAverageRed()
   {
      return averageRed;
   }

   public float getAverageGreen()
   {
      return averageGreen;
   }

   public float getAverageBlue()
   {
      return averageBlue;
   }

   public float getAverageHue()
   {
      return averageHue;
   }

   public float getAverageSaturation()
   {
      return averageSaturation;
   }

   public float getAverageValue()
   {
      return averageValue;
   }

   private void resetSumsAndAverages()
   {
      countPointer.put(0);
      redSumPointer.put(0);
      greenSumPointer.put(0);
      blueSumPointer.put(0);
      averageRed = 0.0f;
      averageGreen = 0.0f;
      averageBlue = 0.0f;
      averageHue = 0.0f;
      averageSaturation = 0.0f;
      averageValue = 0.0f;
   }

   private void updateAverages(long count)
   {
      if (count <= 0)
         return;

      averageRed = (float) (redSumPointer.get() / (double) count);
      averageGreen = (float) (greenSumPointer.get() / (double) count);
      averageBlue = (float) (blueSumPointer.get() / (double) count);

      float red = averageRed / 255.0f;
      float green = averageGreen / 255.0f;
      float blue = averageBlue / 255.0f;

      float max = Math.max(red, Math.max(green, blue));
      float min = Math.min(red, Math.min(green, blue));
      float delta = max - min;

      averageValue = max;
      averageSaturation = max == 0.0f ? 0.0f : delta / max;

      if (delta == 0.0f)
         averageHue = 0.0f;
      else if (max == red)
         averageHue = 60.0f * (((green - blue) / delta) % 6.0f);
      else if (max == green)
         averageHue = 60.0f * (((blue - red) / delta) + 2.0f);
      else
         averageHue = 60.0f * (((red - green) / delta) + 4.0f);

      if (averageHue < 0.0f)
         averageHue += 360.0f;
   }

   private ColorAccessInfo getColorAccessInfo(PixelFormat pixelFormat)
   {
      return switch (pixelFormat)
      {
         case BGR8 -> new ColorAccessInfo(3, 2, 1, 0);
         case BGRA8 -> new ColorAccessInfo(4, 2, 1, 0);
         case RGB8 -> new ColorAccessInfo(3, 0, 1, 2);
         case RGBA8 -> new ColorAccessInfo(4, 0, 1, 2);
         default -> null;
      };
   }

   @Override
   public void close()
   {
      CUDATools.checkCUDAError(cudaFreeHost(transformPointer));
      CUDATools.checkCUDAError(cudaFreeHost(countPointer));
      CUDATools.checkCUDAError(cudaFreeHost(redSumPointer));
      CUDATools.checkCUDAError(cudaFreeHost(greenSumPointer));
      CUDATools.checkCUDAError(cudaFreeHost(blueSumPointer));

      sphereKernel.close();
      capsuleKernel.close();
      program.close();
      CUDAStreamManager.releaseStream(stream);
   }

   private record ColorAccessInfo(int channels, int redOffset, int greenOffset, int blueOffset)
   {
   }
}
