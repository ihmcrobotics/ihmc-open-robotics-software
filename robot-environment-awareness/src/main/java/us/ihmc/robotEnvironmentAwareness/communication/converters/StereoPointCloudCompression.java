package us.ihmc.robotEnvironmentAwareness.communication.converters;

import static us.ihmc.jOctoMap.tools.OcTreeKeyTools.computeCenterOffsetKey;

import java.awt.Color;
import java.nio.ByteBuffer;
import java.nio.CharBuffer;
import java.nio.IntBuffer;
import java.util.function.IntConsumer;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import gnu.trove.list.array.TByteArrayList;
import net.jpountz.lz4.LZ4Exception;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.tools.compression.LZ4CompressionImplementation;

/**
 * This class should be used to create and unpack a {@link StereoVisionPointCloudMessage}.
 */
public class StereoPointCloudCompression
{
   private static final int OCTREE_DEPTH = 16;
   private static final double OCTREE_RESOLUTION_TO_SIZE_RATIO = Math.pow(2.0, OCTREE_DEPTH) - 1;
   private static final ThreadLocal<LZ4CompressionImplementation> compressorThreadLocal = ThreadLocal.withInitial(LZ4CompressionImplementation::new);

   private static final int centerOffsetKey = computeCenterOffsetKey(OCTREE_DEPTH);

   public static class CompressionIntermediateVariablesPackage
   {
      private int numberOfPoints = -1;
      private ByteBuffer rawPointCloudByteBuffer;
      private ByteBuffer rawColorByteBuffer;
      private CharBuffer rawPointCloudCharBuffer;
      private IntBuffer rawColorIntBuffer;
      private final LZ4CompressionImplementation compressor = new LZ4CompressionImplementation();

      private ByteBuffer compressedPointCloudByteBuffer;
      private ByteBuffer compressedColorByteBuffer;
      private final StereoVisionPointCloudMessage message = new StereoVisionPointCloudMessage();

      public CompressionIntermediateVariablesPackage()
      {
      }

      private void initialize(int numberOfPoints)
      {
         int pointCloudByteBufferSize = computePointByteBufferSize(numberOfPoints);
         int colorByteBufferSize = computeColorByteBufferSize(numberOfPoints);

         if (this.numberOfPoints == -1 || this.numberOfPoints < numberOfPoints)
         {
            rawPointCloudByteBuffer = ByteBuffer.allocateDirect(pointCloudByteBufferSize);
            rawColorByteBuffer = ByteBuffer.allocateDirect(colorByteBufferSize);

            rawPointCloudCharBuffer = rawPointCloudByteBuffer.asCharBuffer();
            rawColorIntBuffer = rawColorByteBuffer.asIntBuffer();

            // LZ4 compression apparently needs some additional room to do its thing.
            compressedPointCloudByteBuffer = ByteBuffer.allocate(pointCloudByteBufferSize * 2);
            compressedColorByteBuffer = ByteBuffer.allocate(colorByteBufferSize * 2);
         }
         else
         {
            compressedPointCloudByteBuffer.clear();
            compressedColorByteBuffer.clear();

            rawPointCloudByteBuffer.limit(pointCloudByteBufferSize);
            rawColorByteBuffer.limit(colorByteBufferSize);

            rawPointCloudCharBuffer.limit(numberOfPoints * 3);
            rawColorIntBuffer.limit(numberOfPoints);

            rawPointCloudByteBuffer.position(0);
            rawColorByteBuffer.position(0);

            rawPointCloudCharBuffer.position(0);
            rawColorIntBuffer.position(0);
         }

         this.numberOfPoints = numberOfPoints;
         message.setSequenceId(-1);
         message.setTimestamp(-1);
         message.getSensorPosition().setToZero();
         message.getSensorOrientation().setToZero();
         message.setResolution(0.0);
         message.setNumberOfPoints(0);
         message.getPointCloud().reset();
         message.getColors().reset();
      }
   }

   public static int computeColorByteBufferSize(int numberOfPoints)
   {
      return numberOfPoints * 4;
   }

   public static int computePointByteBufferSize(int numberOfPoints)
   {
      return numberOfPoints * 3 * 2;
   }

   public static StereoVisionPointCloudMessage compressPointCloudFast(long timestamp,
                                                                      PointAccessor pointAccessor,
                                                                      ColorAccessor colorAccessor,
                                                                      int numberOfPoints,
                                                                      double minimumResolution,
                                                                      CompressionIntermediateVariablesPackage variablesPackage)
   {
      BoundingBox3D boundingBox = new BoundingBox3D();
      boundingBox.setToNaN();

      // 1- First apply the filters if any and compute the bounding box of the point-cloud.
      for (int i = 0; i < numberOfPoints; i++)
      {
         boundingBox.updateToIncludePoint(pointAccessor.getX(i), pointAccessor.getY(i), pointAccessor.getZ(i));
      }

      // 2- Store the pointcloud in an octree.
      // The resolution of the octree adapts to the size of the bounding box.
      // Duplicate node are filtered out.
      // The output of this is the key of the occupied leaves of the octree with can be represented a 3*int which is half the memory w.r.t. to a Point3D32.
      double sizeX = boundingBox.getMaxX() - boundingBox.getMinX();
      double sizeY = boundingBox.getMaxY() - boundingBox.getMinY();
      double sizeZ = boundingBox.getMaxZ() - boundingBox.getMinZ();
      float centerX = 0.5f * (float) (boundingBox.getMaxX() + boundingBox.getMinX());
      float centerY = 0.5f * (float) (boundingBox.getMaxY() + boundingBox.getMinY());
      float centerZ = 0.5f * (float) (boundingBox.getMaxZ() + boundingBox.getMinZ());
      double octreeSize = EuclidCoreTools.max(sizeX, sizeY, sizeZ);
      double octreeResolution = Math.max(minimumResolution, octreeSize / OCTREE_RESOLUTION_TO_SIZE_RATIO);

      // 3- Last step: We pack the data in byte buffer and use LZ4CompressionImplementation to compress it.
      ByteBuffer rawPointCloudByteBuffer;
      ByteBuffer rawColorByteBuffer;
      CharBuffer rawPointCloudCharBuffer;
      IntBuffer rawColorIntBuffer;
      ByteBuffer compressedPointCloudByteBuffer;
      ByteBuffer compressedColorByteBuffer;
      LZ4CompressionImplementation compressor;

      if (variablesPackage != null)
      {
         variablesPackage.initialize(numberOfPoints);
         rawPointCloudByteBuffer = variablesPackage.rawPointCloudByteBuffer;
         rawColorByteBuffer = variablesPackage.rawColorByteBuffer;
         rawPointCloudCharBuffer = variablesPackage.rawPointCloudCharBuffer;
         rawColorIntBuffer = variablesPackage.rawColorIntBuffer;
         compressedPointCloudByteBuffer = variablesPackage.compressedPointCloudByteBuffer;
         compressedColorByteBuffer = variablesPackage.compressedColorByteBuffer;
         compressor = variablesPackage.compressor;
      }
      else
      {
         int pointCloudByteBufferSize = computePointByteBufferSize(numberOfPoints);
         int colorByteBufferSize = computeColorByteBufferSize(numberOfPoints);
         rawPointCloudByteBuffer = ByteBuffer.allocate(pointCloudByteBufferSize);
         rawColorByteBuffer = ByteBuffer.allocate(colorByteBufferSize);
         rawPointCloudCharBuffer = rawPointCloudByteBuffer.asCharBuffer();
         rawColorIntBuffer = rawColorByteBuffer.asIntBuffer();
         compressedPointCloudByteBuffer = ByteBuffer.allocate(pointCloudByteBufferSize * 2);
         compressedColorByteBuffer = ByteBuffer.allocate(colorByteBufferSize * 2);
         compressor = compressorThreadLocal.get();
      }

      for (int i = 0; i < numberOfPoints; i++)
      {
         int keyX = (int) ((pointAccessor.getX(i) - centerX) / octreeResolution) + centerOffsetKey;
         int keyY = (int) ((pointAccessor.getY(i) - centerY) / octreeResolution) + centerOffsetKey;
         int keyZ = (int) ((pointAccessor.getZ(i) - centerZ) / octreeResolution) + centerOffsetKey;
         rawPointCloudCharBuffer.put((char) keyX);
         rawPointCloudCharBuffer.put((char) keyY);
         rawPointCloudCharBuffer.put((char) keyZ);

         rawColorIntBuffer.put(colorAccessor.getRGB(i));
      }

      try
      {
         compressor.compress(rawPointCloudByteBuffer, compressedPointCloudByteBuffer);
      }
      catch (LZ4Exception e)
      {
         // TODO See why the compression would fail and how to fix it.
         e.printStackTrace();
         return null;
      }
      try
      {
         compressor.compress(rawColorByteBuffer, compressedColorByteBuffer);
      }
      catch (LZ4Exception e)
      {
         e.printStackTrace();
         return null;
      }

      StereoVisionPointCloudMessage message;

      if (variablesPackage != null)
         message = variablesPackage.message;
      else
         message = new StereoVisionPointCloudMessage();

      compressedPointCloudByteBuffer.flip();
      message.getPointCloud().add(compressedPointCloudByteBuffer.array());

      compressedColorByteBuffer.flip();
      message.getColors().add(compressedColorByteBuffer.array());
      message.setTimestamp(timestamp);
      message.setSensorPoseConfidence(1.0);
      boundingBox.getCenterPoint(message.getPointCloudCenter());
      message.setResolution(octreeResolution);

      message.setNumberOfPoints(numberOfPoints);

      return message;
   }

   /**
    * Compresses the given point-cloud by doing the following:
    * <ul>
    * <li>Filters out the points according to the given {@code filter} if provided.
    * <li>Stores the points into an octree and replaces the use Cartesian coordinates (3 * 32bits) for
    * the octree key system (3 * 16bits).
    * <li>Use {@link LZ4CompressionImplementation} as a final lossless compression pass.
    * </ul>
    */
   public static StereoVisionPointCloudMessage compressPointCloud(long timestamp,
                                                                  Point3DReadOnly[] pointCloud,
                                                                  int[] colors,
                                                                  int numberOfPoints,
                                                                  double minimumResolution,
                                                                  ScanPointFilter filter)
   {
      BoundingBox3D boundingBox = new BoundingBox3D();
      boundingBox.setToNaN();

      // 1- First apply the filters if any and compute the bounding box of the point-cloud.
      if (filter != null)
      {
         int filteredIndex = 0;
         Point3DReadOnly[] filteredPointCloud = new Point3D[numberOfPoints];
         int[] filteredColors = new int[numberOfPoints];

         for (int i = 0; i < numberOfPoints; i++)
         {
            Point3DReadOnly scanPoint = pointCloud[i];
            int color = colors[i];

            if (filter.test(i, scanPoint))
            {
               filteredPointCloud[filteredIndex] = scanPoint;
               filteredColors[filteredIndex] = color;
               boundingBox.updateToIncludePoint(scanPoint);
               filteredIndex++;
            }
         }

         numberOfPoints = filteredIndex;
         pointCloud = filteredPointCloud;
         colors = filteredColors;
      }
      else
      {
         for (Point3DReadOnly scanPoint : pointCloud)
         {
            boundingBox.updateToIncludePoint(scanPoint);
         }
      }

      return compressPointCloudFast(timestamp, PointAccessor.wrap(pointCloud), ColorAccessor.wrapRGB(colors), numberOfPoints, minimumResolution, null);
   }

   public static Point3D32[] decompressPointCloudToArray32(StereoVisionPointCloudMessage message)
   {
      return decompressPointCloudToArray32(message.getPointCloud(), message.getPointCloudCenter(), message.getResolution(), message.getNumberOfPoints());
   }

   public static Point3D32[] decompressPointCloudToArray32(TByteArrayList compressedPointCloud, Point3D center, double resolution, int numberOfPoints)
   {
      Point3D32[] pointCloud = new Point3D32[numberOfPoints];
      decompressPointCloud(compressedPointCloud, center, resolution, numberOfPoints, new PointCoordinateConsumer()
      {
         int index = 0;

         @Override
         public void accept(double x, double y, double z)
         {
            pointCloud[index] = new Point3D32((float) x, (float) y, (float) z);
            index++;
         }
      });
      return pointCloud;
   }

   public static Point3D[] decompressPointCloudToArray(StereoVisionPointCloudMessage message)
   {
      return decompressPointCloudToArray(message.getPointCloud(), message.getPointCloudCenter(), message.getResolution(), message.getNumberOfPoints());
   }

   public static Point3D[] decompressPointCloudToArray(TByteArrayList compressedPointCloud, Point3D center, double resolution, int numberOfPoints)
   {
      Point3D[] pointCloud = new Point3D[numberOfPoints];
      decompressPointCloud(compressedPointCloud, center, resolution, numberOfPoints, new PointCoordinateConsumer()
      {
         int index = 0;

         @Override
         public void accept(double x, double y, double z)
         {
            pointCloud[index] = new Point3D((float) x, (float) y, (float) z);
            index++;
         }
      });
      return pointCloud;
   }

   public static void decompressPointCloud(StereoVisionPointCloudMessage message, PointCoordinateConsumer pointCoordinateConsumer)
   {
      decompressPointCloud(message.getPointCloud(),
                           message.getPointCloudCenter(),
                           message.getResolution(),
                           message.getNumberOfPoints(),
                           pointCoordinateConsumer);
   }

   public static void decompressPointCloud(TByteArrayList compressedPointCloud,
                                           Point3D center,
                                           double resolution,
                                           int numberOfPoints,
                                           PointCoordinateConsumer pointCoordinateConsumer)
   {
      ByteBuffer compressedPointCloudByteBuffer = ByteBuffer.wrap(compressedPointCloud.toArray());
      ByteBuffer decompressedPointCloudByteBuffer = ByteBuffer.allocate(computePointByteBufferSize(numberOfPoints));
      compressorThreadLocal.get().decompress(compressedPointCloudByteBuffer, decompressedPointCloudByteBuffer, computePointByteBufferSize(numberOfPoints));
      decompressedPointCloudByteBuffer.flip();
      CharBuffer pointCloudCharBuffer = decompressedPointCloudByteBuffer.asCharBuffer();

      for (int i = 0; i < numberOfPoints; i++)
      {
         double x = ((int) pointCloudCharBuffer.get() - centerOffsetKey + 0.5) * resolution;
         double y = ((int) pointCloudCharBuffer.get() - centerOffsetKey + 0.5) * resolution;
         double z = ((int) pointCloudCharBuffer.get() - centerOffsetKey + 0.5) * resolution;
         pointCoordinateConsumer.accept(x + center.getX(), y + center.getY(), z + center.getZ());
      }
   }

   public static Color[] decompressColorsToAWTColorArray(StereoVisionPointCloudMessage message)
   {
      return decompressColorsToAWTColorArray(message.getColors(), message.getNumberOfPoints());
   }

   public static Color[] decompressColorsToAWTColorArray(TByteArrayList compressedColors, int numberOfPoints)
   {
      Color[] colors = new Color[numberOfPoints];

      decompressColors(compressedColors, numberOfPoints, new IntConsumer()
      {
         int index = 0;

         @Override
         public void accept(int value)
         {
            colors[index] = new Color(value);
            index++;
         }
      });

      return colors;
   }

   public static int[] decompressColorsToIntArray(StereoVisionPointCloudMessage message)
   {
      return decompressColorsToIntArray(message.getColors(), message.getNumberOfPoints());
   }

   public static int[] decompressColorsToIntArray(TByteArrayList compressedColors, int numberOfPoints)
   {
      int[] colors = new int[numberOfPoints];

      decompressColors(compressedColors, numberOfPoints, new IntConsumer()
      {
         int index = 0;

         @Override
         public void accept(int value)
         {
            colors[index] = value;
            index++;
         }
      });

      return colors;
   }

   public static void decompressColors(StereoVisionPointCloudMessage message, IntConsumer colorConsumer)
   {
      decompressColors(message.getColors(), message.getNumberOfPoints(), colorConsumer);
   }

   public static void decompressColors(TByteArrayList compressedColors, int numberOfPoints, IntConsumer colorConsumer)
   {
      ByteBuffer compressedColorByteBuffer = ByteBuffer.wrap(compressedColors.toArray());
      int colorByteBufferSize = computeColorByteBufferSize(numberOfPoints);
      ByteBuffer decompressedColorByteBuffer = ByteBuffer.allocate(colorByteBufferSize);
      compressorThreadLocal.get().decompress(compressedColorByteBuffer, decompressedColorByteBuffer, colorByteBufferSize);
      decompressedColorByteBuffer.flip();
      IntBuffer colorIntBuffer = decompressedColorByteBuffer.asIntBuffer();

      for (int i = 0; i < numberOfPoints; i++)
      {
         colorConsumer.accept(colorIntBuffer.get());
      }
   }

   public static interface PointAccessor
   {
      float getX(int pointIndex);

      float getY(int pointIndex);

      float getZ(int pointIndex);

      static PointAccessor wrap(float[] buffer)
      {
         return new PointAccessor()
         {
            @Override
            public float getX(int pointIndex)
            {
               return buffer[3 * pointIndex];
            }

            @Override
            public float getY(int pointIndex)
            {
               return buffer[3 * pointIndex + 1];
            }

            @Override
            public float getZ(int pointIndex)
            {
               return buffer[3 * pointIndex + 2];
            }
         };
      }

      static PointAccessor wrap(Point3DReadOnly[] pointcloud)
      {
         return new PointAccessor()
         {
            @Override
            public float getX(int pointIndex)
            {
               return pointcloud[pointIndex].getX32();
            }

            @Override
            public float getY(int pointIndex)
            {
               return pointcloud[pointIndex].getY32();
            }

            @Override
            public float getZ(int pointIndex)
            {
               return pointcloud[pointIndex].getZ32();
            };
         };
      }
   }

   public static interface ColorAccessor
   {
      int getRGB(int pointIndex);

      public static ColorAccessor wrapRGB(int[] rgbBuffer)
      {
         return new ColorAccessor()
         {
            @Override
            public int getRGB(int pointIndex)
            {
               return rgbBuffer[pointIndex];
            }
         };
      }

      public static ColorAccessor wrapRGB(byte[] rgbBuffer)
      {
         return new ColorAccessor()
         {
            @Override
            public int getRGB(int pointIndex)
            {
               int index = pointIndex * 3;
               byte r = rgbBuffer[index++];
               byte g = rgbBuffer[index++];
               byte b = rgbBuffer[index];
               return ((r & 0xFF) << 16) | ((g & 0xFF) << 8) | ((b & 0xFF) << 0);
            }
         };
      }
   }

   public static interface PointCoordinateConsumer
   {
      void accept(double x, double y, double z);
   }
}
