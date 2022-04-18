package us.ihmc.robotEnvironmentAwareness.communication.converters;

import java.awt.Color;
import java.nio.ByteBuffer;
import java.nio.IntBuffer;
import java.nio.ShortBuffer;
import java.util.function.IntConsumer;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import gnu.trove.list.array.TByteArrayList;
import net.jpountz.lz4.LZ4Exception;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.tools.compression.LZ4CompressionImplementation;

/**
 * This class should be used to create and unpack a {@link StereoVisionPointCloudMessage}.
 */
public class StereoPointCloudCompression
{
   private static final boolean USE_LZ4_COMPRESSION_DEFAULT = false;

   private static final int OCTREE_DEPTH = 16;
   private static final double RESOLUTION_TO_SIZE_RATIO = Math.pow(2.0, OCTREE_DEPTH) - 1;
   private static final ThreadLocal<LZ4CompressionImplementation> compressorThreadLocal = ThreadLocal.withInitial(LZ4CompressionImplementation::new);

   public static class CompressionIntermediateVariablesPackage
   {
      private int numberOfPoints = -1;
      private ByteBuffer rawPointCloudByteBuffer;
      private ByteBuffer rawColorByteBuffer;
      private ShortBuffer rawPointCloudShortBuffer;
      private IntBuffer rawColorIntBuffer;
      private final LZ4CompressionImplementation compressor = new LZ4CompressionImplementation();

      private ByteBufferedSequence byteBufferedPointCloud;
      private ByteBufferedSequence byteBufferedColors;
      private ByteBuffer compressedPointCloudByteBuffer;
      private ByteBuffer compressedColorByteBuffer;
      private final StereoVisionPointCloudMessage message = new StereoVisionPointCloudMessage();

      private int pointCloudByteBufferSize;
      private int colorByteBufferSize;

      public CompressionIntermediateVariablesPackage()
      {
      }

      private void initialize(int numberOfPoints)
      {
         pointCloudByteBufferSize = computePointByteBufferSize(numberOfPoints);
         colorByteBufferSize = computeColorByteBufferSize(numberOfPoints);

         if (this.numberOfPoints == -1 || this.numberOfPoints < numberOfPoints)
         {
            rawPointCloudByteBuffer = ByteBuffer.allocate(pointCloudByteBufferSize);
            rawColorByteBuffer = ByteBuffer.allocate(colorByteBufferSize);

            rawPointCloudShortBuffer = rawPointCloudByteBuffer.asShortBuffer();
            rawColorIntBuffer = rawColorByteBuffer.asIntBuffer();

            // LZ4 compression apparently needs some additional room to do its thing.
            byteBufferedPointCloud = new ByteBufferedSequence(pointCloudByteBufferSize * 2, "type_9");
            byteBufferedColors = new ByteBufferedSequence(colorByteBufferSize * 2, "type_9");

            message.point_cloud_ = byteBufferedPointCloud;
            message.colors_ = byteBufferedColors;
            compressedPointCloudByteBuffer = byteBufferedPointCloud.byteBuffer;
            compressedColorByteBuffer = byteBufferedColors.byteBuffer;
         }
         else
         {
            compressedPointCloudByteBuffer.clear();
            compressedColorByteBuffer.clear();

            rawPointCloudByteBuffer.limit(pointCloudByteBufferSize);
            rawColorByteBuffer.limit(colorByteBufferSize);

            rawPointCloudShortBuffer.limit(numberOfPoints * 3);
            rawColorIntBuffer.limit(numberOfPoints);

            rawPointCloudByteBuffer.position(0);
            rawColorByteBuffer.position(0);

            rawPointCloudShortBuffer.position(0);
            rawColorIntBuffer.position(0);
         }

         this.numberOfPoints = numberOfPoints;
         clearMessage();
      }

      private void clearMessage()
      {
         message.setSequenceId(0);
         message.setTimestamp(0);
         message.getSensorPosition().setToZero();
         message.getSensorOrientation().setToZero();
         message.setResolution(0.0);
         message.setNumberOfPoints(0);
         message.getPointCloud().reset();
         message.getColors().reset();
      }
   }

   public static class ByteBufferedSequence extends us.ihmc.idl.IDLSequence.Byte
   {
      private ByteBuffer byteBuffer;

      public ByteBufferedSequence(int maxSize, String typeCode)
      {
         super(maxSize, typeCode);
         byteBuffer = ByteBuffer.wrap(this._data);
         reset();
      }

      @Override
      public void reset()
      {
         super.reset();
         byteBuffer.clear();
         byteBuffer.position(0);
      }

      public void setPosition(int position)
      {
         _pos = position;
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

   public static StereoVisionPointCloudMessage compressPointCloud(long timestamp,
                                                                  PointAccessor pointAccessor,
                                                                  ColorAccessor colorAccessor,
                                                                  int numberOfPoints,
                                                                  double minimumResolution,
                                                                  CompressionIntermediateVariablesPackage variablesPackage)
   {
      return compressPointCloud(timestamp, pointAccessor, colorAccessor, numberOfPoints, minimumResolution, USE_LZ4_COMPRESSION_DEFAULT, variablesPackage);
   }

   public static StereoVisionPointCloudMessage compressPointCloud(long timestamp,
                                                                  PointAccessor pointAccessor,
                                                                  ColorAccessor colorAccessor,
                                                                  int numberOfPoints,
                                                                  double minimumResolution,
                                                                  boolean useLZ4Compressor,
                                                                  CompressionIntermediateVariablesPackage variablesPackage)
   {
      return compressPointCloud(timestamp,
                                pointAccessor,
                                colorAccessor,
                                DiscretizationParameters.bbxCalculator(minimumResolution),
                                numberOfPoints,
                                useLZ4Compressor,
                                variablesPackage);
   }

   public static StereoVisionPointCloudMessage compressPointCloud(long timestamp,
                                                                  PointAccessor pointAccessor,
                                                                  ColorAccessor colorAccessor,
                                                                  DiscretizationParameters discretizationParameters,
                                                                  int numberOfPoints,
                                                                  CompressionIntermediateVariablesPackage variablesPackage)
   {
      return compressPointCloud(timestamp,
                                pointAccessor,
                                colorAccessor,
                                discretizationParameters,
                                numberOfPoints,
                                USE_LZ4_COMPRESSION_DEFAULT,
                                variablesPackage);
   }

   public static StereoVisionPointCloudMessage compressPointCloud(long timestamp,
                                                                  PointAccessor pointAccessor,
                                                                  ColorAccessor colorAccessor,
                                                                  DiscretizationParameters discretizationParameters,
                                                                  int numberOfPoints,
                                                                  boolean useLZ4Compressor,
                                                                  CompressionIntermediateVariablesPackage variablesPackage)
   {
      if (variablesPackage == null)
         variablesPackage = new CompressionIntermediateVariablesPackage();

      variablesPackage.initialize(numberOfPoints);

      StereoVisionPointCloudMessage message = variablesPackage.message;
      message.setTimestamp(timestamp);
      message.setSensorPoseConfidence(1.0);
      message.setNumberOfPoints(numberOfPoints);
      message.setLz4Compressed(useLZ4Compressor);

      discretizationParameters.update(pointAccessor, numberOfPoints);
      discretizationParameters.getPointCloudCenter(message.getPointCloudCenter());

      double centerX = message.getPointCloudCenter().getX();
      double centerY = message.getPointCloudCenter().getY();
      double centerZ = message.getPointCloudCenter().getZ();
      message.setResolution(discretizationParameters.getResolution());
      double invResolution = 1.0 / message.getResolution();

      if (!useLZ4Compressor)
      {
         ByteBuffer pointCloudByteBuffer = variablesPackage.compressedPointCloudByteBuffer;
         ByteBuffer colorByteBuffer = variablesPackage.compressedColorByteBuffer;

         for (int i = 0; i < numberOfPoints; i++)
         {
            pointCloudByteBuffer.putShort((short) ((pointAccessor.getX(i) - centerX) * invResolution));
            pointCloudByteBuffer.putShort((short) ((pointAccessor.getY(i) - centerY) * invResolution));
            pointCloudByteBuffer.putShort((short) ((pointAccessor.getZ(i) - centerZ) * invResolution));
            colorByteBuffer.put(colorAccessor.getRed(i));
            colorByteBuffer.put(colorAccessor.getGreen(i));
            colorByteBuffer.put(colorAccessor.getBlue(i));
         }

         variablesPackage.byteBufferedPointCloud.setPosition(variablesPackage.pointCloudByteBufferSize);
         variablesPackage.byteBufferedColors.setPosition(variablesPackage.colorByteBufferSize);
      }
      else
      {
         ByteBuffer rawPointCloudByteBuffer = variablesPackage.rawPointCloudByteBuffer;
         ByteBuffer rawColorByteBuffer = variablesPackage.rawColorByteBuffer;
         ShortBuffer rawPointCloudShortBuffer = variablesPackage.rawPointCloudShortBuffer;
         IntBuffer rawColorIntBuffer = variablesPackage.rawColorIntBuffer;
         ByteBuffer compressedPointCloudByteBuffer = variablesPackage.compressedPointCloudByteBuffer;
         ByteBuffer compressedColorByteBuffer = variablesPackage.compressedColorByteBuffer;
         LZ4CompressionImplementation compressor = variablesPackage.compressor;

         for (int i = 0; i < numberOfPoints; i++)
         {
            rawPointCloudShortBuffer.put((short) ((pointAccessor.getX(i) - centerX) * invResolution));
            rawPointCloudShortBuffer.put((short) ((pointAccessor.getY(i) - centerY) * invResolution));
            rawPointCloudShortBuffer.put((short) ((pointAccessor.getZ(i) - centerZ) * invResolution));
            rawColorIntBuffer.put(colorAccessor.getRGB(i));
         }

         int compressedPointCloudSize;
         try
         {
            compressedPointCloudSize = compressor.compress(rawPointCloudByteBuffer, compressedPointCloudByteBuffer);
         }
         catch (LZ4Exception e)
         {
            // TODO See why the compression would fail and how to fix it.
            e.printStackTrace();
            return null;
         }
         int compressedColorSize;
         try
         {
            compressedColorSize = compressor.compress(rawColorByteBuffer, compressedColorByteBuffer);
         }
         catch (LZ4Exception e)
         {
            e.printStackTrace();
            return null;
         }

         variablesPackage.byteBufferedPointCloud.setPosition(compressedPointCloudSize);
         variablesPackage.byteBufferedColors.setPosition(compressedColorSize);
      }

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
               filteredIndex++;
            }
         }

         numberOfPoints = filteredIndex;
         pointCloud = filteredPointCloud;
         colors = filteredColors;
      }

      return compressPointCloud(timestamp, PointAccessor.wrap(pointCloud), ColorAccessor.wrapRGB(colors), numberOfPoints, minimumResolution, null);
   }

   public static Point3D32[] decompressPointCloudToArray32(StereoVisionPointCloudMessage message)
   {
      return decompressPointCloudToArray32(message.getPointCloud(),
                                           message.getPointCloudCenter(),
                                           message.getResolution(),
                                           message.getNumberOfPoints(),
                                           message.getLz4Compressed());
   }

   public static Point3D32[] decompressPointCloudToArray32(TByteArrayList compressedPointCloud,
                                                           Point3D center,
                                                           double resolution,
                                                           int numberOfPoints,
                                                           boolean isLZ4Compressed)
   {
      Point3D32[] pointCloud = new Point3D32[numberOfPoints];
      decompressPointCloud(compressedPointCloud, center, resolution, numberOfPoints, isLZ4Compressed, PointCoordinateConsumer.toArray(pointCloud));
      return pointCloud;
   }

   public static Point3D[] decompressPointCloudToArray(StereoVisionPointCloudMessage message)
   {
      return decompressPointCloudToArray(message.getPointCloud(),
                                         message.getPointCloudCenter(),
                                         message.getResolution(),
                                         message.getNumberOfPoints(),
                                         message.getLz4Compressed());
   }

   public static Point3D[] decompressPointCloudToArray(TByteArrayList compressedPointCloud,
                                                       Point3D center,
                                                       double resolution,
                                                       int numberOfPoints,
                                                       boolean isLZ4Compressed)
   {
      Point3D[] pointCloud = new Point3D[numberOfPoints];
      decompressPointCloud(compressedPointCloud, center, resolution, numberOfPoints, isLZ4Compressed, PointCoordinateConsumer.toArray(pointCloud));
      return pointCloud;
   }

   public static void decompressPointCloud(StereoVisionPointCloudMessage message, PointCoordinateConsumer pointCoordinateConsumer)
   {
      decompressPointCloud(message.getPointCloud(),
                           message.getPointCloudCenter(),
                           message.getResolution(),
                           message.getNumberOfPoints(),
                           message.getLz4Compressed(),
                           pointCoordinateConsumer);
   }

   public static void decompressPointCloud(TByteArrayList compressedPointCloud,
                                           Point3D center,
                                           double resolution,
                                           int numberOfPoints,
                                           boolean isLZ4Compressed,
                                           PointCoordinateConsumer pointCoordinateConsumer)
   {
      if (isLZ4Compressed)
      {
         ByteBuffer compressedPointCloudByteBuffer = ByteBuffer.wrap(compressedPointCloud.toArray());
         ByteBuffer decompressedPointCloudByteBuffer = ByteBuffer.allocate(computePointByteBufferSize(numberOfPoints));
         compressorThreadLocal.get().decompress(compressedPointCloudByteBuffer, decompressedPointCloudByteBuffer, computePointByteBufferSize(numberOfPoints));
         decompressedPointCloudByteBuffer.flip();
         ShortBuffer pointCloudShortBuffer = decompressedPointCloudByteBuffer.asShortBuffer();

         for (int i = 0; i < numberOfPoints; i++)
         {
            double x = (pointCloudShortBuffer.get() + 0.5) * resolution;
            double y = (pointCloudShortBuffer.get() + 0.5) * resolution;
            double z = (pointCloudShortBuffer.get() + 0.5) * resolution;
            pointCoordinateConsumer.accept(x + center.getX(), y + center.getY(), z + center.getZ());
         }
      }
      else
      {
         ShortBuffer pointCloudShortBuffer = ByteBuffer.wrap(compressedPointCloud.toArray()).asShortBuffer();

         for (int i = 0; i < numberOfPoints; i++)
         {
            double x = (pointCloudShortBuffer.get() + 0.5) * resolution;
            double y = (pointCloudShortBuffer.get() + 0.5) * resolution;
            double z = (pointCloudShortBuffer.get() + 0.5) * resolution;
            pointCoordinateConsumer.accept(x + center.getX(), y + center.getY(), z + center.getZ());
         }
      }
   }

   public static Color[] decompressColorsToAWTColorArray(StereoVisionPointCloudMessage message)
   {
      return decompressColorsToAWTColorArray(message.getColors(), message.getNumberOfPoints(), message.getLz4Compressed());
   }

   public static Color[] decompressColorsToAWTColorArray(TByteArrayList compressedColors, int numberOfPoints, boolean isLZ4Compressed)
   {
      Color[] colors = new Color[numberOfPoints];

      decompressColors(compressedColors, numberOfPoints, isLZ4Compressed, new IntConsumer()
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
      return decompressColorsToIntArray(message.getColors(), message.getNumberOfPoints(), message.getLz4Compressed());
   }

   public static int[] decompressColorsToIntArray(TByteArrayList compressedColors, int numberOfPoints, boolean isLZ4Compressed)
   {
      int[] colors = new int[numberOfPoints];

      decompressColors(compressedColors, numberOfPoints, isLZ4Compressed, new IntConsumer()
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
      decompressColors(message.getColors(), message.getNumberOfPoints(), message.getLz4Compressed(), colorConsumer);
   }

   public static void decompressColors(TByteArrayList compressedColors, int numberOfPoints, boolean isLZ4Compressed, IntConsumer colorConsumer)
   {
      if (isLZ4Compressed)
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
      else
      {
         for (int i = 0; i < numberOfPoints; i++)
         {
            int offset = 3 * i;
            byte r = compressedColors.get(offset++);
            byte g = compressedColors.get(offset++);
            byte b = compressedColors.get(offset);
            colorConsumer.accept(rgb(r, g, b));
         }
      }
   }

   private static int rgb(byte r, byte g, byte b)
   {
      return ((r & 0xFF) << 16) | ((g & 0xFF) << 8) | ((b & 0xFF) << 0);
   }

   public static interface DiscretizationParameters
   {
      void update(PointAccessor pointCloud, int numberOfPoints);

      double getResolution();

      void getPointCloudCenter(Point3DBasics centerToPack);

      static DiscretizationParameters fixedDiscretizationParameters(Point3DReadOnly center, double resolution)
      {
         return new DiscretizationParameters()
         {
            @Override
            public void update(PointAccessor pointCloud, int numberOfPoints)
            {
            }

            @Override
            public double getResolution()
            {
               return resolution;
            }

            @Override
            public void getPointCloudCenter(Point3DBasics centerToPack)
            {
               if (center != null)
                  centerToPack.set(center);
               else
                  centerToPack.setToZero();

            }
         };
      }

      static DiscretizationParameters bbxCalculator(double minimumResolution)
      {
         return new DiscretizationParameters()
         {
            private float minX, maxX, minY, maxY, minZ, maxZ;
            private float centerX, centerY, centerZ;

            @Override
            public void update(PointAccessor pointAccessor, int numberOfPoints)
            {
               minX = maxX = pointAccessor.getX(0);
               minY = maxY = pointAccessor.getX(0);
               minZ = maxZ = pointAccessor.getX(0);

               for (int i = 1; i < numberOfPoints; i++)
               {
                  float x = pointAccessor.getX(i);
                  float y = pointAccessor.getY(i);
                  float z = pointAccessor.getZ(i);

                  if (x < minX)
                     minX = x;
                  else if (x > maxX)
                     maxX = x;

                  if (y < minY)
                     minY = y;
                  else if (y > maxY)
                     maxY = y;

                  if (z < minZ)
                     minZ = z;
                  else if (z > maxZ)
                     maxZ = z;
               }

               centerX = 0.5f * (maxX + minX);
               centerY = 0.5f * (maxY + minY);
               centerZ = 0.5f * (maxZ + minZ);
            }

            @Override
            public double getResolution()
            {
               double size = EuclidCoreTools.max(maxX - minX, maxY - minY, maxZ - minZ);
               return Math.max(minimumResolution, size / RESOLUTION_TO_SIZE_RATIO);
            }

            @Override
            public void getPointCloudCenter(Point3DBasics centerToPack)
            {
               centerToPack.set(centerX, centerY, centerZ);
            }
         };
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
      byte getRed(int pointIndex);

      byte getGreen(int pointIndex);

      byte getBlue(int pointIndex);

      default int getRGB(int pointIndex)
      {
         return rgb(getRed(pointIndex), getGreen(pointIndex), getBlue(pointIndex));
      }

      public static ColorAccessor wrapRGB(int[] rgbBuffer)
      {
         return new ColorAccessor()
         {
            @Override
            public byte getRed(int pointIndex)
            {
               return (byte) ((rgbBuffer[pointIndex] >> 16) & 0xFF);
            }

            @Override
            public byte getGreen(int pointIndex)
            {
               return (byte) ((rgbBuffer[pointIndex] >> 8) & 0xFF);
            }

            @Override
            public byte getBlue(int pointIndex)
            {
               return (byte) ((rgbBuffer[pointIndex] >> 0) & 0xFF);
            }

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
            public byte getRed(int pointIndex)
            {
               return rgbBuffer[3 * pointIndex];
            }

            @Override
            public byte getGreen(int pointIndex)
            {
               return rgbBuffer[3 * pointIndex + 1];
            }

            @Override
            public byte getBlue(int pointIndex)
            {
               return rgbBuffer[3 * pointIndex + 2];
            }
         };
      }
   }

   public static interface PointCoordinateConsumer
   {
      void accept(double x, double y, double z);

      static PointCoordinateConsumer toArray(Point3D[] pointCloud)
      {
         return new PointCoordinateConsumer()
         {
            int index = 0;

            @Override
            public void accept(double x, double y, double z)
            {
               pointCloud[index++] = new Point3D(x, y, z);
            }
         };
      }

      static PointCoordinateConsumer toArray(Point3D32[] pointCloud)
      {
         return new PointCoordinateConsumer()
         {
            int index = 0;

            @Override
            public void accept(double x, double y, double z)
            {
               pointCloud[index++] = new Point3D32((float) x, (float) y, (float) z);
            }
         };
      }
   }
}
