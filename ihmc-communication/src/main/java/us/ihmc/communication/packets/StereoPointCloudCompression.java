package us.ihmc.communication.packets;

import java.awt.Color;
import java.nio.ByteBuffer;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import net.jpountz.lz4.LZ4Exception;
import us.ihmc.communication.compression.LZ4CompressionImplementation;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.idl.IDLSequence;

/**
 * This class should be used to create and unpack a {@link StereoVisionPointCloudMessage}.
 */
public class StereoPointCloudCompression
{
   private static final boolean USE_LZ4_COMPRESSION_DEFAULT = true;

   private static final int OCTREE_DEPTH = 16;
   private static final double RESOLUTION_TO_SIZE_RATIO = Math.pow(2.0, OCTREE_DEPTH) - 1;
   private static final ThreadLocal<LZ4CompressionImplementation> compressorThreadLocal = ThreadLocal.withInitial(LZ4CompressionImplementation::new);

   public static class CompressionIntermediateVariablesPackage
   {
      private int numberOfPoints = -1;
      private ByteBuffer rawPointCloudByteBuffer;
      private ByteBuffer rawColorByteBuffer;
      private final LZ4CompressionImplementation compressor = new LZ4CompressionImplementation();

      private IDLSequence.Byte byteBufferedPointCloud;
      private IDLSequence.Byte byteBufferedColors;
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

            // LZ4 compression apparently needs some additional room to do its thing.
            byteBufferedPointCloud = new IDLSequence.Byte(pointCloudByteBufferSize * 2, "type_9");
            byteBufferedColors = new IDLSequence.Byte(colorByteBufferSize * 2, "type_9");

            message.point_cloud_ = byteBufferedPointCloud;
            message.colors_ = byteBufferedColors;
            compressedPointCloudByteBuffer = byteBufferedPointCloud.getBuffer();
            compressedColorByteBuffer = byteBufferedColors.getBuffer();
         }
         else
         {
            compressedPointCloudByteBuffer.clear();
            compressedColorByteBuffer.clear();

            rawPointCloudByteBuffer.limit(pointCloudByteBufferSize);
            rawColorByteBuffer.limit(colorByteBufferSize);

            rawPointCloudByteBuffer.position(0);
            rawColorByteBuffer.position(0);
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
         message.getPointCloud().resetQuick();
         message.getColors().resetQuick();
      }
   }

   public static int computeColorByteBufferSize(int numberOfPoints)
   {
      return numberOfPoints * 3;
   }

   public static int computePointByteBufferSize(int numberOfPoints)
   {
      return numberOfPoints * 3 * 2;
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

      float centerX = message.getPointCloudCenter().getX32();
      float centerY = message.getPointCloudCenter().getY32();
      float centerZ = message.getPointCloudCenter().getZ32();
      message.setResolution(discretizationParameters.getResolution());
      float invResolution = (float) (1.0 / message.getResolution());

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

         variablesPackage.byteBufferedPointCloud.getBuffer().position(variablesPackage.pointCloudByteBufferSize);
         variablesPackage.byteBufferedColors.getBuffer().position(variablesPackage.colorByteBufferSize);
      }
      else
      {
         ByteBuffer rawPointCloudByteBuffer = variablesPackage.rawPointCloudByteBuffer;
         ByteBuffer rawColorByteBuffer = variablesPackage.rawColorByteBuffer;
         ByteBuffer compressedPointCloudByteBuffer = variablesPackage.compressedPointCloudByteBuffer;
         ByteBuffer compressedColorByteBuffer = variablesPackage.compressedColorByteBuffer;
         LZ4CompressionImplementation compressor = variablesPackage.compressor;

         for (int i = 0; i < numberOfPoints; i++)
         {
            rawPointCloudByteBuffer.putShort((short) ((pointAccessor.getX(i) - centerX) * invResolution));
            rawPointCloudByteBuffer.putShort((short) ((pointAccessor.getY(i) - centerY) * invResolution));
            rawPointCloudByteBuffer.putShort((short) ((pointAccessor.getZ(i) - centerZ) * invResolution));
            rawColorByteBuffer.put(colorAccessor.getRed(i));
            rawColorByteBuffer.put(colorAccessor.getGreen(i));
            rawColorByteBuffer.put(colorAccessor.getBlue(i));
         }

         rawPointCloudByteBuffer.flip();
         rawColorByteBuffer.flip();

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

         variablesPackage.byteBufferedPointCloud.getBuffer().position(compressedPointCloudSize);
         variablesPackage.byteBufferedColors.getBuffer().position(compressedColorSize);
      }

      return message;
   }

   public static Point3D32[] decompressPointCloudToArray32(StereoVisionPointCloudMessage message)
   {
      return decompressPointCloudToArray32(message.getPointCloud(),
                                           message.getPointCloudCenter(),
                                           message.getResolution(),
                                           message.getNumberOfPoints(),
                                           message.getLz4Compressed());
   }

   public static Point3D32[] decompressPointCloudToArray32(IDLSequence.Byte compressedPointCloud,
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

   public static Point3D[] decompressPointCloudToArray(IDLSequence.Byte compressedPointCloud,
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

   public static void decompressPointCloud(IDLSequence.Byte compressedPointCloud,
                                           Point3D center,
                                           double resolution,
                                           int numberOfPoints,
                                           boolean isLZ4Compressed,
                                           PointCoordinateConsumer pointCoordinateConsumer)
   {
      if (isLZ4Compressed)
      {
         ByteBuffer compressedPointCloudByteBuffer = compressedPointCloud.copyByteBuffer();
         ByteBuffer decompressedPointCloudByteBuffer = ByteBuffer.allocate(computePointByteBufferSize(numberOfPoints));
         compressorThreadLocal.get().decompress(compressedPointCloudByteBuffer, decompressedPointCloudByteBuffer, computePointByteBufferSize(numberOfPoints));
         decompressedPointCloudByteBuffer.flip();

         for (int i = 0; i < numberOfPoints; i++)
         {
            double x = getPointCoordinate(decompressedPointCloudByteBuffer.get(), decompressedPointCloudByteBuffer.get(), resolution);
            double y = getPointCoordinate(decompressedPointCloudByteBuffer.get(), decompressedPointCloudByteBuffer.get(), resolution);
            double z = getPointCoordinate(decompressedPointCloudByteBuffer.get(), decompressedPointCloudByteBuffer.get(), resolution);
            pointCoordinateConsumer.accept(x + center.getX(), y + center.getY(), z + center.getZ());
         }
      }
      else
      {
         int bufferOffset = 0;
         while (bufferOffset < compressedPointCloud.size())
         {
            double x = getPointCoordinate(compressedPointCloud.get(bufferOffset++), compressedPointCloud.get(bufferOffset++), resolution);
            double y = getPointCoordinate(compressedPointCloud.get(bufferOffset++), compressedPointCloud.get(bufferOffset++), resolution);
            double z = getPointCoordinate(compressedPointCloud.get(bufferOffset++), compressedPointCloud.get(bufferOffset++), resolution);
            pointCoordinateConsumer.accept(x + center.getX(), y + center.getY(), z + center.getZ());
         }
      }
   }

   public static double getPointCoordinate(byte byteHi, byte byteLo, double resolution)
   {
      return (toShort(byteHi, byteLo) + 0.5) * resolution;
   }

   public static Color[] decompressColorsToAWTColorArray(StereoVisionPointCloudMessage message)
   {
      return decompressColorsToAWTColorArray(message.getColors(), message.getNumberOfPoints(), message.getLz4Compressed());
   }

   public static Color[] decompressColorsToAWTColorArray(IDLSequence.Byte compressedColors, int numberOfPoints, boolean isLZ4Compressed)
   {
      Color[] colors = new Color[numberOfPoints];
      decompressColors(compressedColors, numberOfPoints, isLZ4Compressed, ColorConsumer.toAWTColorArray(colors));
      return colors;
   }

   public static int[] decompressColorsToIntArray(StereoVisionPointCloudMessage message)
   {
      return decompressColorsToIntArray(message.getColors(), message.getNumberOfPoints(), message.getLz4Compressed());
   }

   public static int[] decompressColorsToIntArray(IDLSequence.Byte compressedColors, int numberOfPoints, boolean isLZ4Compressed)
   {
      int[] colors = new int[numberOfPoints];
      decompressColors(compressedColors, numberOfPoints, isLZ4Compressed, ColorConsumer.toRGBArray(colors));
      return colors;
   }

   public static void decompressColors(StereoVisionPointCloudMessage message, ColorConsumer colorConsumer)
   {
      decompressColors(message.getColors(), message.getNumberOfPoints(), message.getLz4Compressed(), colorConsumer);
   }

   public static void decompressColors(IDLSequence.Byte compressedColors, int numberOfPoints, boolean isLZ4Compressed, ColorConsumer colorConsumer)
   {
      if (isLZ4Compressed)
      {
         ByteBuffer compressedColorByteBuffer = compressedColors.copyByteBuffer();
         int colorByteBufferSize = computeColorByteBufferSize(numberOfPoints);
         ByteBuffer decompressedColorByteBuffer = ByteBuffer.allocate(colorByteBufferSize);
         compressorThreadLocal.get().decompress(compressedColorByteBuffer, decompressedColorByteBuffer, colorByteBufferSize);
         decompressedColorByteBuffer.flip();

         for (int i = 0; i < numberOfPoints; i++)
         {
            byte r = decompressedColorByteBuffer.get();
            byte g = decompressedColorByteBuffer.get();
            byte b = decompressedColorByteBuffer.get();
            colorConsumer.accept(r & 0xFF, g & 0xFF, b & 0xFF);
         }
      }
      else
      {
         int offset = 0;

         while (offset < compressedColors.size())
         {
            byte r = compressedColors.get(offset++);
            byte g = compressedColors.get(offset++);
            byte b = compressedColors.get(offset++);
            colorConsumer.accept(r & 0xFF, g & 0xFF, b & 0xFF);
         }
      }
   }

   public static short toShort(byte byteHi, byte byteLo)
   {
      return (short) (((byteHi & 0xFF) << 8) | ((byteLo & 0xFF) << 0));
   }

   public static int rgb(byte r, byte g, byte b)
   {
      return ((r & 0xFF) << 16) | ((g & 0xFF) << 8) | ((b & 0xFF) << 0);
   }

   public static int rgb(int r, int g, int b)
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

   public static interface ColorConsumer
   {
      /**
       * @param red the red component in the range (0 - 255).
       * @param green the green component in the range (0 - 255).
       * @param blue the blue component in the range (0 - 255).
       */
      void accept(int red, int blue, int green);

      static ColorConsumer toRGBArray(int[] colors)
      {
         return new ColorConsumer()
         {
            int index = 0;

            @Override
            public void accept(int red, int green, int blue)
            {
               colors[index++] = rgb(red, green, blue);
            }
         };
      }

      static ColorConsumer toAWTColorArray(Color[] colors)
      {
         return new ColorConsumer()
         {
            int index = 0;

            @Override
            public void accept(int red, int green, int blue)
            {
               colors[index++] = new Color(red, green, blue);
            }
         };
      }
   }
}
