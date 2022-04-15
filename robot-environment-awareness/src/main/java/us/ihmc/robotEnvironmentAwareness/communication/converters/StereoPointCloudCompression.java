package us.ihmc.robotEnvironmentAwareness.communication.converters;

import static us.ihmc.jOctoMap.tools.OcTreeKeyTools.computeCenterOffsetKey;

import java.awt.Color;
import java.nio.ByteBuffer;
import java.nio.CharBuffer;
import java.util.function.IntConsumer;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import gnu.trove.list.array.TByteArrayList;
import net.jpountz.lz4.LZ4Exception;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.jOctoMap.key.OcTreeKeyReadOnly;
import us.ihmc.jOctoMap.tools.OcTreeKeyConversionTools;
import us.ihmc.jOctoMap.tools.OcTreeKeyTools;
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
      private final LZ4CompressionImplementation compressor = new LZ4CompressionImplementation();

      private ByteBufferedSequence byteBufferedPointCloud;
      private ByteBufferedSequence byteBufferedColors;
      private ByteBuffer compressedPointCloudByteBuffer;
      private ByteBuffer compressedColorByteBuffer;
      private final StereoVisionPointCloudMessage message = new StereoVisionPointCloudMessage();

      public CompressionIntermediateVariablesPackage()
      {
      }

      private void initialize(int numberOfPoints)
      {
         if (this.numberOfPoints == -1 || this.numberOfPoints != numberOfPoints)
         {
            this.numberOfPoints = numberOfPoints;
            int pointCloudByteBufferSize = numberOfPoints * 3 * 2;
            int colorByteBufferSize = numberOfPoints * 3;
            rawPointCloudByteBuffer = ByteBuffer.allocateDirect(pointCloudByteBufferSize);
            rawColorByteBuffer = ByteBuffer.allocateDirect(colorByteBufferSize);

            rawPointCloudCharBuffer = rawPointCloudByteBuffer.asCharBuffer();

            byteBufferedPointCloud = new ByteBufferedSequence(numberOfPoints * 3 * 2 + numberOfPoints / 2, "type_9");
            byteBufferedColors = new ByteBufferedSequence(message.colors_.capacity(), "type_9");

            message.point_cloud_ = byteBufferedPointCloud;
            message.colors_ = byteBufferedColors;
            compressedPointCloudByteBuffer = byteBufferedPointCloud.byteBuffer;
            compressedColorByteBuffer = byteBufferedColors.byteBuffer;
         }
         else
         {
            rawPointCloudByteBuffer.clear();
            rawColorByteBuffer.clear();
            byteBufferedPointCloud.reset();
            byteBufferedColors.reset();
         }
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
      ByteBuffer compressedPointCloudByteBuffer;
      ByteBuffer compressedColorByteBuffer;
      LZ4CompressionImplementation compressor;

      if (variablesPackage != null)
      {
         variablesPackage.initialize(numberOfPoints);
         rawPointCloudByteBuffer = variablesPackage.rawPointCloudByteBuffer;
         rawColorByteBuffer = variablesPackage.rawColorByteBuffer;
         rawPointCloudCharBuffer = variablesPackage.rawPointCloudCharBuffer;
         compressedPointCloudByteBuffer = variablesPackage.compressedPointCloudByteBuffer;
         compressedColorByteBuffer = variablesPackage.compressedColorByteBuffer;
         compressor = variablesPackage.compressor;
      }
      else
      {
         int pointCloudByteBufferSize = numberOfPoints * 3 * 2;
         int colorByteBufferSize = numberOfPoints * 3;
         rawPointCloudByteBuffer = ByteBuffer.allocate(pointCloudByteBufferSize);
         rawColorByteBuffer = ByteBuffer.allocate(colorByteBufferSize);
         rawPointCloudCharBuffer = rawPointCloudByteBuffer.asCharBuffer();
         compressedPointCloudByteBuffer = ByteBuffer.allocate(pointCloudByteBufferSize + numberOfPoints / 2);
         compressedColorByteBuffer = ByteBuffer.allocate(colorByteBufferSize);
         compressor = compressorThreadLocal.get();
      }

      rawColorByteBuffer.position(0);
      rawPointCloudCharBuffer.position(0);

      for (int i = 0; i < numberOfPoints; i++)
      {
         // int keyX = OcTreeKeyConversionTools.coordinateToKey((pointAccessor.getX(i) - centerX), octreeResolution, OCTREE_DEPTH);
         // int keyY = OcTreeKeyConversionTools.coordinateToKey((pointAccessor.getY(i) - centerY), octreeResolution, OCTREE_DEPTH);
         // int keyZ = OcTreeKeyConversionTools.coordinateToKey((pointAccessor.getZ(i) - centerZ), octreeResolution, OCTREE_DEPTH);
         int keyX = (int) ((pointAccessor.getX(i) - centerX) / octreeResolution) + centerOffsetKey;
         int keyY = (int) ((pointAccessor.getY(i) - centerY) / octreeResolution) + centerOffsetKey;
         int keyZ = (int) ((pointAccessor.getZ(i) - centerZ) / octreeResolution) + centerOffsetKey;
         rawPointCloudCharBuffer.put((char) keyX);
         rawPointCloudCharBuffer.put((char) keyY);
         rawPointCloudCharBuffer.put((char) keyZ);

         rawColorByteBuffer.put(colorAccessor.getRed(i));
         rawColorByteBuffer.put(colorAccessor.getGreen(i));
         rawColorByteBuffer.put(colorAccessor.getBlue(i));
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

      StereoVisionPointCloudMessage message;
      if (variablesPackage != null)
      {
         message = variablesPackage.message;
         variablesPackage.byteBufferedPointCloud.setPosition(compressedPointCloudSize);
         variablesPackage.byteBufferedColors.setPosition(compressedColorSize);
      }
      else
      {
         message = new StereoVisionPointCloudMessage();
         compressedPointCloudByteBuffer.flip();
         message.getPointCloud().add(compressedPointCloudByteBuffer.array());

         compressedColorByteBuffer.flip();
         message.getColors().add(compressedColorByteBuffer.array());
      }

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
      ByteBuffer decompressedPointCloudByteBuffer = ByteBuffer.allocate(numberOfPoints * 3 * 4);
      compressorThreadLocal.get().decompress(compressedPointCloudByteBuffer, decompressedPointCloudByteBuffer, numberOfPoints * 3 * 2);
      decompressedPointCloudByteBuffer.flip();
      CharBuffer pointCloudIntBuffer = decompressedPointCloudByteBuffer.asCharBuffer();

      for (int i = 0; i < numberOfPoints; i++)
      {
         double x = OcTreeKeyConversionTools.keyToCoordinate(pointCloudIntBuffer.get(), resolution, OCTREE_DEPTH);
         double y = OcTreeKeyConversionTools.keyToCoordinate(pointCloudIntBuffer.get(), resolution, OCTREE_DEPTH);
         double z = OcTreeKeyConversionTools.keyToCoordinate(pointCloudIntBuffer.get(), resolution, OCTREE_DEPTH);

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
      ByteBuffer decompressedColorByteBuffer = ByteBuffer.allocate(numberOfPoints * 3);
      compressorThreadLocal.get().decompress(compressedColorByteBuffer, decompressedColorByteBuffer, numberOfPoints * 3);
      decompressedColorByteBuffer.flip();

      for (int i = 0; i < numberOfPoints; i++)
      {
         colorConsumer.accept(rgb(decompressedColorByteBuffer.get(), decompressedColorByteBuffer.get(), decompressedColorByteBuffer.get()));
      }
   }

   private static int rgb(byte r, byte g, byte b)
   {
      return ((r & 0xFF) << 16) | ((g & 0xFF) << 8) | ((b & 0xFF) << 0);
   }

   public static interface PointCoordinateConsumer
   {
      void accept(double x, double y, double z);
   }

   private static class CompressionOctreeNode
   {
      private final int depth;
      private CompressionOctreeNode[] children;

      public CompressionOctreeNode(int depth)
      {
         this.depth = depth;
      }

      public boolean insertNode(OcTreeKeyReadOnly key, int treeDepth)
      {
         int childIndex = OcTreeKeyTools.computeChildIndex(key, depth, treeDepth);
         CompressionOctreeNode child = getChild(childIndex);

         boolean childCreated = child == null;

         if (childCreated)
            child = createChild(childIndex);

         if (depth == treeDepth - 2)
         {
            // The child is a leaf.
            return childCreated;
         }
         else
         {
            // Keep going down
            return child.insertNode(key, treeDepth);
         }
      }

      public CompressionOctreeNode getChild(int childIndex)
      {
         if (children == null)
            return null;

         return children[childIndex];
      }

      public CompressionOctreeNode createChild(int childIndex)
      {
         if (children == null)
            children = new CompressionOctreeNode[8];
         CompressionOctreeNode newChild = new CompressionOctreeNode(depth + 1);
         children[childIndex] = newChild;
         return newChild;
      }
   }
}
