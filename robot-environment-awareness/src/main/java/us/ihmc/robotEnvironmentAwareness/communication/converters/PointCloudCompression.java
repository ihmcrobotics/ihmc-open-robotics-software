package us.ihmc.robotEnvironmentAwareness.communication.converters;

import java.awt.Color;
import java.nio.ByteBuffer;
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
import us.ihmc.jOctoMap.key.OcTreeKey;
import us.ihmc.jOctoMap.key.OcTreeKeyReadOnly;
import us.ihmc.jOctoMap.tools.OcTreeKeyConversionTools;
import us.ihmc.jOctoMap.tools.OcTreeKeyTools;
import us.ihmc.tools.compression.LZ4CompressionImplementation;

/**
 * This class should be used to create and unpack a {@link StereoVisionPointCloudMessage}.
 */
public class PointCloudCompression
{
   private static final int OCTREE_DEPTH = 16;
   private static final double OCTREE_RESOLUTION_TO_SIZE_RATIO = Math.pow(2.0, OCTREE_DEPTH) - 1;
   private static final ThreadLocal<LZ4CompressionImplementation> compressorThreadLocal = new ThreadLocal<LZ4CompressionImplementation>()
   {
      @Override
      protected LZ4CompressionImplementation initialValue()
      {
         return new LZ4CompressionImplementation();
      }
   };

   /**
    * Compresses the given point-cloud by doing the following:
    * <ul>
    * <li>Filters out the points according to the given {@code filter} if provided.
    * <li>Stores the points into an octree and replaces the use Cartesian coordinates (3 * 32bits) for
    * the octree key system (3 * 16bits).
    * <li>Use {@link LZ4CompressionImplementation} as a final lossless compression pass.
    * </ul>
    */
   public static StereoVisionPointCloudMessage compressPointCloud(long timestamp, Point3DReadOnly[] pointCloud, int[] colors, int numberOfPoints, double minimumResolution,
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

      // 2- Store the pointcloud in an octree.
      // The resolution of the octree adapts to the size of the bounding box.
      // Duplicate node are filtered out.
      // The output of this is the key of the occupied leaves of the octree with can be represented a 3*int which is half the memory w.r.t. to a Point3D32.
      double sizeX = boundingBox.getMaxX() - boundingBox.getMinX();
      double sizeY = boundingBox.getMaxY() - boundingBox.getMinY();
      double sizeZ = boundingBox.getMaxZ() - boundingBox.getMinZ();
      double centerX = 0.5 * (boundingBox.getMaxX() + boundingBox.getMinX());
      double centerY = 0.5 * (boundingBox.getMaxY() + boundingBox.getMinY());
      double centerZ = 0.5 * (boundingBox.getMaxZ() + boundingBox.getMinZ());
      double octreeSize = EuclidCoreTools.max(sizeX, sizeY, sizeZ);
      double octreeResolution = Math.max(minimumResolution, octreeSize / OCTREE_RESOLUTION_TO_SIZE_RATIO);

      int octreeIndex = 0;
      OcTreeKey[] octreeKeys = new OcTreeKey[numberOfPoints];
      int[] octreeColors = new int[numberOfPoints];
      CompressionOctreeNode rootNode = new CompressionOctreeNode(0);

      for (int i = 0; i < numberOfPoints; i++)
      {
         Point3DReadOnly scanPoint = pointCloud[i];
         int color = colors[i];

         double x = scanPoint.getX() - centerX;
         double y = scanPoint.getY() - centerY;
         double z = scanPoint.getZ() - centerZ;
         OcTreeKey key = OcTreeKeyConversionTools.coordinateToKey(x, y, z, octreeResolution, OCTREE_DEPTH);
         if (rootNode.insertNode(key, OCTREE_DEPTH))
         {
            octreeKeys[octreeIndex] = key;
            octreeColors[octreeIndex] = color;
            octreeIndex++;
         }
      }

      numberOfPoints = octreeIndex;

      // 3- Last step: We pack the data in byte buffer and use LZ4CompressionImplementation to compress it.
      int pointCloudByteBufferSize = numberOfPoints * 3 * 4;
      int colorByteBufferSize = numberOfPoints * 4;
      ByteBuffer rawPointCloudByteBuffer = ByteBuffer.allocate(pointCloudByteBufferSize);
      ByteBuffer rawColorByteBuffer = ByteBuffer.allocate(colorByteBufferSize);

      IntBuffer rawPointCloudIntBuffer = rawPointCloudByteBuffer.asIntBuffer();
      IntBuffer rawColorIntBuffer = rawColorByteBuffer.asIntBuffer();

      for (int i = 0; i < numberOfPoints; i++)
      {
         OcTreeKey key = octreeKeys[i];
         int color = octreeColors[i];

         rawPointCloudIntBuffer.put(3 * i, key.getKey(0));
         rawPointCloudIntBuffer.put(3 * i + 1, key.getKey(1));
         rawPointCloudIntBuffer.put(3 * i + 2, key.getKey(2));
         rawColorIntBuffer.put(i, color);
      }

      ByteBuffer compressedPointCloudByteBuffer = ByteBuffer.allocate(pointCloudByteBufferSize);
      ByteBuffer compressedColorByteBuffer = ByteBuffer.allocate(colorByteBufferSize);
      LZ4CompressionImplementation compressor = compressorThreadLocal.get();

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

      StereoVisionPointCloudMessage message = new StereoVisionPointCloudMessage();
      message.setTimestamp(timestamp);
      message.setSensorPoseConfidence(1.0);
      boundingBox.getCenterPoint(message.getPointCloudCenter());
      message.setResolution(octreeResolution);

      compressedPointCloudByteBuffer.flip();
      for (int i = 0; i < compressedPointCloudSize; i++)
         message.getPointCloud().add(compressedPointCloudByteBuffer.get());

      compressedColorByteBuffer.flip();
      for (int i = 0; i < compressedColorSize; i++)
         message.getColors().add(compressedColorByteBuffer.get());

      message.setNumberOfPoints(numberOfPoints);

      return message;
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

   public static void decompressPointCloud(TByteArrayList compressedPointCloud, Point3D center, double resolution, int numberOfPoints,
                                           PointCoordinateConsumer pointCoordinateConsumer)
   {
      ByteBuffer compressedPointCloudByteBuffer = ByteBuffer.wrap(compressedPointCloud.toArray());
      ByteBuffer decompressedPointCloudByteBuffer = ByteBuffer.allocate(numberOfPoints * 3 * 4);
      compressorThreadLocal.get().decompress(compressedPointCloudByteBuffer, decompressedPointCloudByteBuffer, numberOfPoints * 3 * 4);
      decompressedPointCloudByteBuffer.flip();
      IntBuffer pointCloudIntBuffer = decompressedPointCloudByteBuffer.asIntBuffer();

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
      ByteBuffer decompressedColorByteBuffer = ByteBuffer.allocate(numberOfPoints * 4);
      compressorThreadLocal.get().decompress(compressedColorByteBuffer, decompressedColorByteBuffer, numberOfPoints * 4);
      decompressedColorByteBuffer.flip();
      IntBuffer colorIntBuffer = decompressedColorByteBuffer.asIntBuffer();

      for (int i = 0; i < numberOfPoints; i++)
      {
         colorConsumer.accept(colorIntBuffer.get());
      }
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
