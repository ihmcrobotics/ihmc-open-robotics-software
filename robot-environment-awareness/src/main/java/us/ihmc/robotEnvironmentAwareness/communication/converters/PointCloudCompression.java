package us.ihmc.robotEnvironmentAwareness.communication.converters;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import gnu.trove.list.array.TIntArrayList;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.idl.IDLSequence.Integer;
import us.ihmc.jOctoMap.key.OcTreeKey;
import us.ihmc.jOctoMap.key.OcTreeKeyReadOnly;
import us.ihmc.jOctoMap.tools.OcTreeKeyConversionTools;
import us.ihmc.jOctoMap.tools.OcTreeKeyTools;

public class PointCloudCompression
{
   private static final int OCTREE_DEPTH = 16;
   private static final double OCTREE_RESOLUTION_TO_SIZE_RATIO = Math.pow(2.0, OCTREE_DEPTH) - 1;

   /**
    * Compresses the given point-cloud by doing the following:
    * <ul>
    * <li>Filters out the points according to the given {@code filter} if provided.
    * <li>Stores the points into an octree and replaces the use Cartesian coordinates (3 * 32bits) for
    * the octree key system (3 * 16bits).
    * </ul>
    * 
    * @param timestamp
    * @param pointCloud
    * @param colors
    * @param numberOfPoints
    * @param filter
    * @return
    */
   public static StereoVisionPointCloudMessage compressPointCloud(long timestamp, Point3D[] pointCloud, int[] colors, int numberOfPoints,
                                                                  ScanPointFilter filter)
   {
      BoundingBox3D boundingBox = new BoundingBox3D();
      boundingBox.setToNaN();

      if (filter != null)
      {
         int filteredIndex = 0;
         Point3D[] filteredPointCloud = new Point3D[numberOfPoints];
         int[] filteredColors = new int[numberOfPoints];

         for (int i = 0; i < numberOfPoints; i++)
         {
            Point3D scanPoint = pointCloud[i];
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
         for (Point3D scanPoint : pointCloud)
         {
            boundingBox.updateToIncludePoint(scanPoint);
         }
      }

      double sizeX = boundingBox.getMaxX() - boundingBox.getMinX();
      double sizeY = boundingBox.getMaxY() - boundingBox.getMinY();
      double sizeZ = boundingBox.getMaxZ() - boundingBox.getMinZ();
      double centerX = 0.5 * (boundingBox.getMaxX() + boundingBox.getMinX());
      double centerY = 0.5 * (boundingBox.getMaxY() + boundingBox.getMinY());
      double centerZ = 0.5 * (boundingBox.getMaxZ() + boundingBox.getMinZ());
      double octreeSize = EuclidCoreTools.max(sizeX, sizeY, sizeZ);
      double octreeResolution = Math.max(0.003, octreeSize / OCTREE_RESOLUTION_TO_SIZE_RATIO);

      int octreeIndex = 0;
      OcTreeKey[] octreeKeys = new OcTreeKey[numberOfPoints];
      int[] octreeColors = new int[numberOfPoints];
      CompressionOctreeNode rootNode = new CompressionOctreeNode(0);

      for (int i = 0; i < numberOfPoints; i++)
      {
         Point3D scanPoint = pointCloud[i];
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

      StereoVisionPointCloudMessage message = new StereoVisionPointCloudMessage();
      message.setTimestamp(timestamp);
      message.setSensorPoseConfidence(1.0);
      boundingBox.getCenterPoint(message.getPointCloudCenter());
      message.setResolution(octreeResolution);

      Integer pointCloudBuffer = message.getPointCloud();
      Integer colorBuffer = message.getColors();

      for (int i = 0; i < octreeIndex; i++)
      {
         OcTreeKey key = octreeKeys[i];
         int color = octreeColors[i];

         pointCloudBuffer.add(key.getKey(0));
         pointCloudBuffer.add(key.getKey(1));
         pointCloudBuffer.add(key.getKey(2));
         colorBuffer.add(color);
      }

      return message;
   }

   public static Point3D32[] decompressPointCloudToArray32(StereoVisionPointCloudMessage message)
   {
      return decompressPointCloudToArray32(message.getPointCloud(), message.getPointCloudCenter(), message.getResolution());
   }

   public static Point3D32[] decompressPointCloudToArray32(TIntArrayList keys, Point3D center, double resolution)
   {
      Point3D32[] pointCloud = new Point3D32[keys.size() / 3];
      decompressPointCloud(keys, center, resolution, new PointCoordinateConsumer()
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
      return decompressPointCloudToArray(message.getPointCloud(), message.getPointCloudCenter(), message.getResolution());
   }

   public static Point3D[] decompressPointCloudToArray(TIntArrayList keys, Point3D center, double resolution)
   {
      Point3D[] pointCloud = new Point3D[keys.size() / 3];
      decompressPointCloud(keys, center, resolution, new PointCoordinateConsumer()
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
      decompressPointCloud(message.getPointCloud(), message.getPointCloudCenter(), message.getResolution(), pointCoordinateConsumer);
   }

   public static void decompressPointCloud(TIntArrayList keys, Point3D center, double resolution, PointCoordinateConsumer pointCoordinateConsumer)
   {
      int numberOfPoints = keys.size() / 3;

      for (int i = 0; i < numberOfPoints; i++)
      {
         double x = OcTreeKeyConversionTools.keyToCoordinate(keys.get(3 * i), resolution, OCTREE_DEPTH);
         double y = OcTreeKeyConversionTools.keyToCoordinate(keys.get(3 * i + 1), resolution, OCTREE_DEPTH);
         double z = OcTreeKeyConversionTools.keyToCoordinate(keys.get(3 * i + 2), resolution, OCTREE_DEPTH);

         pointCoordinateConsumer.accept(x + center.getX(), y + center.getY(), z + center.getZ());
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
