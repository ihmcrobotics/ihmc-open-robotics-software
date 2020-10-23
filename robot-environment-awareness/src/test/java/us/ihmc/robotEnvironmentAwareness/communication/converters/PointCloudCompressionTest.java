package us.ihmc.robotEnvironmentAwareness.communication.converters;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.HashSet;
import java.util.Random;
import java.util.Set;

import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;

public class PointCloudCompressionTest
{

   @Test
   public void test()
   {
      Random random = new Random(5235346);

      for (int i = 0; i < 20; i++)
      {
         long inputTimestamp = random.nextLong();
         int inputNumberOfPoints = random.nextInt(100000);
         double minimumResolution = (random.nextInt(100) + 1) / 1000.0;
         Set<Point3D> pointSet = new HashSet<>(inputNumberOfPoints);
         BoundingBox3D boundingBox = new BoundingBox3D();
         boundingBox.setToNaN();

         while (pointSet.size() < inputNumberOfPoints)
         {
            Point3D point = EuclidCoreRandomTools.nextPoint3D(random, 100.0);
            roundToPrecision(point, 3.0 * minimumResolution); // Makes sure points are not simplified.
            pointSet.add(point);
            boundingBox.updateToIncludePoint(point);
         }

         Point3D center = new Point3D();
         boundingBox.getCenterPoint(center);

         assertEquals(inputNumberOfPoints, pointSet.size());

         Point3D[] inputPointCloud = pointSet.toArray(new Point3D[inputNumberOfPoints]);
         int[] inputColors = new int[inputNumberOfPoints];
         StereoVisionPointCloudMessage message = PointCloudCompression.compressPointCloud(inputTimestamp,
                                                                                          inputPointCloud,
                                                                                          inputColors,
                                                                                          inputNumberOfPoints,
                                                                                          minimumResolution,
                                                                                          null);

         assertEquals(inputTimestamp, message.getTimestamp());
         assertEquals(inputNumberOfPoints, message.getNumberOfPoints());
         EuclidCoreTestTools.assertTuple3DEquals(center, message.getPointCloudCenter(), 1.0e-12);

         int[] outputColors = PointCloudCompression.decompressColorsToIntArray(message);
         assertArrayEquals(inputColors, outputColors);

         Point3D[] outputPointCloud = PointCloudCompression.decompressPointCloudToArray(message);

         assertEquals(inputNumberOfPoints, outputColors.length);

         for (int j = 0; j < inputNumberOfPoints; j++)
         {
            // The LZ4 compression seems to be messing with the digits that are below the resolution.
            EuclidCoreTestTools.assertTuple3DEquals("Iteration " + i + ", min resolution " + minimumResolution + ", point index " + j, inputPointCloud[j], outputPointCloud[j], 2.0 * minimumResolution);
         }
      }
   }

   private static void roundToPrecision(Point3D point, double precision)
   {
      point.setX(MathTools.roundToPrecision(point.getX(), precision));
      point.setY(MathTools.roundToPrecision(point.getY(), precision));
      point.setZ(MathTools.roundToPrecision(point.getZ(), precision));
   }
}
