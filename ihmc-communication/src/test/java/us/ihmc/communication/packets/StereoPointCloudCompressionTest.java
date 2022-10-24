package us.ihmc.communication.packets;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

import java.awt.Color;
import java.util.HashSet;
import java.util.Random;
import java.util.Set;

import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.commons.MathTools;
import us.ihmc.communication.packets.StereoPointCloudCompression.ColorAccessor;
import us.ihmc.communication.packets.StereoPointCloudCompression.CompressionIntermediateVariablesPackage;
import us.ihmc.communication.packets.StereoPointCloudCompression.DiscretizationParameters;
import us.ihmc.communication.packets.StereoPointCloudCompression.PointAccessor;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;

public class StereoPointCloudCompressionTest
{
   @Test
   public void test()
   {
      Random random = new Random(5235346);
      CompressionIntermediateVariablesPackage variablesPackage = new CompressionIntermediateVariablesPackage();

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
         for (int j = 0; j < inputColors.length; j++)
         {
            Color color = new Color(random.nextInt(256), random.nextInt(256), random.nextInt(256), 0);
            inputColors[j] = color.getRGB();
         }
         PointAccessor pointAccessor = PointAccessor.wrap(inputPointCloud);
         ColorAccessor colorAccessor = ColorAccessor.wrapRGB(inputColors);
         StereoVisionPointCloudMessage message = StereoPointCloudCompression.compressPointCloud(inputTimestamp,
                                                                                                pointAccessor,
                                                                                                colorAccessor,
                                                                                                inputNumberOfPoints,
                                                                                                minimumResolution,
                                                                                                variablesPackage);

         assertEquals(inputTimestamp, message.getTimestamp());
         assertEquals(inputNumberOfPoints, message.getNumberOfPoints());
         EuclidCoreTestTools.assertEquals(center, message.getPointCloudCenter(), 1.0e-5);

         int[] outputColors = StereoPointCloudCompression.decompressColorsToIntArray(message);
         assertEquals(inputNumberOfPoints, outputColors.length);
         assertArrayEquals(inputColors, outputColors);

         Point3D[] outputPointCloud = StereoPointCloudCompression.decompressPointCloudToArray(message);

         for (int j = 0; j < inputNumberOfPoints; j++)
         {
            // The LZ4 compression seems to be messing with the digits that are below the resolution.
            EuclidCoreTestTools.assertEquals("Iteration " + i + ", min resolution " + minimumResolution + ", point index " + j,
                                                    inputPointCloud[j],
                                                    outputPointCloud[j],
                                                    2.0 * minimumResolution);
         }
      }
   }

   public static void main(String[] args)
   { // For benchmark

      Random random = new Random(1231);

      int width = 1024;
      int height = 768;
      int inputNumberOfPoints = width * height;
      Point3D[] inputPointCloud = new Point3D[inputNumberOfPoints];
      int[] inputColors = new int[inputNumberOfPoints];
      double minimumResolution = 1.0e-3;
      CompressionIntermediateVariablesPackage variablesPackage = new CompressionIntermediateVariablesPackage();

      RigidBodyTransform pose = new RigidBodyTransform();
      double timeElapsedFiltered = 0.0;

      for (int i = 0; i < 2000000; i++)
      {

         for (int w = 0; w < width; w++)
         {
            for (int h = 0; h < height; h++)
            {
               inputPointCloud[w * height + h] = new Point3D(w / (double) width, h / (double) height, random.nextDouble());
               inputColors[w * height + h] = 0;
            }
         }

         pose.set(EuclidCoreRandomTools.nextRigidBodyTransform(random));

         for (Point3D point : inputPointCloud)
         {
            pose.transform(point);
         }

         long start = System.nanoTime();
         StereoVisionPointCloudMessage message = StereoPointCloudCompression.compressPointCloud(-1,
                                                                                                PointAccessor.wrap(inputPointCloud),
                                                                                                ColorAccessor.wrapRGB(inputColors),
                                                                                                DiscretizationParameters.fixedDiscretizationParameters(null,
                                                                                                                                                       minimumResolution),
                                                                                                inputNumberOfPoints,
                                                                                                variablesPackage);
         long end = System.nanoTime();
         double timeElapsedMs = (end - start) * 1.0e-6;
         if (timeElapsedFiltered == 0.0)
            timeElapsedFiltered = timeElapsedMs;
         else
            timeElapsedFiltered = EuclidCoreTools.interpolate(timeElapsedFiltered, timeElapsedMs, 0.1);

         System.out.println("Time elapsed: " + EuclidCoreIOTools.getStringOf("[ms], filt.: ", timeElapsedMs, timeElapsedFiltered) + "[ms], PC size [comp.: "
               + message.getPointCloud().size() + ", raw: " + StereoPointCloudCompression.computePointByteBufferSize(inputNumberOfPoints)
               + "], color size [comp.: " + message.getColors().size() + ", raw: " + StereoPointCloudCompression.computeColorByteBufferSize(inputNumberOfPoints)
               + "]");
      }
   }

   private static void roundToPrecision(Point3D point, double precision)
   {
      point.setX(MathTools.roundToPrecision(point.getX(), precision));
      point.setY(MathTools.roundToPrecision(point.getY(), precision));
      point.setZ(MathTools.roundToPrecision(point.getZ(), precision));
   }
}
