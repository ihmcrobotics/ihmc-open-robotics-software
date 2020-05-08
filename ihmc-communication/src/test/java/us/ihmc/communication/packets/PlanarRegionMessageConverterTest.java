package us.ihmc.communication.packets;

import static us.ihmc.robotics.Assert.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.PlanarRegionMessage;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTestTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class PlanarRegionMessageConverterTest
{
   private static final int ITERATIONS = 30;
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testPlanarRegionConversion()
   {
      Random random = new Random(6342);

      for (int i = 0; i < ITERATIONS; i++)
      {
         PlanarRegion expected = nextPlanarRegion(random);

         PlanarRegionMessage message = PlanarRegionMessageConverter.convertToPlanarRegionMessage(expected);

         PlanarRegion actual = PlanarRegionMessageConverter.convertToPlanarRegion(message);

         PlanarRegionTestTools.assertPlanarRegionsEqual(expected, actual, EPSILON);
      }
   }

   @Test
   public void testPlanarRegionsListConversion()
   {
      Random random = new Random(6342);

      for (int i = 0; i < ITERATIONS; i++)
      {
         PlanarRegionsList expected = nextPlanarRegionsList(random);

         PlanarRegionsListMessage message = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(expected);

         PlanarRegionsList actual = PlanarRegionMessageConverter.convertToPlanarRegionsList(message);

         assertEquals(expected.getNumberOfPlanarRegions(), actual.getNumberOfPlanarRegions());
         for (int j = 0; j < expected.getNumberOfPlanarRegions(); j++)
         {
            PlanarRegion expectedPlanarRegion = expected.getPlanarRegion(j);
            PlanarRegion actualPlanarRegion = actual.getPlanarRegion(j);

            PlanarRegionTestTools.assertPlanarRegionsEqual(expectedPlanarRegion, actualPlanarRegion, EPSILON);
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         PlanarRegionsList expected = nextPlanarRegionsList(random);

         List<PlanarRegionMessage> messages = new ArrayList<>();
         for (PlanarRegion planarRegion : expected.getPlanarRegionsAsList())
            messages.add(PlanarRegionMessageConverter.convertToPlanarRegionMessage(planarRegion));

         PlanarRegionsListMessage message = PlanarRegionMessageConverter.createPlanarRegionsListMessage(messages);

         PlanarRegionsList actual = PlanarRegionMessageConverter.convertToPlanarRegionsList(message);

         assertEquals(expected.getNumberOfPlanarRegions(), actual.getNumberOfPlanarRegions());
         for (int j = 0; j < expected.getNumberOfPlanarRegions(); j++)
         {
            PlanarRegion expectedPlanarRegion = expected.getPlanarRegion(j);
            PlanarRegion actualPlanarRegion = actual.getPlanarRegion(j);

            PlanarRegionTestTools.assertPlanarRegionsEqual(expectedPlanarRegion, actualPlanarRegion, EPSILON);
         }
      }
   }


   private static PlanarRegionsList nextPlanarRegionsList(Random random)
   {
      return new PlanarRegionsList(nextPlanarRegionList(random));
   }

   private static List<PlanarRegion> nextPlanarRegionList(Random random)
   {
      int size = random.nextInt(100);
      return IntStream.range(0, size).mapToObj(i -> nextPlanarRegion(random)).collect(Collectors.toList());
   }

   private static PlanarRegion nextPlanarRegion(Random random)
   {
      RigidBodyTransform transformToWorld = nextRegionTransform(random);
      List<Point2D> concaveHullVertices = nextPoint2DList(random);
      List<ConvexPolygon2D> convexPolygons = nextConvexPolygon2Ds(random);
      PlanarRegion next = new PlanarRegion(transformToWorld, concaveHullVertices, convexPolygons);
      next.setRegionId(random.nextInt());
      return next;
   }

   private static RigidBodyTransform nextRegionTransform(Random random)
   {
      Vector3D regionNormal = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
      Point3D regionOrigin = EuclidCoreRandomTools.nextPoint3D(random);
      return new RigidBodyTransform(EuclidGeometryTools.axisAngleFromZUpToVector3D(regionNormal), regionOrigin);
   }

   private static List<ConvexPolygon2D> nextConvexPolygon2Ds(Random random)
   {
      int size = random.nextInt(100);
      return IntStream.range(0, size).mapToObj(i -> EuclidGeometryRandomTools.nextConvexPolygon2D(random, 10.0, 100)).collect(Collectors.toList());
   }

   private static List<Point2D> nextPoint2DList(Random random)
   {
      int size = random.nextInt(500);
      return IntStream.range(0, size).mapToObj(i -> EuclidCoreRandomTools.nextPoint2D(random)).collect(Collectors.toList());
   }
}
