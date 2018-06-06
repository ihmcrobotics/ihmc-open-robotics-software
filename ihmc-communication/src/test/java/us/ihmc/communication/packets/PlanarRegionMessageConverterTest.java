package us.ihmc.communication.packets;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import org.junit.Test;

import controller_msgs.msg.dds.PlanarRegionMessage;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.PlanarRegion;
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

         assertPlanarRegionEquals(expected, actual);
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

            assertPlanarRegionEquals(expectedPlanarRegion, actualPlanarRegion);
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

            assertPlanarRegionEquals(expectedPlanarRegion, actualPlanarRegion);
         }
      }
   }

   private void assertPlanarRegionEquals(PlanarRegion expected, PlanarRegion actual)
   {
      RigidBodyTransform expectedTransform = new RigidBodyTransform();
      RigidBodyTransform actualTransform = new RigidBodyTransform();
      expected.getTransformToWorld(expectedTransform);
      actual.getTransformToWorld(actualTransform);
      EuclidCoreTestTools.assertRigidBodyTransformGeometricallyEquals(expectedTransform, actualTransform, EPSILON);

      assertEquals(expected.getConcaveHullSize(), actual.getConcaveHullSize());

      for (int i = 0; i < expected.getConcaveHullSize(); i++)
      {
         EuclidCoreTestTools.assertTuple2DEquals(expected.getConcaveHullVertex(i), actual.getConcaveHullVertex(i), EPSILON);
      }

      assertEquals(expected.getNumberOfConvexPolygons(), actual.getNumberOfConvexPolygons());

      for (int i = 0; i < expected.getNumberOfConvexPolygons(); i++)
      {
         ConvexPolygon2D expectedConvexPolygon = expected.getConvexPolygon(i);
         ConvexPolygon2D actualConvexPolygon = actual.getConvexPolygon(i);
         assertEquals(expectedConvexPolygon.getNumberOfVertices(), actualConvexPolygon.getNumberOfVertices());
         assertTrue(expectedConvexPolygon.epsilonEquals(actualConvexPolygon, EPSILON));
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
      Point2D[] concaveHullVertices = nextPoint2DArray(random);
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

   private static Point2D[] nextPoint2DArray(Random random)
   {
      int size = random.nextInt(500);
      return IntStream.range(0, size).mapToObj(i -> EuclidCoreRandomTools.nextPoint2D(random)).toArray(Point2D[]::new);
   }
}
