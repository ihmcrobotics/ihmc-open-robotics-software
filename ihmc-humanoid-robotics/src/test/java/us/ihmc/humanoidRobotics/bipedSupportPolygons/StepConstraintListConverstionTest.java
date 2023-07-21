package us.ihmc.humanoidRobotics.bipedSupportPolygons;

import controller_msgs.msg.dds.StepConstraintMessage;
import controller_msgs.msg.dds.StepConstraintsListMessage;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StepConstraintRegionCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StepConstraintsListCommand;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.PlanarRegion;

import java.util.Arrays;
import java.util.Random;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.fail;

public class StepConstraintListConverstionTest
{
   private static final int iterations = 1000;

   @Test
   public void testEndToEnd()
   {
      Random random = new Random(1738L);
      StepConstraintsListCommand stepConstraintsListCommand1 = new StepConstraintsListCommand();
      StepConstraintsListCommand stepConstraintsListCommand2 = new StepConstraintsListCommand();
      StepConstraintRegionCommand stepConstraintsCommand1 = new StepConstraintRegionCommand();
      StepConstraintRegionCommand stepConstraintsCommand2 = new StepConstraintRegionCommand();
      StepConstraintRegionsList stepConstraintRegions = new StepConstraintRegionsList();
      StepConstraintRegion stepConstraintRegion = new StepConstraintRegion();

      for (int i = 0; i < iterations; i++)
      {
         PlanarRegion planarRegion = PlanarRegion.generatePlanarRegionFromRandomPolygonsWithRandomTransform(random, 1, 10.0, 5);

         if (i == 24)
            LogTools.info("poopy");

         // test just the region
         StepConstraintMessage message = StepConstraintMessageConverter.convertToStepConstraintMessage(planarRegion);
         stepConstraintsCommand1.setFromMessage(message);
         stepConstraintsCommand2.set(stepConstraintsCommand1);
         stepConstraintsCommand2.getStepConstraintRegion(stepConstraintRegion);

         PlanarRegion convertedRegion = StepConstraintListConverter.convertStepConstraintRegionToPlanarRegion(stepConstraintRegion);

         String failureMessage = "Failed on iteration " + i;

         assertMessageEquals(failureMessage, planarRegion, message, 1e-5);
         assertCommandEquals(failureMessage, planarRegion, stepConstraintsCommand2, 1e-5);
         assertPlanarRegionsEquals(failureMessage, planarRegion, stepConstraintRegion, 1e-5);
         assertPlanarRegionsEquals(failureMessage, planarRegion, convertedRegion, 1e-5);

         StepConstraintsListMessage listMessage = StepConstraintMessageConverter.convertToStepConstraintsListMessageFromPlanarRegions(Arrays.asList(planarRegion));
         stepConstraintsListCommand1.setFromMessage(listMessage);
         stepConstraintsListCommand2.set(stepConstraintsListCommand1);
         stepConstraintsListCommand2.get(stepConstraintRegions);

         convertedRegion = StepConstraintListConverter.convertStepConstraintRegionToPlanarRegion(stepConstraintRegions.getAsList().get(0));

         assertPlanarRegionsEquals(failureMessage, planarRegion, convertedRegion, 1e-5);
      }
   }

   private static void assertMessageEquals(String failureMessage, PlanarRegion expected, StepConstraintMessage actual, double epsilon)
   {
      EuclidCoreTestTools.assertGeometricallyEquals(failureMessage, expected.getNormal(), actual.getRegionNormal(), epsilon);
      EuclidCoreTestTools.assertGeometricallyEquals(failureMessage, expected.getPoint(), actual.getRegionOrigin(), epsilon);
      assertEquals(failureMessage, expected.getConcaveHullSize(), actual.getConcaveHullSize());
      for (int i = 0; i < expected.getConcaveHullSize(); i++)
      {
         EuclidCoreTestTools.assertEquals(failureMessage, expected.getConcaveHullVertex(i), new Point2D(actual.getVertexBuffer().get(i)), epsilon);
      }
   }

   private static void assertCommandEquals(String failureMessage, PlanarRegion expected, StepConstraintRegionCommand actual, double epsilon)
   {
      EuclidCoreTestTools.assertGeometricallyEquals(failureMessage, expected.getPoint(), actual.getRegionOrigin(), epsilon);
      EuclidCoreTestTools.assertGeometricallyEquals(failureMessage, expected.getNormal(), actual.getRegionNormal(), epsilon);

      EuclidCoreTestTools.assertGeometricallyEquals(failureMessage, expected.getTransformToWorld(), actual.getTransformToWorld(), epsilon);
      EuclidCoreTestTools.assertGeometricallyEquals(failureMessage, expected.getTransformToLocal(), actual.getTransformFromWorld(), epsilon);

      assertEquals(failureMessage, expected.getConcaveHullSize(), actual.getConcaveHullVertices().size());
      for (int i = 0; i < expected.getConcaveHullSize(); i++)
      {
         EuclidCoreTestTools.assertEquals(failureMessage, expected.getConcaveHullVertex(i), actual.getConcaveHullVertices().get(i), epsilon);
      }


//      assertEquals(expected.getNumberOfConvexPolygons(), actual.getNumberOfConvexPolygons());
//      for (int i = 0; i < expected.getNumberOfConvexPolygons(); i++)
//      {
//         EuclidCoreTestTools.assertEquals(actual.getConvexPolygon(i), expected.getConvexPolygon(i), epsilon);
//      }

   }

   private static void assertPlanarRegionsEquals(String failureMessage, PlanarRegion expected, StepConstraintRegion actual, double epsilon)
   {
      EuclidCoreTestTools.assertGeometricallyEquals(failureMessage, expected.getPoint(), actual.getPoint(), epsilon);
      EuclidCoreTestTools.assertGeometricallyEquals(failureMessage, expected.getNormal(), actual.getNormal(), epsilon);

      EuclidCoreTestTools.assertEquals(failureMessage, expected.getTransformToWorld(), actual.getTransformToWorld(), epsilon);
      EuclidCoreTestTools.assertEquals(failureMessage, expected.getTransformToLocal(), actual.getTransformToLocal(), epsilon);
      assertEquals(failureMessage, expected.getConcaveHullSize(), actual.getConcaveHullSize());
      for (int i = 0; i < expected.getConcaveHullSize(); i++)
      {
         EuclidCoreTestTools.assertEquals(failureMessage, expected.getConcaveHullVertex(i), actual.getConcaveHullVertexInRegionFrame(i), epsilon);
      }
//      assertEquals(expected.getNumberOfConvexPolygons(), actual.getNumberOfConvexPolygons());
//      for (int i = 0; i < expected.getNumberOfConvexPolygons(); i++)
//      {
//         EuclidCoreTestTools.assertEquals(actual.getConvexPolygon(i), expected.getConvexPolygon(i), epsilon);
//      }

   }

   private static void assertPlanarRegionsEquals(String failureMessage, PlanarRegion expected, PlanarRegion actual, double epsilon)
   {
      EuclidCoreTestTools.assertGeometricallyEquals(failureMessage, expected.getPoint(), actual.getPoint(), epsilon);
      EuclidCoreTestTools.assertGeometricallyEquals(failureMessage, expected.getNormal(), actual.getNormal(), epsilon);

      EuclidCoreTestTools.assertEquals(failureMessage, expected.getTransformToWorld(), actual.getTransformToWorld(), epsilon);
      EuclidCoreTestTools.assertEquals(failureMessage, expected.getTransformToLocal(), actual.getTransformToLocal(), epsilon);
      assertEquals(failureMessage, expected.getConcaveHullSize(), actual.getConcaveHullSize());
      for (int i = 0; i < expected.getConcaveHullSize(); i++)
      {
         EuclidCoreTestTools.assertEquals(failureMessage, expected.getConcaveHullVertex(i), actual.getConcaveHullVertex(i), epsilon);
      }
      assertEquals(failureMessage, expected.getNumberOfConvexPolygons(), actual.getNumberOfConvexPolygons());
      for (int i = 0; i < expected.getNumberOfConvexPolygons(); i++)
      {
         EuclidCoreTestTools.assertEquals(failureMessage, actual.getConvexPolygon(i), expected.getConvexPolygon(i), epsilon);
      }

   }
}
