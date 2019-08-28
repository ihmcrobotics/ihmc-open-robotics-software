package us.ihmc.robotEnvironmentAwareness.geometry;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.ConvexPolygonTools;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

public class ConcavePolygonToolsTest
{
   @Test
   public void testCutSimpleConcavePolygonAbove()
   {
      // create simple convex polygon
      ConcaveHull concaveSPolygon = new ConcaveHull();
      concaveSPolygon.addVertex(-2.0, 2.0);
      concaveSPolygon.addVertex(2.0, 2.0);
      concaveSPolygon.addVertex(2.0, 1.0);
      concaveSPolygon.addVertex(-1.0, 1.0);
      concaveSPolygon.addVertex(-1.0, 0.0);
      concaveSPolygon.addVertex(2.0, 0.0);
      concaveSPolygon.addVertex(2.0, -3.0);
      concaveSPolygon.addVertex(-2.0, -3.0);
      concaveSPolygon.addVertex(-2.0, -2.0);
      concaveSPolygon.addVertex(1.0, -2.0);
      concaveSPolygon.addVertex(1.0, -1.0);
      concaveSPolygon.addVertex(-2.0, -1.0);

      LogTools.info("{}", concaveSPolygon.getVertex(0));

      // create line and up direction
      Line2D yAxis = new Line2D(0.0, 0.0, 0.0, 1.0);
      Vector2D xDirection = new Vector2D(1.0, 0.0);

      // cut it above a line
      List<ConcaveHull> result = ConcavePolygonTools.cropPolygonToAboveLine(concaveSPolygon, yAxis, xDirection);

      assertEquals(2, result.size(), "should be two polygons");

      // create the ideal result
      ConcaveHull cutPolygon1 = new ConcaveHull();
      cutPolygon1.addVertex(1.0, 1.0);
      cutPolygon1.addVertex(1.0, -1.0);
      cutPolygon1.addVertex(0.0, -1.0);
      cutPolygon1.addVertex(0.0, 1.0);

      ConcaveHull cutPolygon2 = new ConcaveHull();
      cutPolygon2.addVertex(1.0, 1.0);
      cutPolygon2.addVertex(1.0, -1.0);
      cutPolygon2.addVertex(0.0, -1.0);
      cutPolygon2.addVertex(0.0, 1.0);

      // assert equal
      assertTrue(result.get(0).epsilonEquals(cutPolygon1, 1e-7));
   }

   @Test
   public void testCutSimpleConvexPolygonAbove()
   {
      // create simple convex polygon
      ConcaveHull size2square0center = new ConcaveHull();
      size2square0center.addVertex(1.0, 1.0);
      size2square0center.addVertex(1.0, -1.0);
      size2square0center.addVertex(-1.0, -1.0);
      size2square0center.addVertex(-1.0, 1.0);

      LogTools.info("{}", size2square0center.getVertex(0));

      // create line and up direction
      Line2D yAxis = new Line2D(0.0, 0.0, 0.0, 1.0);
      Vector2D xDirection = new Vector2D(1.0, 0.0);

      // cut it above a line
      List<ConcaveHull> result = ConcavePolygonTools.cropPolygonToAboveLine(size2square0center, yAxis, xDirection);

      assertEquals(1, result.size(), "supposed to cut");

      // create the ideal result
      ConcaveHull aboveYAxisRectangle = new ConcaveHull();
      aboveYAxisRectangle.addVertex(1.0, 1.0);
      aboveYAxisRectangle.addVertex(1.0, -1.0);
      aboveYAxisRectangle.addVertex(0.0, -1.0);
      aboveYAxisRectangle.addVertex(0.0, 1.0);

      // assert equal
      assertTrue(result.get(0).epsilonEquals(aboveYAxisRectangle, 1e-7));
   }

   @Test
   public void testCutSimpleConvexPolygonBelow()
   {
      // create simple convex polygon
      ConcaveHull size2square0center = new ConcaveHull();
      size2square0center.addVertex(1.0, 1.0);
      size2square0center.addVertex(1.0, -1.0);
      size2square0center.addVertex(-1.0, -1.0);
      size2square0center.addVertex(-1.0, 1.0);

      // create line and up direction
      Line2D yAxis = new Line2D(0.0, 0.0, 0.0, 1.0);
      Vector2D negativeXDirection = new Vector2D(-1.0, 0.0);

      // cut it above a line
      List<ConcaveHull> result = ConcavePolygonTools.cropPolygonToAboveLine(size2square0center, yAxis, negativeXDirection);

      assertEquals(1, result.size(), "supposed to cut");

      // create the ideal result
      ConcaveHull expected = new ConcaveHull();
      expected.addVertex(-1.0, 1.0);
      expected.addVertex(0.0, 1.0);
      expected.addVertex(0.0, -1.0);
      expected.addVertex(-1.0, -1.0);

      // assert equal
      assertTrue(result.get(0).epsilonEquals(expected, 1e-7));
   }

   @Test
   public void testKeepSideSimpleConvexPolygonAbove1()
   {
      // create simple convex polygon
      ConcaveHull size2square0center = new ConcaveHull();
      size2square0center.addVertex(1.0, 1.0);
      size2square0center.addVertex(1.0, -1.0);
      size2square0center.addVertex(-1.0, -1.0);
      size2square0center.addVertex(-1.0, 1.0);

      LogTools.info("{}", size2square0center.getVertex(0));

      // create line and up direction
      Line2D yAxis = new Line2D(1.0, 0.0, 0.0, 1.0);
      Vector2D xDirection = new Vector2D(1.0, 0.0);

      // cut it above a line
      List<ConcaveHull> result = ConcavePolygonTools.cropPolygonToAboveLine(size2square0center, yAxis, xDirection);

      assertEquals(1, result.size(), "supposed to cut");
      LogTools.debug("{}", result.get(0));

      // create the ideal result
      ConcaveHull expected = new ConcaveHull();
      expected.addVertex(1.0, 1.0);
      expected.addVertex(1.0, -1.0);

      // assert equal
      assertTrue(result.get(0).epsilonEquals(expected, 1e-7));
   }

   @Test
   public void testKeepLongSideSimpleConvexPolygonAbove1() // this is a pretty crazy edge case where the initial polygon has a redundant point along
   {                                                       // a colinear intersection with the cutting line
      // create simple convex polygon
      ConcaveHull size2square0center = new ConcaveHull();
      size2square0center.addVertex(1.0, 1.0);
      size2square0center.addVertex(1.0, 0.0);
      size2square0center.addVertex(1.0, -1.0);
      size2square0center.addVertex(-1.0, -1.0);

      LogTools.info("{}", size2square0center);

      // create line and up direction
      Line2D yAxis = new Line2D(1.0, 0.0, 0.0, 1.0);
      Vector2D xDirection = new Vector2D(1.0, 0.0);

      // cut it above a line
      List<ConcaveHull> result = ConcavePolygonTools.cropPolygonToAboveLine(size2square0center, yAxis, xDirection);

      LogTools.info("{}", result.get(0));
      assertEquals(1, result.size(), "supposed to cut");

      // create the ideal result
      ConcaveHull expected = new ConcaveHull();
      expected.addVertex(1.0, 1.0);
      expected.addVertex(1.0, -1.0);

      // assert equal
      assertTrue(result.get(0).epsilonEquals(expected, 1e-7));
   }


   @Test
   public void testRemoveAllSimpleConvexPolygonAbove2()
   {
      // create simple convex polygon
      ConcaveHull size2square0center = new ConcaveHull();
      size2square0center.addVertex(1.0, 1.0);
      size2square0center.addVertex(1.0, -1.0);
      size2square0center.addVertex(-1.0, -1.0);
      size2square0center.addVertex(-1.0, 1.0);

      LogTools.info("{}", size2square0center.getVertex(0));

      // create line and up direction
      Line2D yAxis = new Line2D(2.0, 0.0, 0.0, 1.0);
      Vector2D xDirection = new Vector2D(1.0, 0.0);

      // cut it above a line
      List<ConcaveHull> result = ConcavePolygonTools.cropPolygonToAboveLine(size2square0center, yAxis, xDirection);

      assertEquals(0, result.size(), "supposed to cut");
   }

   @Test
   public void testKeepAllSimpleConvexPolygonAbove1()
   {
      // create simple convex polygon
      ConcaveHull size2square0center = new ConcaveHull();
      size2square0center.addVertex(1.0, 1.0);
      size2square0center.addVertex(-1.0, 1.0);
      size2square0center.addVertex(1.0, -1.0);
      size2square0center.addVertex(-1.0, -1.0);

      LogTools.info("{}", size2square0center);

      // create line and up direction
      Line2D yAxis = new Line2D(1.0, 0.0, 0.0, 1.0);
      Vector2D xDirection = new Vector2D(-1.0, 0.0);

      // cut it above a line
      List<ConcaveHull> result = ConcavePolygonTools.cropPolygonToAboveLine(size2square0center, yAxis, xDirection);

      assertEquals(1, result.size(), "supposed to cut");
      LogTools.debug("{}", result.get(0));

      // create the ideal result
      ConcaveHull expected = new ConcaveHull();
      expected.addVertex(-1.0, 1.0);
      expected.addVertex(1.0, 1.0);
      expected.addVertex(1.0, -1.0);
      expected.addVertex(-1.0, -1.0);

      // assert equal
      assertTrue(result.get(0).epsilonEquals(expected, 1e-7));
   }

   @Test
   public void testKeepAllSimpleConvexPolygonAbove2()
   {
      // create simple convex polygon
      ConcaveHull size2square0center = new ConcaveHull();
      size2square0center.addVertex(1.0, 1.0);
      size2square0center.addVertex(1.0, -1.0);
      size2square0center.addVertex(-1.0, -1.0);
      size2square0center.addVertex(-1.0, 1.0);

      LogTools.info("{}", size2square0center.getVertex(0));

      // create line and up direction
      Line2D yAxis = new Line2D(2.0, 0.0, 0.0, 1.0);
      Vector2D xDirection = new Vector2D(-1.0, 0.0);

      // cut it above a line
      List<ConcaveHull> result = ConcavePolygonTools.cropPolygonToAboveLine(size2square0center, yAxis, xDirection);

      assertEquals(1, result.size(), "supposed to cut");

      // create the ideal result
      ConcaveHull expected = new ConcaveHull();
      expected.addVertex(1.0, 1.0);
      expected.addVertex(1.0, -1.0);
      expected.addVertex(-1.0, -1.0);
      expected.addVertex(-1.0, 1.0);

      // assert equal
      assertTrue(result.get(0).epsilonEquals(expected, 1e-7));
   }

   public static void main(String[] args)
   {
      // TODO: Uncomment when ihmc-commons released
//      MutationTestFacilitator mutationTestFacilitator = new MutationTestFacilitator();
//      mutationTestFacilitator.addClassesToMutate(ConcavePolygonTools.class, ConvexPolygonTools.class);
//      mutationTestFacilitator.addMethodsToMutate("cropPolygonToAboveLine");
//      mutationTestFacilitator.addTestClassesToRun(ConcavePolygonToolsTest.class);
//      mutationTestFacilitator.addSourceDirectories(PathTools.findPathInline(".", "ihmc-open-robotics-software", "ihmc-robotics-toolkit/src"));
//      mutationTestFacilitator.addSourceDirectories(PathTools.findPathInline(".", "ihmc-open-robotics-software", "robot-environment-awareness/src"));
//      mutationTestFacilitator.doMutationTest();
//      mutationTestFacilitator.openResultInBrowser();

      MutationTestFacilitator.facilitateMutationTestForClass(ConcavePolygonTools.class, ConvexPolygonTools.class);
   }
}
