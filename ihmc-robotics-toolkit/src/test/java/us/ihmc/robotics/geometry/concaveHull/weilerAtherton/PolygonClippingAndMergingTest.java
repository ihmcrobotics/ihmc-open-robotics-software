package us.ihmc.robotics.geometry.concaveHull.weilerAtherton;

import org.junit.jupiter.api.Test;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2D;
import us.ihmc.robotics.geometry.concavePolygon2D.GeometryPolygonTools;
import us.ihmc.robotics.geometry.concavePolygon2D.weilerAtherton.PolygonClippingAndMerging;

import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertTrue;

public class PolygonClippingAndMergingTest
{
   @Test
   public void testClippingRemoveSquareChunkFromSideOfSquare()
   {
      ConcavePolygon2D polygonToClip = new ConcavePolygon2D();
      polygonToClip.addVertex(-1.0, 1.0);
      polygonToClip.addVertex(1.0, 1.0);
      polygonToClip.addVertex(1.0, -1.0);
      polygonToClip.addVertex(-1.0, -1.0);
      polygonToClip.update();

      assertTrue(polygonToClip.isPointInside(0.5, 0.5));

      ConcavePolygon2D clippingPolygon = new ConcavePolygon2D();
      clippingPolygon.addVertex(1.0 + -0.5, 0.5);
      clippingPolygon.addVertex(1.0 + 0.5, 0.5);
      clippingPolygon.addVertex(1.0 + 0.5, -0.5);
      clippingPolygon.addVertex(1.0 + -0.5, -0.5);
      clippingPolygon.update();

      assertTrue(polygonToClip.isPointInside(clippingPolygon.getVertex(0)));
      assertFalse(polygonToClip.isPointInside(clippingPolygon.getVertex(1)));
      assertFalse(polygonToClip.isPointInside(clippingPolygon.getVertex(2)));
      assertTrue(polygonToClip.isPointInside(clippingPolygon.getVertex(3)));

      assertTrue(GeometryPolygonTools.doPolygonsIntersect(polygonToClip, clippingPolygon));

      ConcavePolygon2D clippedPolygonExpected = new ConcavePolygon2D();
      clippedPolygonExpected.addVertex(-1.0, 1.0);
      clippedPolygonExpected.addVertex(1.0, 1.0);
      clippedPolygonExpected.addVertex(1.0, 0.5);
      clippedPolygonExpected.addVertex(0.5, 0.5);
      clippedPolygonExpected.addVertex(0.5, -0.5);
      clippedPolygonExpected.addVertex(1.0, -0.5);
      clippedPolygonExpected.addVertex(1.0, -1.0);
      clippedPolygonExpected.addVertex(-1.0, -1.0);
      clippedPolygonExpected.update();

      ConcavePolygon2D clippedPolygon = new ConcavePolygon2D();
      PolygonClippingAndMerging.removeAreaInsideClip(clippingPolygon, polygonToClip, clippedPolygon);

      assertTrue(clippedPolygon.epsilonEquals(clippedPolygonExpected, 1e-7));
   }

   @Test
   public void testMergeAddingSquareChunkFromSideOfSquare()
   {
      ConcavePolygon2D polygonB = new ConcavePolygon2D();
      polygonB.addVertex(-1.0, 1.0);
      polygonB.addVertex(1.0, 1.0);
      polygonB.addVertex(1.0, -1.0);
      polygonB.addVertex(-1.0, -1.0);
      polygonB.update();

      assertTrue(polygonB.isPointInside(0.5, 0.5));

      ConcavePolygon2D polygonA = new ConcavePolygon2D();
      polygonA.addVertex(1.0 + -0.5, 0.5);
      polygonA.addVertex(1.0 + 0.5, 0.5);
      polygonA.addVertex(1.0 + 0.5, -0.5);
      polygonA.addVertex(1.0 + -0.5, -0.5);
      polygonA.update();


      ConcavePolygon2D mergedPolygonExpected = new ConcavePolygon2D();
      mergedPolygonExpected.addVertex(-1.0, 1.0);
      mergedPolygonExpected.addVertex(1.0, 1.0);
      mergedPolygonExpected.addVertex(1.0, 0.5);
      mergedPolygonExpected.addVertex(1.5, 0.5);
      mergedPolygonExpected.addVertex(1.5, -0.5);
      mergedPolygonExpected.addVertex(1.0, -0.5);
      mergedPolygonExpected.addVertex(1.0, -1.0);
      mergedPolygonExpected.addVertex(-1.0, -1.0);
      mergedPolygonExpected.update();

      ConcavePolygon2D mergedPolygon = new ConcavePolygon2D();
      PolygonClippingAndMerging.merge(polygonA, polygonB, mergedPolygon);

      assertTrue(mergedPolygon.epsilonEquals(mergedPolygonExpected, 1e-7));
   }

   @Test
   public void testDoPolygonsIntersectWithNoCrossContainedPoints()
   {
      ConcavePolygon2D polygonA = new ConcavePolygon2D();
      polygonA.addVertex(1.0, 0.5);
      polygonA.addVertex(1.0, -0.5);
      polygonA.addVertex(-1.0, -0.5);
      polygonA.addVertex(-1.0, 0.5);
      polygonA.update();

      ConcavePolygon2D polygonB = new ConcavePolygon2D();
      polygonB.addVertex(0.5, 1.0);
      polygonB.addVertex(0.5, -1.0);
      polygonB.addVertex(-0.5, -1.0);
      polygonB.addVertex(-0.5, 1.0);
      polygonB.update();

      assertFalse(polygonA.getVertexBufferView().stream().anyMatch(polygonB::isPointInside));
      assertFalse(polygonB.getVertexBufferView().stream().anyMatch(polygonA::isPointInside));
      assertTrue(GeometryPolygonTools.doPolygonsIntersect(polygonA, polygonB));
   }


   @Test
   public void testClipRemoveSquareChunkFromCornerOfSquare()
   {
      ConcavePolygon2D polygonToClip = new ConcavePolygon2D();
      polygonToClip.addVertex(-1.0, 1.0);
      polygonToClip.addVertex(1.0, 1.0);
      polygonToClip.addVertex(1.0, -1.0);
      polygonToClip.addVertex(-1.0, -1.0);
      polygonToClip.update();

      assertTrue(polygonToClip.isPointInside(0.5, 0.5));

      ConcavePolygon2D clippingPolygon = new ConcavePolygon2D();
      clippingPolygon.addVertex(1.0 + -0.5, 1.0 + 0.5);
      clippingPolygon.addVertex(1.0 + 0.5, 1.0 + 0.5);
      clippingPolygon.addVertex(1.0 + 0.5, 1.0 - 0.5);
      clippingPolygon.addVertex(1.0 + -0.5, 1.0 - 0.5);
      clippingPolygon.update();

      assertTrue(GeometryPolygonTools.doPolygonsIntersect(polygonToClip, clippingPolygon));

      ConcavePolygon2D clippedPolygonExpected = new ConcavePolygon2D();
      clippedPolygonExpected.addVertex(-1.0, 1.0);
      clippedPolygonExpected.addVertex(0.5, 1.0);
      clippedPolygonExpected.addVertex(0.5, 0.5);
      clippedPolygonExpected.addVertex(1.0, 0.5);
      clippedPolygonExpected.addVertex(1.0, -1.0);
      clippedPolygonExpected.addVertex(-1.0, -1.0);
      clippedPolygonExpected.update();

      ConcavePolygon2D clippedPolygon = new ConcavePolygon2D();
      PolygonClippingAndMerging.removeAreaInsideClip(clippingPolygon, polygonToClip, clippedPolygon);

      assertTrue(clippedPolygon.epsilonEquals(clippedPolygonExpected, 1e-7));
   }

   @Test
   public void testMergeAddSquareChunkFromCornerOfSquare()
   {
      ConcavePolygon2D polygonA = new ConcavePolygon2D();
      polygonA.addVertex(-1.0, 1.0);
      polygonA.addVertex(1.0, 1.0);
      polygonA.addVertex(1.0, -1.0);
      polygonA.addVertex(-1.0, -1.0);
      polygonA.update();

      assertTrue(polygonA.isPointInside(0.5, 0.5));

      ConcavePolygon2D polygonB = new ConcavePolygon2D();
      polygonB.addVertex(1.0 + -0.5, 1.0 + 0.5);
      polygonB.addVertex(1.0 + 0.5, 1.0 + 0.5);
      polygonB.addVertex(1.0 + 0.5, 1.0 - 0.5);
      polygonB.addVertex(1.0 + -0.5, 1.0 - 0.5);
      polygonB.update();

      ConcavePolygon2D mergedPolygonExpected = new ConcavePolygon2D();
      mergedPolygonExpected.addVertex(-1.0, 1.0);
      mergedPolygonExpected.addVertex(0.5, 1.0);
      mergedPolygonExpected.addVertex(0.5, 1.5);
      mergedPolygonExpected.addVertex(1.5, 1.5);
      mergedPolygonExpected.addVertex(1.5, 0.5);
      mergedPolygonExpected.addVertex(1.0, 0.5);
      mergedPolygonExpected.addVertex(1.0, -1.0);
      mergedPolygonExpected.addVertex(-1.0, -1.0);
      mergedPolygonExpected.update();

      ConcavePolygon2D clippedPolygon = new ConcavePolygon2D();
      PolygonClippingAndMerging.merge(polygonB, polygonA, clippedPolygon);

      assertTrue(clippedPolygon.epsilonEquals(mergedPolygonExpected, 1e-7));
   }

   @Test
   public void testClippingWithTriangleClippingCornerOfSquare()
   {
      ConcavePolygon2D polygonToClip = new ConcavePolygon2D();
      polygonToClip.addVertex(-1.0, 1.0);
      polygonToClip.addVertex(1.0, 1.0);
      polygonToClip.addVertex(1.0, -1.0);
      polygonToClip.addVertex(-1.0, -1.0);
      polygonToClip.update();

      assertTrue(polygonToClip.isPointInside(0.5, 0.5));

      ConcavePolygon2D clippingPolygon = new ConcavePolygon2D();
      clippingPolygon.addVertex(0.0, 1.5);
      clippingPolygon.addVertex(1.5, 1.5);
      clippingPolygon.addVertex(1.5, 0.0);
      clippingPolygon.update();

      ConcavePolygon2D clippedPolygonExpected = new ConcavePolygon2D();
      clippedPolygonExpected.addVertex(-1.0, 1.0);
      clippedPolygonExpected.addVertex(0.5, 1.0);
      clippedPolygonExpected.addVertex(1.0, 0.5);
      clippedPolygonExpected.addVertex(1.0, -1.0);
      clippedPolygonExpected.addVertex(-1.0, -1.0);
      clippedPolygonExpected.update();

      ConcavePolygon2D clippedPolygon = new ConcavePolygon2D();
      PolygonClippingAndMerging.removeAreaInsideClip(clippingPolygon, polygonToClip, clippedPolygon);

      assertTrue(clippedPolygon.epsilonEquals(clippedPolygonExpected, 1e-7));
   }

   @Test
   public void testMergingWithTriangleClippingCornerOfSquare()
   {
      ConcavePolygon2D polygonA = new ConcavePolygon2D();
      polygonA.addVertex(-1.0, 1.0);
      polygonA.addVertex(1.0, 1.0);
      polygonA.addVertex(1.0, -1.0);
      polygonA.addVertex(-1.0, -1.0);
      polygonA.update();

      assertTrue(polygonA.isPointInside(0.5, 0.5));

      ConcavePolygon2D polygonB = new ConcavePolygon2D();
      polygonB.addVertex(0.0, 1.5);
      polygonB.addVertex(1.5, 1.5);
      polygonB.addVertex(1.5, 0.0);
      polygonB.update();

      ConcavePolygon2D mergedPolygonExpected = new ConcavePolygon2D();
      mergedPolygonExpected.addVertex(-1.0, 1.0);
      mergedPolygonExpected.addVertex(0.5, 1.0);
      mergedPolygonExpected.addVertex(0.0, 1.5);
      mergedPolygonExpected.addVertex(1.5, 1.5);
      mergedPolygonExpected.addVertex(1.5, 0.0);
      mergedPolygonExpected.addVertex(1.0, 0.5);
      mergedPolygonExpected.addVertex(1.0, -1.0);
      mergedPolygonExpected.addVertex(-1.0, -1.0);
      mergedPolygonExpected.update();

      ConcavePolygon2D mergedPolygon = new ConcavePolygon2D();
      PolygonClippingAndMerging.merge(polygonB, polygonA, mergedPolygon);

      assertTrue(mergedPolygon.epsilonEquals(mergedPolygonExpected, 1e-7));
   }

   @Test
   public void testClippingWithTriangleOnTopEdgeOfSquare()
   {
      ConcavePolygon2D polygonToClip = new ConcavePolygon2D();
      polygonToClip.addVertex(-1.0, 1.0);
      polygonToClip.addVertex(1.0, 1.0);
      polygonToClip.addVertex(1.0, -1.0);
      polygonToClip.addVertex(-1.0, -1.0);
      polygonToClip.update();

      assertTrue(polygonToClip.isPointInside(0.5, 0.5));

      ConcavePolygon2D clippingPolygon = new ConcavePolygon2D();
      clippingPolygon.addVertex(-1.0, 1.5);
      clippingPolygon.addVertex(1.0, 1.5);
      clippingPolygon.addVertex(0.0, 0.5);
      clippingPolygon.update();

      ConcavePolygon2D clippedPolygonExpected = new ConcavePolygon2D();
      clippedPolygonExpected.addVertex(-1.0, 1.0);
      clippedPolygonExpected.addVertex(-0.5, 1.0);
      clippedPolygonExpected.addVertex(0.0, 0.5);
      clippedPolygonExpected.addVertex(0.5, 1.0);
      clippedPolygonExpected.addVertex(1.0, 1.0);
      clippedPolygonExpected.addVertex(1.0, -1.0);
      clippedPolygonExpected.addVertex(-1.0, -1.0);
      clippedPolygonExpected.update();

      ConcavePolygon2D clippedPolygon = new ConcavePolygon2D();
      PolygonClippingAndMerging.removeAreaInsideClip(clippingPolygon, polygonToClip, clippedPolygon);

      assertTrue(clippedPolygon.epsilonEquals(clippedPolygonExpected, 1e-7));

      clippingPolygon = new ConcavePolygon2D();
      clippingPolygon.addVertex(-1.5, 1.5);
      clippingPolygon.addVertex(1.5, 1.5);
      clippingPolygon.addVertex(0.0, 0.0);
      clippingPolygon.update();

      clippedPolygonExpected = new ConcavePolygon2D();
      clippedPolygonExpected.addVertex(-1.0, 1.0);
      clippedPolygonExpected.addVertex(0.0, 0.0);
      clippedPolygonExpected.addVertex(1.0, 1.0);
      clippedPolygonExpected.addVertex(1.0, -1.0);
      clippedPolygonExpected.addVertex(-1.0, -1.0);
      clippedPolygonExpected.update();

      clippedPolygon = new ConcavePolygon2D();
      PolygonClippingAndMerging.removeAreaInsideClip(clippingPolygon, polygonToClip, clippedPolygon);

      assertTrue(clippedPolygon.epsilonEquals(clippedPolygonExpected, 1e-7));
   }

   @Test
   public void testMergingWithTriangleOnTopEdgeOfSquare()
   {
      ConcavePolygon2D polygonA = new ConcavePolygon2D();
      polygonA.addVertex(-1.0, 1.0);
      polygonA.addVertex(1.0, 1.0);
      polygonA.addVertex(1.0, -1.0);
      polygonA.addVertex(-1.0, -1.0);
      polygonA.update();


      ConcavePolygon2D polygonB = new ConcavePolygon2D();
      polygonB.addVertex(-1.0, 1.5);
      polygonB.addVertex(1.0, 1.5);
      polygonB.addVertex(0.0, 0.5);
      polygonB.update();

      ConcavePolygon2D mergedPolygonExpected = new ConcavePolygon2D();
      mergedPolygonExpected.addVertex(-1.0, 1.0);
      mergedPolygonExpected.addVertex(-0.5, 1.0);
      mergedPolygonExpected.addVertex(-1.0, 1.5);
      mergedPolygonExpected.addVertex(1.0, 1.5);
      mergedPolygonExpected.addVertex(0.5, 1.0);
      mergedPolygonExpected.addVertex(1.0, 1.0);
      mergedPolygonExpected.addVertex(1.0, -1.0);
      mergedPolygonExpected.addVertex(-1.0, -1.0);
      mergedPolygonExpected.update();

      ConcavePolygon2D mergedPolygon = new ConcavePolygon2D();
      PolygonClippingAndMerging.merge(polygonA, polygonB, mergedPolygon);

      assertTrue(mergedPolygon.epsilonEquals(mergedPolygonExpected, 1e-7));

      polygonB = new ConcavePolygon2D();
      polygonB.addVertex(-1.5, 1.5);
      polygonB.addVertex(1.5, 1.5);
      polygonB.addVertex(0.0, 0.0);
      polygonB.update();

      mergedPolygonExpected = new ConcavePolygon2D();
      mergedPolygonExpected.addVertex(-1.0, 1.0);
      mergedPolygonExpected.addVertex(-1.5, 1.5);
      mergedPolygonExpected.addVertex(1.5, 1.5);
      mergedPolygonExpected.addVertex(1.0, 1.0);
      mergedPolygonExpected.addVertex(1.0, -1.0);
      mergedPolygonExpected.addVertex(-1.0, -1.0);
      mergedPolygonExpected.update();

      mergedPolygon = new ConcavePolygon2D();
      PolygonClippingAndMerging.merge(polygonB, polygonA, mergedPolygon);

      assertTrue(mergedPolygon.epsilonEquals(mergedPolygonExpected, 1e-7));
   }

   @Test
   public void testClippingRemoveComplexShapeAcrossTopEdge()
   {
      ConcavePolygon2D polygonToClip = new ConcavePolygon2D();
      polygonToClip.addVertex(-1.5, 1.0);
      polygonToClip.addVertex(2.0, 1.0);
      polygonToClip.addVertex(2.0, -1.0);
      polygonToClip.addVertex(-1.5, -1.0);
      polygonToClip.update();

      assertTrue(polygonToClip.isPointInside(0.5, 0.5));

      ConcavePolygon2D clippingPolygon = new ConcavePolygon2D();
      clippingPolygon.addVertex(-1.0, 2.0);
      clippingPolygon.addVertex(2.0, 2.0);
      clippingPolygon.addVertex(2.0, 1.5);
      clippingPolygon.addVertex(1.0, 0.5);
      clippingPolygon.addVertex(0.0, 1.5);
      clippingPolygon.addVertex(-1.0, 0.5);
      clippingPolygon.update();

      ConcavePolygon2D clippedPolygonExpected = new ConcavePolygon2D();
      clippedPolygonExpected.addVertex(-1.5, 1.0);
      clippedPolygonExpected.addVertex(-1.0, 1.0);
      clippedPolygonExpected.addVertex(-1.0, 0.5);
      clippedPolygonExpected.addVertex(-0.5, 1.0);
      clippedPolygonExpected.addVertex(0.5, 1.0);
      clippedPolygonExpected.addVertex(1.0, 0.5);
      clippedPolygonExpected.addVertex(1.5, 1.0);
      clippedPolygonExpected.addVertex(2.0, 1.0);
      clippedPolygonExpected.addVertex(2.0, -1.0);
      clippedPolygonExpected.addVertex(-1.5, -1.0);
      clippedPolygonExpected.update();

      ConcavePolygon2D clippedPolygon = new ConcavePolygon2D();
      PolygonClippingAndMerging.removeAreaInsideClip(clippingPolygon, polygonToClip, clippedPolygon);

      assertTrue(clippedPolygon.epsilonEquals(clippedPolygonExpected, 1e-7));
   }

   @Test
   public void testMergingComplexShapeAcrossTopEdge()
   {
      ConcavePolygon2D polygonA = new ConcavePolygon2D();
      polygonA.addVertex(-1.5, 1.0);
      polygonA.addVertex(2.0, 1.0);
      polygonA.addVertex(2.0, -1.0);
      polygonA.addVertex(-1.5, -1.0);
      polygonA.update();

      assertTrue(polygonA.isPointInside(0.5, 0.5));

      ConcavePolygon2D polygonB = new ConcavePolygon2D();
      polygonB.addVertex(-1.0, 2.0);
      polygonB.addVertex(2.0, 2.0);
      polygonB.addVertex(2.0, 1.5);
      polygonB.addVertex(1.0, 0.5);
      polygonB.addVertex(0.0, 1.5);
      polygonB.addVertex(-1.0, 0.5);
      polygonB.update();

      ConcavePolygon2D mergedPolygonExpected = new ConcavePolygon2D();
      mergedPolygonExpected.addVertex(-1.5, 1.0);
      mergedPolygonExpected.addVertex(-1.0, 1.0);
      mergedPolygonExpected.addVertex(-1.0, 2.0);
      mergedPolygonExpected.addVertex(2.0, 2.0);
      mergedPolygonExpected.addVertex(2.0, 1.5);
      mergedPolygonExpected.addVertex(1.5, 1.0);
      mergedPolygonExpected.addVertex(2.0, 1.0);
      mergedPolygonExpected.addVertex(2.0, -1.0);
      mergedPolygonExpected.addVertex(-1.5, -1.0);
      mergedPolygonExpected.update();

      ConcavePolygon2D mergedPolygon = new ConcavePolygon2D();
      PolygonClippingAndMerging.merge(polygonB, polygonA, mergedPolygon);

      assertTrue(mergedPolygon.epsilonEquals(mergedPolygonExpected, 1e-7));
   }

   @Test
   public void testClipWithTrickyOverlappingJoints()
   {
      ConcavePolygon2D polygonToClip = new ConcavePolygon2D();
      polygonToClip.addVertex(-1.0, 1.0);
      polygonToClip.addVertex(1.0, 1.0);
      polygonToClip.addVertex(1.0, -1.0);
      polygonToClip.addVertex(-1.0, -1.0);
      polygonToClip.update();

      ConcavePolygon2D clippingPolygon = new ConcavePolygon2D();
      clippingPolygon.addVertex(0.5, 1.5);
      clippingPolygon.addVertex(1.5, 1.5);
      clippingPolygon.addVertex(0.5, 0.5);
      clippingPolygon.update();

      ConcavePolygon2D clippedPolygonExpected = new ConcavePolygon2D();
      clippedPolygonExpected.addVertex(-1.0, 1.0);
      clippedPolygonExpected.addVertex(0.5, 1.0);
      clippedPolygonExpected.addVertex(0.5, 0.5);
      clippedPolygonExpected.addVertex(1.0, 1.0);
      clippedPolygonExpected.addVertex(1.0, -1.0);
      clippedPolygonExpected.addVertex(-1.0, -1.0);
      clippedPolygonExpected.update();

      ConcavePolygon2D clippedPolygon = new ConcavePolygon2D();
      PolygonClippingAndMerging.removeAreaInsideClip(clippingPolygon, polygonToClip, clippedPolygon);

      assertTrue(clippedPolygon.epsilonEquals(clippedPolygonExpected, 1e-7));
   }

   @Test
   public void testMergeWithTrickyOverlappingJoints()
   {
      ConcavePolygon2D polygonA = new ConcavePolygon2D();
      polygonA.addVertex(-1.0, 1.0);
      polygonA.addVertex(1.0, 1.0);
      polygonA.addVertex(1.0, -1.0);
      polygonA.addVertex(-1.0, -1.0);
      polygonA.update();

      ConcavePolygon2D polygonB = new ConcavePolygon2D();
      polygonB.addVertex(0.5, 1.5);
      polygonB.addVertex(1.5, 1.5);
      polygonB.addVertex(0.5, 0.5);
      polygonB.update();

      ConcavePolygon2D mergedPolygonExpected = new ConcavePolygon2D();
      mergedPolygonExpected.addVertex(-1.0, 1.0);
      mergedPolygonExpected.addVertex(0.5, 1.0);
      mergedPolygonExpected.addVertex(0.5, 1.5);
      mergedPolygonExpected.addVertex(1.5, 1.5);
      mergedPolygonExpected.addVertex(1.0, 1.0);
      mergedPolygonExpected.addVertex(1.0, -1.0);
      mergedPolygonExpected.addVertex(-1.0, -1.0);
      mergedPolygonExpected.update();

      ConcavePolygon2D mergedPolygon = new ConcavePolygon2D();
      PolygonClippingAndMerging.merge(polygonB, polygonA, mergedPolygon);

      assertTrue(mergedPolygon.epsilonEquals(mergedPolygonExpected, 1e-7));
   }

}