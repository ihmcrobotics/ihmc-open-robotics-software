package us.ihmc.robotics.geometry.concaveHull.weilerAtherton;

import jdk.nashorn.internal.ir.annotations.Ignore;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.concaveHull.GeometryPolygonTestTools;
import us.ihmc.robotics.geometry.concavePolygon2D.ComplexPolygonException;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2D;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2DBasics;
import us.ihmc.robotics.geometry.concavePolygon2D.GeometryPolygonTools;
import us.ihmc.robotics.geometry.concavePolygon2D.weilerAtherton.PolygonClippingAndMerging;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.robotics.Assert.*;

public class PolygonClippingAndMergingTest
{
   @Test
   public void testClipCapturedRegion()
   {
      ConcavePolygon2D polygonToClip = new ConcavePolygon2D();
      polygonToClip.addVertex(0.5, 0.95);
      polygonToClip.addVertex(1.1, 0.95);
      polygonToClip.addVertex(1.1, -0.95);
      polygonToClip.addVertex(0.5, -0.95);
      polygonToClip.update();

      ConcavePolygon2D clippingPolygon = new ConcavePolygon2D();
      clippingPolygon.addVertex(-1.0, -0.1);
      clippingPolygon.addVertex(-1.1, -0.0);
      clippingPolygon.addVertex(-1.0, 0.1);
      clippingPolygon.addVertex(1.0, 0.1);
      clippingPolygon.addVertex(1.1, 0.0);
      clippingPolygon.addVertex(1.0, -0.1);
      clippingPolygon.update();

      ConcavePolygon2D clippedPolygon = new ConcavePolygon2D();
      List<ConcavePolygon2DBasics> clippedPolygons = PolygonClippingAndMerging.removeAreaInsideClip(clippingPolygon, polygonToClip);
   }

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

      List<ConcavePolygon2DBasics> clippedPolygons = PolygonClippingAndMerging.removeAreaInsideClip(clippingPolygon, polygonToClip);

      assertEquals(1, clippedPolygons.size());
      GeometryPolygonTestTools.assertConcavePolygon2DEquals(clippedPolygonExpected, clippedPolygons.get(0), 1e-7);
   }

   @Test
   public void testClippingThatBreaksPolygonInHalf()
   {
      ConcavePolygon2D polygonToClip = new ConcavePolygon2D();
      polygonToClip.addVertex(-1.0, 1.0);
      polygonToClip.addVertex(1.0, 1.0);
      polygonToClip.addVertex(1.0, -1.0);
      polygonToClip.addVertex(-1.0, -1.0);
      polygonToClip.update();

      ConcavePolygon2D clippingPolygon = new ConcavePolygon2D();
      clippingPolygon.addVertex(0.05, 2.0);
      clippingPolygon.addVertex(0.05, -2.0);
      clippingPolygon.addVertex(-0.05, -2.0);
      clippingPolygon.addVertex(-0.05, 2.0);
      clippingPolygon.update();

      assertTrue(GeometryPolygonTools.doPolygonsIntersect(polygonToClip, clippingPolygon));

      ConcavePolygon2D clippedPolygonExpected1 = new ConcavePolygon2D();
      clippedPolygonExpected1.addVertex(-1.0, 1.0);
      clippedPolygonExpected1.addVertex(-0.05, 1.0);
      clippedPolygonExpected1.addVertex(-0.05, -1.0);
      clippedPolygonExpected1.addVertex(-1.0, -1.0);
      clippedPolygonExpected1.update();

      ConcavePolygon2D clippedPolygonExpected2 = new ConcavePolygon2D();
      clippedPolygonExpected2.addVertex(0.05, 1.0);
      clippedPolygonExpected2.addVertex(1.0, 1.0);
      clippedPolygonExpected2.addVertex(1.0, -1.0);
      clippedPolygonExpected2.addVertex(0.05, -1.0);
      clippedPolygonExpected2.update();

      List<ConcavePolygon2DBasics> clippedPolygons = PolygonClippingAndMerging.removeAreaInsideClip(clippingPolygon, polygonToClip);

      assertEquals(2, clippedPolygons.size());

      double epsilon = 1e-7;
      if (clippedPolygonExpected1.epsilonEquals(clippedPolygons.get(0), epsilon))
      {
         GeometryPolygonTestTools.assertConcavePolygon2DEquals(clippedPolygonExpected1, clippedPolygons.get(0), 1e-7);
         GeometryPolygonTestTools.assertConcavePolygon2DEquals(clippedPolygonExpected2, clippedPolygons.get(1), 1e-7);
      }
      else
      {
         GeometryPolygonTestTools.assertConcavePolygon2DEquals(clippedPolygonExpected2, clippedPolygons.get(0), 1e-7);
         GeometryPolygonTestTools.assertConcavePolygon2DEquals(clippedPolygonExpected1, clippedPolygons.get(1), 1e-7);
      }
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

      List<ConcavePolygon2DBasics> clippedPolygons = PolygonClippingAndMerging.removeAreaInsideClip(clippingPolygon, polygonToClip);

      assertEquals(1, clippedPolygons.size());
      GeometryPolygonTestTools.assertConcavePolygon2DEquals(clippedPolygonExpected, clippedPolygons.get(0), 1e-7);
   }

   @Test
   public void testClipRemoveSquareChunkFromCornerOfSquare2()
   {
      ConcavePolygon2D polygonToClip = new ConcavePolygon2D();
      polygonToClip.addVertex(-1.0, 1.0);
      polygonToClip.addVertex(1.0, 1.0);
      polygonToClip.addVertex(0.75, 0.5);
      polygonToClip.addVertex(1.0, -1.0);
      polygonToClip.addVertex(-1.0, -1.0);
      polygonToClip.update();

      assertTrue(polygonToClip.isPointInside(0.5, 0.5));

      ConcavePolygon2D clippingPolygon = new ConcavePolygon2D();
      clippingPolygon.addVertex(0.5, 1.5);
      clippingPolygon.addVertex(1.5, 1.5);
      clippingPolygon.addVertex(1.5, 0.5);
      clippingPolygon.addVertex(0.5, 0.5);
      clippingPolygon.update();

      assertTrue(GeometryPolygonTools.doPolygonsIntersect(polygonToClip, clippingPolygon));

      ConcavePolygon2D clippedPolygonExpected = new ConcavePolygon2D();
      clippedPolygonExpected.addVertex(-1.0, 1.0);
      clippedPolygonExpected.addVertex(0.5, 1.0);
      clippedPolygonExpected.addVertex(0.5, 0.5);
      clippedPolygonExpected.addVertex(0.75, 0.5);
      clippedPolygonExpected.addVertex(1.0, -1.0);
      clippedPolygonExpected.addVertex(-1.0, -1.0);
      clippedPolygonExpected.update();

      List<ConcavePolygon2DBasics> clippedPolygons = PolygonClippingAndMerging.removeAreaInsideClip(clippingPolygon, polygonToClip);

      assertEquals(1, clippedPolygons.size());
      GeometryPolygonTestTools.assertConcavePolygon2DEquals(clippedPolygonExpected, clippedPolygons.get(0), 1e-7);
   }

   @Test
   public void testMergeAllPossible()
   {
      ConcavePolygon2D polygon1 = new ConcavePolygon2D();
      polygon1.addVertex(-0.1, 1.0);
      polygon1.addVertex(0.1, 1.0);
      polygon1.addVertex(0.1, -1.0);
      polygon1.addVertex(-0.1, -1.0);
      polygon1.update();

      ConcavePolygon2D polygon2 = new ConcavePolygon2D();
      polygon2.addVertex(0.9, 1.0);
      polygon2.addVertex(1.1, 1.0);
      polygon2.addVertex(1.1, -1.0);
      polygon2.addVertex(0.9, -1.0);
      polygon2.update();

      ConcavePolygon2D polygon3 = new ConcavePolygon2D();
      polygon3.addVertex(-0.09, 0.1);
      polygon3.addVertex(1.0, 0.1);
      polygon3.addVertex(1.0, -0.1);
      polygon3.addVertex(-0.09, -0.1);
      polygon3.update();

      List<ConcavePolygon2DBasics> polygons = new ArrayList<>();
      polygons.add(polygon1);
      polygons.add(polygon2);
      polygons.add(polygon3);

      ConcavePolygon2D mergedPolygonExpected = new ConcavePolygon2D();
      mergedPolygonExpected.addVertex(-0.1, 1.0);
      mergedPolygonExpected.addVertex(0.1, 1.0);
      mergedPolygonExpected.addVertex(0.1, 0.1);
      mergedPolygonExpected.addVertex(0.9, 0.1);
      mergedPolygonExpected.addVertex(0.9, 1.0);
      mergedPolygonExpected.addVertex(1.1, 1.0);
      mergedPolygonExpected.addVertex(1.1, -1.0);
      mergedPolygonExpected.addVertex(0.9, -1.0);
      mergedPolygonExpected.addVertex(0.9, -0.1);
      mergedPolygonExpected.addVertex(0.1, -0.1);
      mergedPolygonExpected.addVertex(0.1, -1.0);
      mergedPolygonExpected.addVertex(-0.1, -1.0);
      mergedPolygonExpected.update();

      ConcavePolygon2D mergedPolygonOther1 = new ConcavePolygon2D();
      ConcavePolygon2D mergedPolygonOther2 = new ConcavePolygon2D();
      PolygonClippingAndMerging.merge(polygon1, polygon3, mergedPolygonOther1);
      PolygonClippingAndMerging.merge(mergedPolygonOther1, polygon2, mergedPolygonOther2);

      PolygonClippingAndMerging.mergeAllPossible(polygons);

      assertEquals(1, polygons.size());
      //
      GeometryPolygonTestTools.assertConcavePolygon2DEquals(mergedPolygonExpected, mergedPolygonOther2, 1e-7);
      GeometryPolygonTestTools.assertConcavePolygon2DEquals(mergedPolygonExpected, polygons.get(0), 1e-7);

      polygon1 = new ConcavePolygon2D();
      polygon1.addVertex(-0.1, 1.0);
      polygon1.addVertex(0.1, 1.0);
      polygon1.addVertex(0.1, -1.0);
      polygon1.addVertex(-0.1, -1.0);
      polygon1.update();

      polygon2 = new ConcavePolygon2D();
      polygon2.addVertex(0.9, 0.1);
      polygon2.addVertex(1.0, 0.1);
      polygon2.addVertex(1.0, -0.1);
      polygon2.addVertex(0.9, -0.1);
      polygon2.update();

      polygon3 = new ConcavePolygon2D();
      polygon3.addVertex(-0.1, 0.1);
      polygon3.addVertex(1.0, 0.1);
      polygon3.addVertex(1.0, -0.1);
      polygon3.addVertex(-0.1, -0.1);
      polygon3.update();

      mergedPolygonExpected.clear();
      mergedPolygonExpected.addVertex(-0.1, 1.0);
      mergedPolygonExpected.addVertex(0.1, 1.0);
      mergedPolygonExpected.addVertex(0.1, 0.1);
      mergedPolygonExpected.addVertex(0.9, 0.1);
      mergedPolygonExpected.addVertex(1.0, 0.1);
      mergedPolygonExpected.addVertex(1.0, -0.1);
      mergedPolygonExpected.addVertex(0.9, -0.1);
      mergedPolygonExpected.addVertex(0.1, -0.1);
      mergedPolygonExpected.addVertex(0.1, -1.0);
      mergedPolygonExpected.addVertex(-0.1, -1.0);
      mergedPolygonExpected.addVertex(-0.1, -0.1);
      mergedPolygonExpected.addVertex(-0.1, 0.1);
      mergedPolygonExpected.update();

      polygons = new ArrayList<>();
      polygons.add(polygon1);
      polygons.add(polygon2);
      polygons.add(polygon3);

      PolygonClippingAndMerging.merge(polygon1, polygon3, mergedPolygonOther1);

      PolygonClippingAndMerging.mergeAllPossible(polygons);

      assertEquals(1, polygons.size());

      GeometryPolygonTestTools.assertConcavePolygon2DEquals(mergedPolygonExpected, polygons.get(0), 1e-7);
   }

   @Test
   public void testMergeAll()
   {
      ConcavePolygon2D blockPolygon1 = new ConcavePolygon2D();
      blockPolygon1.addVertex(-0.1, 0.2);
      blockPolygon1.addVertex(-0.1, -0.2);
      blockPolygon1.addVertex(-0.2, -0.2);
      blockPolygon1.addVertex(-0.2, 0.2);
      blockPolygon1.update();

      ConcavePolygon2D blockPolygon2 = new ConcavePolygon2D();
      blockPolygon2.addVertex(0.1, 0.2);
      blockPolygon2.addVertex(0.2, 0.2);
      blockPolygon2.addVertex(0.2, -0.2);
      blockPolygon2.addVertex(0.1, -0.2);
      blockPolygon2.update();

      ConcavePolygon2D blockPolygon3 = new ConcavePolygon2D();
      blockPolygon3.addVertex(-0.2, -0.1);
      blockPolygon3.addVertex(0.2, -0.1);
      blockPolygon3.addVertex(0.2, -0.2);
      blockPolygon3.addVertex(-0.2, -0.2);
      blockPolygon3.update();

      ConcavePolygon2D blockPolygon4 = new ConcavePolygon2D();
      blockPolygon4.addVertex(-0.2, 0.2);
      blockPolygon4.addVertex(0.2, 0.2);
      blockPolygon4.addVertex(0.2, 0.1);
      blockPolygon4.addVertex(-0.2, 0.1);
      blockPolygon4.update();

      List<ConcavePolygon2DBasics> polygons = new ArrayList<>();
      polygons.add(blockPolygon1);
      polygons.add(blockPolygon2);
      polygons.add(blockPolygon3);
      polygons.add(blockPolygon4);

      PolygonClippingAndMerging.mergeAllPossible(polygons);

      ConcavePolygon2D mergedPolygonExpected = new ConcavePolygon2D();
      mergedPolygonExpected.addVertex(-0.2, 0.2);
      mergedPolygonExpected.addVertex(0.2, 0.2);
      mergedPolygonExpected.addVertex(0.2, -0.2);
      mergedPolygonExpected.addVertex(-0.2, -0.2);
      mergedPolygonExpected.update();

      if (polygons.size() > 1)
      {
         LogTools.info("Polygons intersect : " + GeometryPolygonTools.doPolygonsIntersect(polygons.get(0), polygons.get(1)));
      }
      assertEquals(1, polygons.size());
      GeometryPolygonTestTools.assertConcavePolygon2DEquals(mergedPolygonExpected, polygons.get(0), 1e-7);

      ConcavePolygon2D extrudedBlockPolygon1 = new ConcavePolygon2D();
      extrudedBlockPolygon1.addVertex(0.0, 0.2);
      extrudedBlockPolygon1.addVertex(0.0, -0.2);
      extrudedBlockPolygon1.addVertex(-0.1, -0.3);
      extrudedBlockPolygon1.addVertex(-0.2, -0.3);
      extrudedBlockPolygon1.addVertex(-0.3, -0.2);
      extrudedBlockPolygon1.addVertex(-0.3, 0.2);
      extrudedBlockPolygon1.addVertex(-0.2, 0.3);
      extrudedBlockPolygon1.addVertex(-0.1, 0.3);
      extrudedBlockPolygon1.update();

      ConcavePolygon2D extrudedBlockPolygon2 = new ConcavePolygon2D();
      extrudedBlockPolygon2.addVertex(0.0, 0.2);
      extrudedBlockPolygon2.addVertex(0.0, -0.2);
      extrudedBlockPolygon2.addVertex(-0.1, -0.3);
      extrudedBlockPolygon2.addVertex(-0.2, -0.3);
      extrudedBlockPolygon2.addVertex(-0.3, -0.2);
      extrudedBlockPolygon2.addVertex(-0.3, 0.2);
      extrudedBlockPolygon2.addVertex(-0.2, 0.3);
      extrudedBlockPolygon2.addVertex(-0.1, 0.3);
      extrudedBlockPolygon2.update();

      ConcavePolygon2D extrudedBlockPolygon3 = new ConcavePolygon2D();
      extrudedBlockPolygon3.addVertex(-0.3, 0.2);
      extrudedBlockPolygon3.addVertex(-0.2, 0.3);
      extrudedBlockPolygon3.addVertex(0.2, 0.3);
      extrudedBlockPolygon3.addVertex(0.3, 0.2);
      extrudedBlockPolygon3.addVertex(0.3, 0.1);
      extrudedBlockPolygon3.addVertex(0.2, 0.0);
      extrudedBlockPolygon3.addVertex(-0.2, 0.0);
      extrudedBlockPolygon3.addVertex(-0.3, 0.1);
      extrudedBlockPolygon3.update();

      ConcavePolygon2D extrudedBlockPolygon4 = new ConcavePolygon2D();
      extrudedBlockPolygon4.addVertex(-0.3, -0.2);
      extrudedBlockPolygon4.addVertex(-0.3, -0.1);
      extrudedBlockPolygon4.addVertex(-0.2, 0.0);
      extrudedBlockPolygon4.addVertex(0.2, 0.0);
      extrudedBlockPolygon4.addVertex(0.3, -0.1);
      extrudedBlockPolygon4.addVertex(0.3, -0.2);
      extrudedBlockPolygon4.addVertex(0.2, -0.3);
      extrudedBlockPolygon4.addVertex(-0.2, -0.3);
      extrudedBlockPolygon4.update();

      polygons = new ArrayList<>();
      polygons.add(extrudedBlockPolygon1);
      polygons.add(extrudedBlockPolygon2);
      polygons.add(extrudedBlockPolygon3);
      polygons.add(extrudedBlockPolygon4);

      PolygonClippingAndMerging.mergeAllPossible(polygons);

      mergedPolygonExpected = new ConcavePolygon2D();
      mergedPolygonExpected.addVertex(-0.2, 0.3);
      mergedPolygonExpected.addVertex(0.2, 0.3);
      mergedPolygonExpected.addVertex(0.3, 0.2);
      mergedPolygonExpected.addVertex(0.3, 0.1);
      mergedPolygonExpected.addVertex(0.2, 0.0);
      mergedPolygonExpected.addVertex(0.3, -0.1);
      mergedPolygonExpected.addVertex(0.3, -0.2);
      mergedPolygonExpected.addVertex(0.2, -0.3);
      mergedPolygonExpected.addVertex(-0.2, -0.3);
      mergedPolygonExpected.addVertex(-0.3, -0.2);
      mergedPolygonExpected.addVertex(-0.3, -0.1);
      mergedPolygonExpected.addVertex(-0.2, -0.0);
      mergedPolygonExpected.addVertex(-0.3, 0.1);
      mergedPolygonExpected.addVertex(-0.3, 0.2);
      mergedPolygonExpected.update();

      assertEquals(1, polygons.size());
      GeometryPolygonTestTools.assertConcavePolygon2DEquals(mergedPolygonExpected, polygons.get(0), 1e-7);
   }

   @Test
   public void testAwfulMerge()
   {
      ConcavePolygon2D polygon1 = new ConcavePolygon2D();
      polygon1.addVertex(-0.1, 1.0);
      polygon1.addVertex(0.1, 1.0);
      polygon1.addVertex(0.1, -1.0);
      polygon1.addVertex(-0.1, -1.0);
      polygon1.update();

      ConcavePolygon2D polygon3 = new ConcavePolygon2D();
      polygon3.addVertex(-0.1, 0.1);
      polygon3.addVertex(1.0, 0.1);
      polygon3.addVertex(1.0, -0.1);
      polygon3.addVertex(-0.1, -0.1);
      polygon3.update();

      ConcavePolygon2D mergedPolygon = new ConcavePolygon2D();

      PolygonClippingAndMerging.merge(polygon1, polygon3, mergedPolygon);
   }

   @Test
   public void testMergingCreatesHole()
   {
      ConcavePolygon2D uPolygon = new ConcavePolygon2D();
      uPolygon.addVertex(-1.0, 1.0);
      uPolygon.addVertex(-0.9, 1.0);
      uPolygon.addVertex(-0.9, -0.9);
      uPolygon.addVertex(0.9, -0.9);
      uPolygon.addVertex(0.9, 1.0);
      uPolygon.addVertex(1.0, 1.0);
      uPolygon.addVertex(1.0, -1.0);
      uPolygon.addVertex(-1.0, -1.0);
      uPolygon.update();

      ConcavePolygon2D hat = new ConcavePolygon2D();
      hat.addVertex(-1.0, 1.0);
      hat.addVertex(1.0, 1.0);
      hat.addVertex(1.0, 0.9);
      hat.addVertex(-1.0, 0.9);
      hat.update();

      ConcavePolygon2D mergedOuterExpected = new ConcavePolygon2D();
      mergedOuterExpected.addVertex(-1.0, 1.0);
      mergedOuterExpected.addVertex(-0.9, 1.0);
      mergedOuterExpected.addVertex(0.9, 1.0);
      mergedOuterExpected.addVertex(1.0, 1.0);
      mergedOuterExpected.addVertex(1.0, 0.9);
      mergedOuterExpected.addVertex(1.0, -1.0);
      mergedOuterExpected.addVertex(-1.0, -1.0);
      mergedOuterExpected.addVertex(-1.0, 0.9);
      mergedOuterExpected.update();

      ConcavePolygon2D holeExpected = new ConcavePolygon2D();
      holeExpected.addVertex(-0.9, 0.9);
      holeExpected.addVertex(0.9, 0.9);
      holeExpected.addVertex(0.9, -0.9);
      holeExpected.addVertex(-0.9, -0.9);
      holeExpected.update();

      ConcavePolygon2D mergedOuter = new ConcavePolygon2D();
      PolygonClippingAndMerging.merge(uPolygon, hat, mergedOuter);

      //      assertEquals(1, holes.size());

      GeometryPolygonTestTools.assertConcavePolygon2DEquals(mergedOuterExpected, mergedOuter, 1e-7);
      //      GeometryPolygonTestTools.assertConcavePolygon2DEquals(holeExpected, holes.get(0), 1e-7);

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

      List<ConcavePolygon2DBasics> clippedPolygons = PolygonClippingAndMerging.removeAreaInsideClip(clippingPolygon, polygonToClip);

      assertEquals(1, clippedPolygons.size());
      GeometryPolygonTestTools.assertConcavePolygon2DEquals(clippedPolygonExpected, clippedPolygons.get(0), 1e-7);
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

      List<ConcavePolygon2DBasics> clippedPolygons = PolygonClippingAndMerging.removeAreaInsideClip(clippingPolygon, polygonToClip);

      assertEquals(1, clippedPolygons.size());
      GeometryPolygonTestTools.assertConcavePolygon2DEquals(clippedPolygonExpected, clippedPolygons.get(0), 1e-7);

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

      clippedPolygons = PolygonClippingAndMerging.removeAreaInsideClip(clippingPolygon, polygonToClip);

      assertEquals(1, clippedPolygons.size());
      GeometryPolygonTestTools.assertConcavePolygon2DEquals(clippedPolygonExpected, clippedPolygons.get(0), 1e-7);
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
      //      PolygonClippingAndMerging.merge(polygonA, polygonB, mergedPolygon);

      //      assertTrue(mergedPolygon.epsilonEquals(mergedPolygonExpected, 1e-7));

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
   public void testClippingWithOneCommonVertex()
   {
      ConcavePolygon2D polygonToClip = new ConcavePolygon2D();
      polygonToClip.addVertex(1.0, 1.0);
      polygonToClip.addVertex(1.0, -1.0);
      polygonToClip.addVertex(-1.0, -1.0);
      polygonToClip.addVertex(-1.0, 1.0);
      polygonToClip.update();

      ConcavePolygon2D clippingPolygon = new ConcavePolygon2D();
      clippingPolygon.addVertex(-0.5, 1.5);
      clippingPolygon.addVertex(0.5, 1.5);
      clippingPolygon.addVertex(0, 1.0);
      clippingPolygon.update();

      ConcavePolygon2D clippedPolygonExpected = new ConcavePolygon2D();
      clippedPolygonExpected.addVertex(1.0, 1.0);
      clippedPolygonExpected.addVertex(1.0, -1.0);
      clippedPolygonExpected.addVertex(-1.0, -1.0);
      clippedPolygonExpected.addVertex(-1.0, 1.0);
      clippedPolygonExpected.addVertex(0.0, 1.0);
      clippedPolygonExpected.update();

      List<ConcavePolygon2DBasics> clippedPolygons = PolygonClippingAndMerging.removeAreaInsideClip(clippingPolygon, polygonToClip);

      assertEquals(1, clippedPolygons.size());

      GeometryPolygonTestTools.assertConcavePolygon2DEquals(clippedPolygonExpected, clippedPolygons.get(0), 1e-7);
   }

   @Test
   public void testMergingWithOneCommonVertex()
   {
      ConcavePolygon2D polygonToClip = new ConcavePolygon2D();
      polygonToClip.addVertex(1.0, 1.0);
      polygonToClip.addVertex(1.0, -1.0);
      polygonToClip.addVertex(-1.0, -1.0);
      polygonToClip.addVertex(-1.0, 1.0);
      polygonToClip.update();

      ConcavePolygon2D clippingPolygon = new ConcavePolygon2D();
      clippingPolygon.addVertex(-0.5, 1.5);
      clippingPolygon.addVertex(0.5, 1.5);
      clippingPolygon.addVertex(0, 1.0);
      clippingPolygon.update();

      ConcavePolygon2D mergedPolygonExpected = new ConcavePolygon2D(clippingPolygon);
      ConcavePolygon2D mergedPolygon = new ConcavePolygon2D();
      assertTrue(GeometryPolygonTools.doPolygonsIntersect(polygonToClip, clippingPolygon));
      PolygonClippingAndMerging.merge(clippingPolygon, polygonToClip, mergedPolygon);

      GeometryPolygonTestTools.assertConcavePolygon2DEquals(mergedPolygonExpected, mergedPolygon, 1e-7);
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

      List<ConcavePolygon2DBasics> clippedPolygons = PolygonClippingAndMerging.removeAreaInsideClip(clippingPolygon, polygonToClip);

      assertEquals(1, clippedPolygons.size());
      GeometryPolygonTestTools.assertConcavePolygon2DEquals(clippedPolygonExpected, clippedPolygons.get(0), 1e-7);
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
      try
      {
         PolygonClippingAndMerging.merge(polygonB, polygonA, mergedPolygon);
      }
      catch (ComplexPolygonException e)
      {
         PolygonClippingAndMerging.merge(polygonB, polygonA, mergedPolygon);
      }

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

      List<ConcavePolygon2DBasics> clippedPolygons = PolygonClippingAndMerging.removeAreaInsideClip(clippingPolygon, polygonToClip);

      assertEquals(1, clippedPolygons.size());
      GeometryPolygonTestTools.assertConcavePolygon2DEquals(clippedPolygonExpected, clippedPolygons.get(0), 1e-7);
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

   /**
    *  This one currently doesn't work. That's because there's no place to start, since no point is outside either polygon. Neither polygon is inside the other one
    */
   @Disabled
   @Test
   public void testMergingIdenticalPolygons()
   {
      ConcavePolygon2D polygonA = new ConcavePolygon2D();
      polygonA.addVertex(-1.0, 1.0);
      polygonA.addVertex(1.0, 1.0);
      polygonA.addVertex(1.0, -1.0);
      polygonA.addVertex(-1.0, -1.0);
      polygonA.update();

      ConcavePolygon2D polygonB = new ConcavePolygon2D();
      polygonB.addVertex(-1.0, 1.0);
      polygonB.addVertex(0.0, 1.0);
      polygonB.addVertex(1.0, 1.0);
      polygonB.addVertex(1.0, 0.0);
      polygonB.addVertex(1.0, -1.0);
      polygonB.addVertex(0.0, -1.0);
      polygonB.addVertex(-1.0, -1.0);
      polygonB.addVertex(-1.0, 0.0);
      polygonB.update();

      ConcavePolygon2D mergedPolygonExpected = new ConcavePolygon2D(polygonB);
      ConcavePolygon2D mergedPolygon = new ConcavePolygon2D();

      PolygonClippingAndMerging.merge(polygonA, polygonB, mergedPolygon);

      GeometryPolygonTestTools.assertConcavePolygon2DEquals(mergedPolygonExpected, mergedPolygon, 1e-7);
   }

   @Test
   public void testMergeL()
   {
      ConcavePolygon2D polygonA = new ConcavePolygon2D();
      polygonA.addVertex(-1.0, -1.0);
      polygonA.addVertex(-1.0, 1.0);
      polygonA.addVertex(-0.9, 1.0);
      polygonA.addVertex(-0.9, -1.0);
      polygonA.update();

      ConcavePolygon2D polygonB = new ConcavePolygon2D();
      polygonB.addVertex(-1.0, 1.0);
      polygonB.addVertex(1.0, 1.0);
      polygonB.addVertex(1.0, 0.9);
      polygonB.addVertex(-1.0, 0.9);
      polygonB.update();

      ConcavePolygon2D mergedPolygonExpected = new ConcavePolygon2D();
      mergedPolygonExpected.addVertex(-1.0, -1.0);
      mergedPolygonExpected.addVertex(-1.0, 1.0);
      mergedPolygonExpected.addVertex(1.0, 1.0);
      mergedPolygonExpected.addVertex(1.0, 0.9);
      mergedPolygonExpected.addVertex(-0.9, 0.9);
      mergedPolygonExpected.addVertex(-0.9, -1.0);
      mergedPolygonExpected.update();

      ConcavePolygon2D mergedPolygon = new ConcavePolygon2D();
      PolygonClippingAndMerging.merge(polygonA, polygonB, mergedPolygon);

      GeometryPolygonTestTools.assertConcavePolygon2DEquals(mergedPolygonExpected, mergedPolygon, 1e-8);
   }

   @Test
   public void testClipL()
   {
      ConcavePolygon2D polygonA = new ConcavePolygon2D();
      polygonA.addVertex(-1.0, -1.0);
      polygonA.addVertex(-1.0, 1.0);
      polygonA.addVertex(-0.9, 1.0);
      polygonA.addVertex(-0.9, -1.0);
      polygonA.update();

      ConcavePolygon2D polygonB = new ConcavePolygon2D();
      polygonB.addVertex(-1.0, 1.0);
      polygonB.addVertex(1.0, 1.0);
      polygonB.addVertex(1.0, 0.9);
      polygonB.addVertex(-1.0, 0.9);
      polygonB.update();

      ConcavePolygon2D clippedPolygonExpected = new ConcavePolygon2D();
      clippedPolygonExpected.addVertex(-1.0, -1.0);
      clippedPolygonExpected.addVertex(-1.0, 0.9);
      clippedPolygonExpected.addVertex(-0.9, 0.9);
      clippedPolygonExpected.addVertex(-0.9, -1.0);
      clippedPolygonExpected.update();

      List<ConcavePolygon2DBasics> clippedPolygons = PolygonClippingAndMerging.removeAreaInsideClip(polygonB, polygonA);

      assertEquals(1, clippedPolygons.size());
      GeometryPolygonTestTools.assertConcavePolygon2DEquals(clippedPolygonExpected, clippedPolygons.get(0), 1e-8);
   }

   @Test
   public void testClippingWeirdU()
   {
      ConcavePolygon2D uPolygon = new ConcavePolygon2D();
      uPolygon.addVertex(-1.0, 1.0);
      uPolygon.addVertex(-0.9, 1.0);
      uPolygon.addVertex(-0.9, -0.9);
      uPolygon.addVertex(0.9, -0.9);
      uPolygon.addVertex(0.9, 1.0);
      uPolygon.addVertex(1.0, 1.0);
      uPolygon.addVertex(1.0, -1.0);
      uPolygon.addVertex(-1.0, -1.0);
      uPolygon.update();

      ConcavePolygon2D hat = new ConcavePolygon2D();
      hat.addVertex(-1.0, 1.0);
      hat.addVertex(1.0, 1.0);
      hat.addVertex(1.0, 0.9);
      hat.addVertex(-1.0, 0.9);
      hat.update();

      ConcavePolygon2D clippedPolygonExpected = new ConcavePolygon2D();
      clippedPolygonExpected.addVertex(-1.0, 0.9);
      clippedPolygonExpected.addVertex(-0.9, 0.9);
      clippedPolygonExpected.addVertex(-0.9, -0.9);
      clippedPolygonExpected.addVertex(0.9, -0.9);
      clippedPolygonExpected.addVertex(0.9, 0.9);
      clippedPolygonExpected.addVertex(1.0, 0.9);
      clippedPolygonExpected.addVertex(1.0, -1.0);
      clippedPolygonExpected.addVertex(-1.0, -1.0);
      clippedPolygonExpected.update();

      List<ConcavePolygon2DBasics> clippedPolygons = PolygonClippingAndMerging.removeAreaInsideClip(hat, uPolygon);

      assertEquals(1, clippedPolygons.size());
      GeometryPolygonTestTools.assertConcavePolygon2DEquals(clippedPolygonExpected, clippedPolygons.get(0), 1e-7);

      // TODO this is an edge case that currently doesn't work.
      /*
      clippedPolygonExpected = new ConcavePolygon2D();
      clippedPolygonExpected.addVertex(-0.9, 1.0);
      clippedPolygonExpected.addVertex(0.9, 1.0);
      clippedPolygonExpected.addVertex(0.9, 0.9);
      clippedPolygonExpected.addVertex(-0.9, 0.9);
      clippedPolygonExpected.update();

      clippedPolygons = PolygonClippingAndMerging.removeAreaInsideClip(uPolygon, hat);

      assertEquals(1, clippedPolygons.size());
      GeometryPolygonTestTools.assertConcavePolygon2DEquals(clippedPolygonExpected, clippedPolygons.get(0), 1e-7);

       */
   }

   @Test
   public void testTrickyCut()
   {
      ConcavePolygon2D polygonToClip = new ConcavePolygon2D();
      polygonToClip.addVertex(1.0, 1.0);
      polygonToClip.addVertex(1.0, -1.0);
      polygonToClip.addVertex(-1.0, -1.0);
      polygonToClip.addVertex(-1.0, -0.1);
      polygonToClip.addVertex(0.0, -0.1);
      polygonToClip.addVertex(0.1, 0.0);
      polygonToClip.addVertex(0.0, 0.1);
      polygonToClip.addVertex(-1.0, 0.1);
      polygonToClip.addVertex(-1.0, 1.0);
      polygonToClip.update();

      ConcavePolygon2D clippingPolygon = new ConcavePolygon2D();
      clippingPolygon.addVertex(-0.1, 1.0);
      clippingPolygon.addVertex(0.0, 1.1);
      clippingPolygon.addVertex(0.1, 1.0);
      clippingPolygon.addVertex(0.1, -1.0);
      clippingPolygon.addVertex(0.0, -1.1);
      clippingPolygon.addVertex(-0.1, -1.0);
      clippingPolygon.update();

      List<ConcavePolygon2DBasics> clippedPolygons = PolygonClippingAndMerging.removeAreaInsideClip(clippingPolygon, polygonToClip);

      ConcavePolygon2D clippedPolygonExpected1 = new ConcavePolygon2D();
      clippedPolygonExpected1.addVertex(1.0, 1.0);
      clippedPolygonExpected1.addVertex(1.0, -1.0);
      clippedPolygonExpected1.addVertex(0.1, -1.0);
      clippedPolygonExpected1.addVertex(0.1, 1.0);
      clippedPolygonExpected1.update();

      ConcavePolygon2D clippedPolygonExpected2 = new ConcavePolygon2D();
      clippedPolygonExpected2.addVertex(-1.0, 1.0);
      clippedPolygonExpected2.addVertex(-0.1, 1.0);
      clippedPolygonExpected2.addVertex(-0.1, 0.1);
      clippedPolygonExpected2.addVertex(-1.0, 0.1);
      clippedPolygonExpected2.update();

      ConcavePolygon2D clippedPolygonExpected3 = new ConcavePolygon2D();
      clippedPolygonExpected3.addVertex(-1.0, -1.0);
      clippedPolygonExpected3.addVertex(-1.0, -0.1);
      clippedPolygonExpected3.addVertex(-0.1, -0.1);
      clippedPolygonExpected3.addVertex(-0.1, -1.0);
      clippedPolygonExpected3.update();

      boolean has1 = false;
      boolean has2 = false;
      boolean has3 = false;
      for (int i = 0; i < 3; i++)
      {
         if (clippedPolygonExpected1.epsilonEquals(clippedPolygons.get(i), 1e-7))
            has1 = true;
         if (clippedPolygonExpected2.epsilonEquals(clippedPolygons.get(i), 1e-7))
            has2 = true;
         if (clippedPolygonExpected3.epsilonEquals(clippedPolygons.get(i), 1e-7))
            has3 = true;
      }
      assertTrue(has1);
      assertTrue(has2);
      assertTrue(has3);
   }

   @Test
   public void testDumbMerge()
   {
      ConcavePolygon2D wall1 = new ConcavePolygon2D();
      wall1.addVertex(-1.1, 0.0);
      wall1.addVertex(-1.0, 0.1);
      wall1.addVertex(0.0, 0.1);
      wall1.addVertex(0.1, 0.0);
      wall1.addVertex(0.0, -0.1);
      wall1.addVertex(-1.0, -0.1);
      wall1.update();

      ConcavePolygon2D wall2 = new ConcavePolygon2D();
      wall2.addVertex(0.0, 1.1);
      wall2.addVertex(0.1, 1.0);
      wall2.addVertex(0.1, -1.0);
      wall2.addVertex(0.0, -1.1);
      wall2.addVertex(-0.1, -1.0);
      wall2.addVertex(-0.1, 1.0);
      wall2.update();

      ConcavePolygon2D mergedPolygonExpected = new ConcavePolygon2D();
      ConcavePolygon2D mergedPolygon = new ConcavePolygon2D();

      mergedPolygonExpected.addVertex(-0.1, 1.0);
      mergedPolygonExpected.addVertex(0.0, 1.1);
      mergedPolygonExpected.addVertex(0.1, 1.0);
      mergedPolygonExpected.addVertex(0.1, -1.0);
      mergedPolygonExpected.addVertex(0.0, -1.1);
      mergedPolygonExpected.addVertex(-0.1, -1.0);
      mergedPolygonExpected.addVertex(-0.1, -0.1);
      mergedPolygonExpected.addVertex(-1.0, -0.1);
      mergedPolygonExpected.addVertex(-1.1, 0.0);
      mergedPolygonExpected.addVertex(-1.0, 0.1);
      mergedPolygonExpected.addVertex(-0.1, 0.1);
      mergedPolygonExpected.update();

      PolygonClippingAndMerging.merge(wall1, wall2, mergedPolygon);

      GeometryPolygonTestTools.assertConcavePolygon2DEquals(mergedPolygonExpected, mergedPolygon, 1e-7);

      PolygonClippingAndMerging.merge(wall2, wall1, mergedPolygon);

      GeometryPolygonTestTools.assertConcavePolygon2DEquals(mergedPolygonExpected, mergedPolygon, 1e-7);
   }
}