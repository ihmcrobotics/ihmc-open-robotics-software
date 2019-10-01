package us.ihmc.pathPlanning.visibilityGraphs.tools;

import static org.junit.jupiter.api.Assertions.*;

import java.util.ArrayList;
import java.util.concurrent.CountDownLatch;

import org.apache.commons.lang3.tuple.Pair;
import org.junit.jupiter.api.Test;

import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javafx.applicationCreator.JavaFXApplicationCreator;
import us.ihmc.pathPlanning.visibilityGraphs.ui.graphics.PlanarRegionsGraphic;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class ConcaveHullMergerTest
{
   @Test
   public void testMergePlanarRegions()
   {
      Point2D pointA0 = new Point2D(0.0, 0.0);
      Point2D pointA1 = new Point2D(0.0, 1.0);
      Point2D pointA2 = new Point2D(1.0, 1.0);
      Point2D pointA3 = new Point2D(1.0, 0.0);

      Point2D pointB0 = new Point2D(-0.5, -0.5);
      Point2D pointB1 = new Point2D(-0.49, 0.5);
      Point2D pointB2 = new Point2D(0.5, 0.5);
      Point2D pointB3 = new Point2D(0.5, -0.5);

      ConvexPolygon2D polygonA = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(pointA0, pointA1, pointA2, pointA3));
      ConvexPolygon2D polygonB = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(pointB0, pointB1, pointB2, pointB3));

      PlanarRegion regionA = new PlanarRegion(new RigidBodyTransform(), polygonA);
      PlanarRegion regionB = new PlanarRegion(new RigidBodyTransform(), polygonB);

      ArrayList<PlanarRegion> mergedPlanarRegions = ConcaveHullMerger.mergePlanarRegions(regionA, regionB, 1.0);
      PlanarRegion mergedPlanarRegion = mergedPlanarRegions.get(0);

      Point2D[] concaveHull = mergedPlanarRegion.getConcaveHull();
      assertEquals(8, concaveHull.length);

      int i = 0;
      double epsilon = 1e-7;
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointB0, concaveHull[i++], epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointB1, concaveHull[i++], epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(0.0, 0.5), concaveHull[i++], epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointA1, concaveHull[i++], epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointA2, concaveHull[i++], epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointA3, concaveHull[i++], epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(0.5, 0.0), concaveHull[i++], epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointB3, concaveHull[i++], epsilon);

      assertEquals(3, mergedPlanarRegion.getNumberOfConvexPolygons());
   }

   @Test
   public void testMergeSquareInsideAnotherSquare()
   {
      Point2D pointA0 = new Point2D(0.0, 0.0);
      Point2D pointA1 = new Point2D(0.0, 1.0);
      Point2D pointA2 = new Point2D(1.0, 1.0);
      Point2D pointA3 = new Point2D(1.0, 0.0);

      Point2D pointB0 = new Point2D(0.2, 0.2);
      Point2D pointB1 = new Point2D(0.2, 0.8);
      Point2D pointB2 = new Point2D(0.8, 0.8);
      Point2D pointB3 = new Point2D(0.8, 0.2);

      ConvexPolygon2D polygonA = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(pointA0, pointA1, pointA2, pointA3));
      ConvexPolygon2D polygonB = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(pointB0, pointB1, pointB2, pointB3));

      PlanarRegion regionA = new PlanarRegion(new RigidBodyTransform(), polygonA);
      PlanarRegion regionB = new PlanarRegion(new RigidBodyTransform(), polygonB);

      ArrayList<PlanarRegion> mergedPlanarRegions = ConcaveHullMerger.mergePlanarRegions(regionA, regionB, 1.0);
      PlanarRegion mergedPlanarRegion = mergedPlanarRegions.get(0);

      Point2D[] concaveHull = mergedPlanarRegion.getConcaveHull();
      assertEquals(4, concaveHull.length);

      assertConcaveHullContains(concaveHull, pointA0);
      assertConcaveHullContains(concaveHull, pointA1);
      assertConcaveHullContains(concaveHull, pointA2);
      assertConcaveHullContains(concaveHull, pointA3);

      assertEquals(1, mergedPlanarRegion.getNumberOfConvexPolygons());

      mergedPlanarRegions = ConcaveHullMerger.mergePlanarRegions(regionB, regionA, 1.0);
      mergedPlanarRegion = mergedPlanarRegions.get(0);

      concaveHull = mergedPlanarRegion.getConcaveHull();
      assertEquals(4, concaveHull.length);

      assertConcaveHullContains(concaveHull, pointA0);
      assertConcaveHullContains(concaveHull, pointA1);
      assertConcaveHullContains(concaveHull, pointA2);
      assertConcaveHullContains(concaveHull, pointA3);

      assertEquals(1, mergedPlanarRegion.getNumberOfConvexPolygons());
   }

   @Test
   public void testMergeSquaresClearlyNotIntersecting()
   {
      Point2D pointA0 = new Point2D(0.0, 0.0);
      Point2D pointA1 = new Point2D(0.0, 1.0);
      Point2D pointA2 = new Point2D(1.0, 1.0);
      Point2D pointA3 = new Point2D(1.0, 0.0);

      Point2D pointB0 = new Point2D(1.2, 0.2);
      Point2D pointB1 = new Point2D(1.2, 0.8);
      Point2D pointB2 = new Point2D(1.8, 0.8);
      Point2D pointB3 = new Point2D(1.8, 0.2);

      ConvexPolygon2D polygonA = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(pointA0, pointA1, pointA2, pointA3));
      ConvexPolygon2D polygonB = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(pointB0, pointB1, pointB2, pointB3));

      PlanarRegion regionA = new PlanarRegion(new RigidBodyTransform(), polygonA);
      PlanarRegion regionB = new PlanarRegion(new RigidBodyTransform(), polygonB);

      ArrayList<PlanarRegion> mergedPlanarRegions = ConcaveHullMerger.mergePlanarRegions(regionA, regionB, 1.0);
      assertTrue(mergedPlanarRegions.isEmpty());

      mergedPlanarRegions = ConcaveHullMerger.mergePlanarRegions(regionB, regionA, 1.0);
      assertTrue(mergedPlanarRegions.isEmpty());
   }

   @Test
   public void testMergeShapesNotIntersectingThoughBoundingBoxesClearlyIntersect()
   {
      boolean visualize = false; //true;

      Point2D pointA0 = new Point2D(0.0, 0.0);
      Point2D pointA1 = new Point2D(0.0, 1.0);
      Point2D pointA2 = new Point2D(1.0, 1.0);

      Point2D pointB0 = new Point2D(0.4, 0.3);
      Point2D pointB1 = new Point2D(0.6, 0.5);
      Point2D pointB2 = new Point2D(0.6, 0.3);

      ConvexPolygon2D polygonA = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(pointA0, pointA1, pointA2));
      ConvexPolygon2D polygonB = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(pointB0, pointB1, pointB2));

      PlanarRegion regionA = new PlanarRegion(new RigidBodyTransform(), polygonA);
      PlanarRegion regionB = new PlanarRegion(new RigidBodyTransform(), polygonB);

      ConcaveHullMergerListener listener = (visualize ? new ConcaveHullGraphicalMergerListener() : new EnsureNoLoopsListener());

      ArrayList<PlanarRegion> mergedPlanarRegions = ConcaveHullMerger.mergePlanarRegions(regionA, regionB, 1.0, listener);
      if (visualize)
         ThreadTools.sleepForever();
      assertTrue(mergedPlanarRegions.isEmpty());

      mergedPlanarRegions = ConcaveHullMerger.mergePlanarRegions(regionB, regionA, 1.0);
      assertTrue(mergedPlanarRegions.isEmpty());
   }

   @Test
   public void testMergeShapesNotIntersectingButBoundingBoxesLookLikeFullyInside()
   {
      boolean visualize = false;

      Point2D pointA0 = new Point2D(0.0, 0.0);
      Point2D pointA1 = new Point2D(0.0, 1.0);
      Point2D pointA2 = new Point2D(1.0, 1.0);

      Point2D pointB0 = new Point2D(0.2, 0.0);
      Point2D pointB1 = new Point2D(1.2, 1.0);
      Point2D pointB2 = new Point2D(1.2, 0.0);

      ConvexPolygon2D polygonA = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(pointA0, pointA1, pointA2));
      ConvexPolygon2D polygonB = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(pointB0, pointB1, pointB2));

      PlanarRegion regionA = new PlanarRegion(new RigidBodyTransform(), polygonA);
      PlanarRegion regionB = new PlanarRegion(new RigidBodyTransform(), polygonB);

      ConcaveHullMergerListener listener = (visualize ? new ConcaveHullGraphicalMergerListener() : new EnsureNoLoopsListener());

      ArrayList<PlanarRegion> mergedPlanarRegions = ConcaveHullMerger.mergePlanarRegions(regionA, regionB, 1.0, listener);
      assertTrue(mergedPlanarRegions.isEmpty());

      mergedPlanarRegions = ConcaveHullMerger.mergePlanarRegions(regionB, regionA, 1.0);
      assertTrue(mergedPlanarRegions.isEmpty());

      if (visualize)
         ThreadTools.sleepForever();
   }

   @Test
   public void testMergePlanarRegionsWithDifferentTranslations()
   {
      boolean visualize = false;

      Point2D pointA0 = new Point2D(0.0, 0.0);
      Point2D pointA1 = new Point2D(0.0, 1.0);
      Point2D pointA2 = new Point2D(1.0, 1.0);
      Point2D pointA3 = new Point2D(1.0, 0.0);

      Point2D pointB0 = new Point2D(0.0, 0.0);
      Point2D pointB1 = new Point2D(0.0, 1.0);
      Point2D pointB2 = new Point2D(1.0, 1.0);
      Point2D pointB3 = new Point2D(1.0, 0.0);

      ConvexPolygon2D polygonA = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(pointA0, pointA1, pointA2, pointA3));
      ConvexPolygon2D polygonB = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(pointB0, pointB1, pointB2, pointB3));

      RigidBodyTransform transformA = new RigidBodyTransform();
      transformA.setTranslation(1.0, 2.0, 3.0);
      RigidBodyTransform transformB = new RigidBodyTransform();
      transformB.setTranslation(1.5, 2.5, 3.0);

      PlanarRegion regionA = new PlanarRegion(transformA, polygonA);
      PlanarRegion regionB = new PlanarRegion(transformB, polygonB);
      ArrayList<PlanarRegion> mergedPlanarRegions = ConcaveHullMerger.mergePlanarRegions(regionA, regionB, 1.0);
      PlanarRegion mergedPlanarRegion = mergedPlanarRegions.get(0);

      regionA.setRegionId(1);
      regionB.setRegionId(2);
      mergedPlanarRegion.setRegionId(3);

      BoundingBox3D mergedBoundingBox = mergedPlanarRegion.getBoundingBox3dInWorld();
      assertEquals(new BoundingBox3D(1.0, 2.0, 3.0, 2.5, 3.5, 3.0), mergedBoundingBox);

      Point2D[] concaveHull = mergedPlanarRegion.getConcaveHull();
      assertEquals(8, concaveHull.length);

      assertConcaveHullContains(concaveHull, 0.0, 0.0);
      assertConcaveHullContains(concaveHull, 0.0, 1.0);
      assertConcaveHullContains(concaveHull, 0.5, 1.0);
      assertConcaveHullContains(concaveHull, 0.5, 1.5);
      assertConcaveHullContains(concaveHull, 1.5, 1.5);
      assertConcaveHullContains(concaveHull, 1.5, 0.5);
      assertConcaveHullContains(concaveHull, 1.0, 0.5);
      assertConcaveHullContains(concaveHull, 1.0, 0.0);

      assertEquals(2, mergedPlanarRegion.getNumberOfConvexPolygons());

      if (visualize)
      {
         visualizePlanarRegions(regionA, regionB, mergedPlanarRegion);
         ThreadTools.sleepForever();
      }
   }

   @Test
   public void testMergePlanarRegionsWithDifferentTransforms()
   {
      boolean visualize = false;

      Point2D pointA0 = new Point2D(1.0, 2.0);
      Point2D pointA1 = new Point2D(1.0, 3.0);
      Point2D pointA2 = new Point2D(2.0, 3.0);
      Point2D pointA3 = new Point2D(2.0, 2.0);

      ConvexPolygon2D polygonA = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(pointA0, pointA1, pointA2, pointA3));

      RigidBodyTransform transformA = new RigidBodyTransform();
      transformA.setTranslation(0.1, -0.2, 0.3);
      transformA.setRotationYawPitchRoll(0.16, 0.37, 0.44);

      RigidBodyTransform transformAInverse = new RigidBodyTransform(transformA);
      transformAInverse.invert();

      PlanarRegion regionA = new PlanarRegion(transformA, polygonA);

      Point3D pointOnPlaneA = new Point3D(5.17, 6.3, 0.0);
      transformA.transform(pointOnPlaneA);

      RigidBodyTransform transformB = new RigidBodyTransform();
      transformB.setTranslation(pointOnPlaneA);
      transformB.setRotationYawPitchRoll(0.16, 0.37, 0.44);
      RigidBodyTransform transformBInverse = new RigidBodyTransform(transformB);
      transformBInverse.invert();

      Point3D pointB0 = new Point3D(1.5, 2.5, 0.0);
      Point3D pointB1 = new Point3D(1.5, 3.5, 0.0);
      Point3D pointB2 = new Point3D(2.5, 3.5, 0.0);
      Point3D pointB3 = new Point3D(2.5, 2.5, 0.0);

      transformA.transform(pointB0);
      transformBInverse.transform(pointB0);

      transformA.transform(pointB1);
      transformBInverse.transform(pointB1);

      transformA.transform(pointB2);
      transformBInverse.transform(pointB2);

      transformA.transform(pointB3);
      transformBInverse.transform(pointB3);

      assertEquals(0.0, pointB0.getZ(), 1e-7);
      assertEquals(0.0, pointB1.getZ(), 1e-7);
      assertEquals(0.0, pointB2.getZ(), 1e-7);
      assertEquals(0.0, pointB3.getZ(), 1e-7);

      Point2D point2DB0 = new Point2D(pointB0);
      Point2D point2DB1 = new Point2D(pointB1);
      Point2D point2DB2 = new Point2D(pointB2);
      Point2D point2DB3 = new Point2D(pointB3);

      ConvexPolygon2D polygonB = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(point2DB0, point2DB1, point2DB2, point2DB3));
      PlanarRegion regionB = new PlanarRegion(transformB, polygonB);

      ArrayList<PlanarRegion> mergedPlanarRegions = ConcaveHullMerger.mergePlanarRegions(regionA, regionB, 1.0);
      PlanarRegion mergedPlanarRegion = mergedPlanarRegions.get(0);

      regionA.setRegionId(1);
      regionB.setRegionId(2);
      mergedPlanarRegion.setRegionId(3);

      BoundingBox3D mergedBoundingBox = mergedPlanarRegion.getBoundingBox3dInWorld();

      BoundingBox3D expectedBox = generateBoundingBox(transformA,
                                                      pointA0,
                                                      pointA1,
                                                      pointA2,
                                                      pointA3,
                                                      new Point2D(1.5, 2.5),
                                                      new Point2D(1.5, 3.5),
                                                      new Point2D(2.5, 3.5),
                                                      new Point2D(2.5, 2.5));

      assertTrue(expectedBox.epsilonEquals(mergedBoundingBox, 1e-7));

      Point2D[] concaveHull = mergedPlanarRegion.getConcaveHull();
      assertEquals(8, concaveHull.length);

      assertConcaveHullContains(concaveHull, 1.0, 2.0);
      assertConcaveHullContains(concaveHull, 1.0, 3.0);
      assertConcaveHullContains(concaveHull, 1.5, 3.0);
      assertConcaveHullContains(concaveHull, 1.5, 3.5);
      assertConcaveHullContains(concaveHull, 2.5, 3.5);
      assertConcaveHullContains(concaveHull, 2.5, 2.5);
      assertConcaveHullContains(concaveHull, 2.0, 2.5);
      assertConcaveHullContains(concaveHull, 2.0, 2.0);

      assertEquals(2, mergedPlanarRegion.getNumberOfConvexPolygons());

      if (visualize)
      {
         visualizePlanarRegions(regionA, regionB, mergedPlanarRegion);
         ThreadTools.sleepForever();
      }
   }

   private BoundingBox3D generateBoundingBox(RigidBodyTransform transform, Point2D... pointsInBox)
   {
      Point3D minPoint = new Point3D(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
      Point3D maxPoint = new Point3D(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY);

      for (Point2D pointInBox : pointsInBox)
      {
         Point3D point = new Point3D(pointInBox);
         transform.transform(point);

         minPoint.set(Math.min(minPoint.getX(), point.getX()), Math.min(minPoint.getY(), point.getY()), Math.min(minPoint.getZ(), point.getZ()));
         maxPoint.set(Math.max(maxPoint.getX(), point.getX()), Math.max(maxPoint.getY(), point.getY()), Math.max(maxPoint.getZ(), point.getZ()));
      }

      return new BoundingBox3D(minPoint, maxPoint);
   }

   @Test
   public void testMergeConcaveHullsSimpleSquares()
   {
      ArrayList<Point2D> hullAVertices = new ArrayList<Point2D>();

      Point2D pointA0 = new Point2D(0.0, 0.0);
      Point2D pointA1 = new Point2D(0.0, 1.0);
      Point2D pointA2 = new Point2D(1.0, 1.0);
      Point2D pointA3 = new Point2D(1.0, 0.0);

      hullAVertices.add(pointA0);
      hullAVertices.add(pointA1);
      hullAVertices.add(pointA2);
      hullAVertices.add(pointA3);

      ArrayList<Point2D> hullBVertices = new ArrayList<Point2D>();

      Point2D pointB0 = new Point2D(-0.5, -0.5);
      Point2D pointB1 = new Point2D(-0.5, 0.5);
      Point2D pointB2 = new Point2D(0.5, 0.5);
      Point2D pointB3 = new Point2D(0.5, -0.49);

      hullBVertices.add(pointB0);
      hullBVertices.add(pointB1);
      hullBVertices.add(pointB2);
      hullBVertices.add(pointB3);

      ArrayList<Point2D> mergedHulls = ConcaveHullMerger.mergeConcaveHulls(hullAVertices, hullBVertices, null);

      assertEquals(8, mergedHulls.size());

      int i = 0;
      double epsilon = 1e-7;
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointB0, mergedHulls.get(i++), epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointB1, mergedHulls.get(i++), epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(0.0, 0.5), mergedHulls.get(i++), epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointA1, mergedHulls.get(i++), epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointA2, mergedHulls.get(i++), epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointA3, mergedHulls.get(i++), epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(0.5, 0.0), mergedHulls.get(i++), epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointB3, mergedHulls.get(i++), epsilon);
   }

   @Test
   public void testMergeConcaveHullsSmallSquareInsideLargeSquare()
   {
      boolean visualize = false;

      ArrayList<Point2D> hullAVertices = new ArrayList<Point2D>();

      Point2D pointA0 = new Point2D(0.0, 0.0);
      Point2D pointA1 = new Point2D(0.0, 1.0);
      Point2D pointA2 = new Point2D(1.0, 1.0);
      Point2D pointA3 = new Point2D(1.0, 0.0);

      hullAVertices.add(pointA0);
      hullAVertices.add(pointA1);
      hullAVertices.add(pointA2);
      hullAVertices.add(pointA3);

      ArrayList<Point2D> hullBVertices = new ArrayList<Point2D>();

      Point2D pointB0 = new Point2D(0.5, 0.5);
      Point2D pointB1 = new Point2D(0.5, 0.6);
      Point2D pointB2 = new Point2D(0.6, 0.6);
      Point2D pointB3 = new Point2D(0.6, 0.5);

      hullBVertices.add(pointB0);
      hullBVertices.add(pointB1);
      hullBVertices.add(pointB2);
      hullBVertices.add(pointB3);

      ConcaveHullMergerListener listener = (visualize ? new ConcaveHullGraphicalMergerListener() : new EnsureNoLoopsListener());
      ArrayList<Point2D> mergedHulls = ConcaveHullMerger.mergeConcaveHulls(hullAVertices, hullBVertices, listener);

      assertEquals(4, mergedHulls.size());

      int i = 0;
      double epsilon = 1e-7;
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointA0, mergedHulls.get(i++), epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointA1, mergedHulls.get(i++), epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointA2, mergedHulls.get(i++), epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointA3, mergedHulls.get(i++), epsilon);

      if (visualize)
         ThreadTools.sleepForever();
   }

   @Test
   public void testMergeConcaveHullsInteriorHole()
   {
      boolean visualize = false;

      ArrayList<Point2D> hullAVertices = new ArrayList<Point2D>();

      Point2D pointA0 = new Point2D(0.0, 0.0);
      Point2D pointA1 = new Point2D(0.0, 1.0);
      Point2D pointA2 = new Point2D(1.0, 1.0);
      Point2D pointA3 = new Point2D(1.0, 0.0);

      hullAVertices.add(pointA0);
      hullAVertices.add(pointA1);
      hullAVertices.add(pointA2);
      hullAVertices.add(pointA3);

      ArrayList<Point2D> hullBVertices = new ArrayList<Point2D>();

      Point2D pointB0 = new Point2D(0.1, -0.5);
      Point2D pointB1 = new Point2D(0.1, 0.5);
      Point2D pointB2 = new Point2D(0.2, 0.5);
      Point2D pointB3 = new Point2D(0.2, -0.4);
      Point2D pointB4 = new Point2D(0.3, -0.4);
      Point2D pointB5 = new Point2D(0.3, 0.5);
      Point2D pointB6 = new Point2D(0.4, 0.5);
      Point2D pointB7 = new Point2D(0.4, -0.5);

      hullBVertices.add(pointB0);
      hullBVertices.add(pointB1);
      hullBVertices.add(pointB2);
      hullBVertices.add(pointB3);
      hullBVertices.add(pointB4);
      hullBVertices.add(pointB5);
      hullBVertices.add(pointB6);
      hullBVertices.add(pointB7);

      ConcaveHullMergerListener listener = (visualize ? new ConcaveHullGraphicalMergerListener() : new EnsureNoLoopsListener());
      ArrayList<Point2D> mergedHulls = ConcaveHullMerger.mergeConcaveHulls(hullAVertices, hullBVertices, listener);

      assertEquals(8, mergedHulls.size());

      int i = 0;
      double epsilon = 1e-7;
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointA0, mergedHulls.get(i++), epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointA1, mergedHulls.get(i++), epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointA2, mergedHulls.get(i++), epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointA3, mergedHulls.get(i++), epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(0.4, 0.0), mergedHulls.get(i++), epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointB7, mergedHulls.get(i++), epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointB0, mergedHulls.get(i++), epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(0.1, 0.0), mergedHulls.get(i++), epsilon);

      if (visualize)
         ThreadTools.sleepForever();
   }

   @Test
   public void testMergeExactSame()
   {
      boolean visualize = false;

      ArrayList<Point2D> hullAVertices = new ArrayList<Point2D>();

      Point2D pointA0 = new Point2D(0.0, 0.0);
      Point2D pointA1 = new Point2D(0.0, 1.0);
      Point2D pointA2 = new Point2D(1.0, 1.0);
      Point2D pointA3 = new Point2D(1.0, 0.0);

      hullAVertices.add(pointA0);
      hullAVertices.add(pointA1);
      hullAVertices.add(pointA2);
      hullAVertices.add(pointA3);

      ArrayList<Point2D> hullBVertices = new ArrayList<Point2D>();

      Point2D pointB0 = new Point2D(0.0, 0.0);
      Point2D pointB1 = new Point2D(0.0, 1.0);
      Point2D pointB2 = new Point2D(1.0, 1.0);
      Point2D pointB3 = new Point2D(1.0, 0.0);

      hullBVertices.add(pointB0);
      hullBVertices.add(pointB1);
      hullBVertices.add(pointB2);
      hullBVertices.add(pointB3);

      ConcaveHullMergerListener listener = (visualize ? new ConcaveHullGraphicalMergerListener() : new EnsureNoLoopsListener());
      ArrayList<Point2D> mergedHulls = ConcaveHullMerger.mergeConcaveHulls(hullAVertices, hullBVertices, listener);

      assertEquals(4, mergedHulls.size());

      int i = 0;
      double epsilon = 1e-7;
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointA0, mergedHulls.get(i++), epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointA1, mergedHulls.get(i++), epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointA2, mergedHulls.get(i++), epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointA3, mergedHulls.get(i++), epsilon);

      if (visualize)
         ThreadTools.sleepForever();
   }

   @Test
   public void testMergeRemoveColinearPoints()
   {
      boolean visualize = false;

      ArrayList<Point2D> hullAVertices = new ArrayList<Point2D>();

      Point2D pointA0 = new Point2D(0.0, 0.0);
      Point2D pointC0 = new Point2D(0.0, 0.5);
      Point2D pointA1 = new Point2D(0.0, 1.0);
      Point2D pointC1 = new Point2D(0.5, 1.0);
      Point2D pointA2 = new Point2D(1.0, 1.0);
      Point2D pointC2 = new Point2D(1.0, 0.5);
      Point2D pointA3 = new Point2D(1.0, 0.0);
      Point2D pointC3 = new Point2D(0.5, 0.0);

      hullAVertices.add(pointA0);
      hullAVertices.add(pointC0);
      hullAVertices.add(pointA1);
      hullAVertices.add(pointC1);
      hullAVertices.add(pointA2);
      hullAVertices.add(pointC2);
      hullAVertices.add(pointA3);
      hullAVertices.add(pointC3);

      ArrayList<Point2D> hullBVertices = new ArrayList<Point2D>();

      Point2D pointB0 = new Point2D(0.5, 0.5);
      Point2D pointB1 = new Point2D(0.5, 0.6);
      Point2D pointB2 = new Point2D(0.6, 0.6);
      Point2D pointB3 = new Point2D(0.6, 0.5);

      hullBVertices.add(pointB0);
      hullBVertices.add(pointB1);
      hullBVertices.add(pointB2);
      hullBVertices.add(pointB3);

      ConcaveHullMergerListener listener = (visualize ? new ConcaveHullGraphicalMergerListener() : new EnsureNoLoopsListener());
      ArrayList<Point2D> mergedHulls = ConcaveHullMerger.mergeConcaveHulls(hullAVertices, hullBVertices, listener);

      assertEquals(4, mergedHulls.size());

      int i = 0;
      double epsilon = 1e-7;
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointA0, mergedHulls.get(i++), epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointA1, mergedHulls.get(i++), epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointA2, mergedHulls.get(i++), epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointA3, mergedHulls.get(i++), epsilon);

      if (visualize)
         ThreadTools.sleepForever();
   }

   @Test
   public void testMergeRemoveSlivers()
   {
      boolean visualize = false;

      ArrayList<Point2D> hullAVertices = new ArrayList<Point2D>();

      Point2D pointA0 = new Point2D(0.0, 0.0);
      Point2D pointA1 = new Point2D(0.0, 1.0);
      Point2D pointS1 = new Point2D(0.1, 10.1);
      Point2D pointS2 = new Point2D(0.01, 1.0);
      Point2D pointS3 = new Point2D(0.9999, 1.0);
      Point2D pointS4 = new Point2D(0.99995, 1.01);
      Point2D pointA2 = new Point2D(1.0, 1.0);
      Point2D pointA3 = new Point2D(1.0, 0.0);

      hullAVertices.add(pointA0);
      hullAVertices.add(pointA1);
      hullAVertices.add(pointS1);
      hullAVertices.add(pointS2);
      hullAVertices.add(pointS3);
      hullAVertices.add(pointS4);
      hullAVertices.add(pointA2);
      hullAVertices.add(pointA3);

      ArrayList<Point2D> hullBVertices = new ArrayList<Point2D>();

      Point2D pointB0 = new Point2D(0.5, 0.5);
      Point2D pointB1 = new Point2D(0.5, 0.6);
      Point2D pointB2 = new Point2D(0.6, 0.6);
      Point2D pointB3 = new Point2D(0.6, 0.5);

      hullBVertices.add(pointB0);
      hullBVertices.add(pointB1);
      hullBVertices.add(pointB2);
      hullBVertices.add(pointB3);

      ConcaveHullMergerListener listener = (visualize ? new ConcaveHullGraphicalMergerListener() : new EnsureNoLoopsListener());
      ArrayList<Point2D> mergedHulls = ConcaveHullMerger.mergeConcaveHulls(hullAVertices, hullBVertices, listener);

      assertEquals(4, mergedHulls.size());

      int i = 0;
      double epsilon = 1e-7;
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointA0, mergedHulls.get(i++), epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointA1, mergedHulls.get(i++), epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointS3, mergedHulls.get(i++), epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointA3, mergedHulls.get(i++), epsilon);

      if (visualize)
         ThreadTools.sleepForever();
   }

   @Test
   public void testMergeConcaveHullsTroublesomeOne()
   {
      boolean visualize = false;

      double[][] hullOne = new double[][] {{-0.32080844044685364, 0.9086798429489136}, {-0.28522926568984985, 0.9880890846252441},
            {-0.1836146116256714, 1.0075507164001465}, {-0.1378334015607834, 0.9325186014175415}, {-0.12150357663631439, 0.8489445447921753},
            {-0.11247798800468445, 0.7848914265632629}, {-0.10055802017450333, 0.7219595313072205}, {-0.0858231782913208, 0.6531859636306763},
            {-0.08974666893482208, 0.5877435803413391}, {-0.07057010382413864, 0.44702088832855225}, {-0.05137510970234871, 0.35009393095970154},
            {-0.048323068767786026, 0.2116333544254303}, {-0.029583362862467766, 0.06909555941820145}, {-0.012792153283953667, -0.005410400684922934},
            {-0.002692134352400899, -0.07482614368200302}, {0.015383324585855007, -0.17355236411094666}, {0.02966991625726223, -0.3129369914531708},
            {0.030642254278063774, -0.38066574931144714}, {0.09247396886348724, -0.41485705971717834}, {0.19452345371246338, -0.4152769446372986},
            {0.23610325157642365, -0.414505273103714}, {0.3529823124408722, -0.41112226247787476}, {0.41674885153770447, -0.41055598855018616},
            {0.5968952775001526, -0.41316646337509155}, {0.6718282699584961, -0.4079112112522125}, {0.7358959913253784, -0.37659206986427307},
            {0.7940598726272583, -0.39256760478019714}, {0.8539779782295227, -0.36987099051475525}, {0.8775563836097717, -0.44087880849838257},
            {0.8951427340507507, -0.497694730758667}, {0.8463701009750366, -0.5413227081298828}, {0.7558804750442505, -0.5653964281082153},
            {0.6759657263755798, -0.551409125328064}, {0.6368259787559509, -0.549574077129364}, {0.5739794373512268, -0.550038754940033},
            {0.5343534350395203, -0.588975727558136}, {0.45960354804992676, -0.5972794890403748}, {0.3683496415615082, -0.587111234664917},
            {0.2744791507720947, -0.6093615889549255}, {0.14267343282699585, -0.6075680255889893}, {0.0530564971268177, -0.6351839900016785},
            {-0.0165009256452322, -0.6289142370223999}, {-0.08789733052253723, -0.6491257548332214}, {-0.14297418296337128, -0.5095702409744263},
            {-0.1441502869129181, -0.4497186839580536}, {-0.1623517870903015, -0.36984652280807495}, {-0.16642220318317413, -0.31222599744796753},
            {-0.17461782693862915, -0.2546726167201996}, {-0.19403919577598572, -0.13188748061656952}, {-0.20356975495815277, -0.03135628625750542},
            {-0.22276757657527924, 0.028552822768688202}, {-0.23816534876823425, 0.12956836819648743}, {-0.2233646810054779, 0.20830659568309784},
            {-0.23749388754367828, 0.26624444127082825}, {-0.2414117306470871, 0.36849892139434814}, {-0.24244770407676697, 0.3880820572376251},
            {-0.2581060230731964, 0.46913930773735046}, {-0.2633146047592163, 0.5846882462501526}, {-0.2798686921596527, 0.6706967353820801},
            {-0.3030450940132141, 0.8290156126022339}};

      double[][] hullTwo = new double[][] {{-0.15936496272840953, 0.08901430953161665}, {-0.047907207824508136, 0.11249011839155193},
            {-0.009886369055515487, 0.0780884316128021}, {-0.004574789669821888, -0.03442622045027294}, {0.022450792800197822, -0.1683871873291485},
            {0.012758309221879843, -0.24440572474261255}, {0.01789889409201989, -0.3444630600706097}, {0.055439442362793676, -0.41044137718121043},
            {0.12838851032560822, -0.4200984524054827}, {0.18588113170475543, -0.40900853006219284}, {0.2616279871221178, -0.40034939313016116},
            {0.3728812252385902, -0.40692794894670714}, {0.5067275085573423, -0.3890745762402008}, {0.5853448326672742, -0.3998915611253164},
            {0.8214038080362078, -0.4249131537925772}, {0.8878281581325169, -0.4934313704624567}, {0.8562573850357904, -0.5638341162746424},
            {0.6972782956386417, -0.556293100791231}, {0.6306328030329531, -0.5724172189201938}, {0.5776880238324902, -0.567426514063733},
            {0.4619108600260218, -0.5413079293009101}, {0.38840872535166443, -0.5895591884394169}, {0.29777405386267325, -0.584678478243929},
            {0.21465674088276535, -0.5958315076800703}, {0.13354830510427668, -0.6164547163407885}, {0.023641897671828227, -0.6240701572006659},
            {-0.02899821040360062, -0.6240849919270718}, {-0.12330590277407918, -0.5602205030858636}, {-0.11745372007422264, -0.4666361629461157},
            {-0.14178226448362094, -0.32370311833003684}, {-0.140508233857459, -0.2346025872800086}, {-0.17330307409554194, -0.149159839926986},
            {-0.19239786762148312, -0.09986922600319931}, {-0.19854388436229184, -8.514394122500213E-4}, {-0.22962867228965547, 0.05809039869701854}};

      ConcaveHullMergerListener listener = (visualize ? new ConcaveHullGraphicalMergerListener() : new EnsureNoLoopsListener());
      ArrayList<Point2D> mergedHulls = ConcaveHullMerger.mergeConcaveHulls(convertToPointArrayList(hullOne), convertToPointArrayList(hullTwo), listener);

      assertEquals(72, mergedHulls.size());

      if (visualize)
         ThreadTools.sleepForever();

   }

   @Test
   public void testMergeConcaveHullsTroublesomeTwo()
   {
      boolean visualize = false;

      double[][] hullOne = new double[][] {{-2.082104444503784, 0.05125557631254196}, {-1.9877903461456299, 0.05051760375499725},
            {-1.8934645652770996, 0.032866742461919785}, {-1.8063348531723022, 0.017969924956560135}, {-1.8078804016113281, -0.05258311331272125},
            {-1.7552627325057983, -0.1071154922246933}, {-1.7423378229141235, -0.12050453573465347}, {-1.676104187965393, -0.18628227710723877},
            {-1.6244268417358398, -0.2372368574142456}, {-1.5305248498916626, -0.25680819153785706}, {-1.4671834707260132, -0.2996377944946289},
            {-1.3831865787506104, -0.31048935651779175}, {-1.3015289306640625, -0.31212350726127625}, {-1.2350205183029175, -0.31031280755996704},
            {-1.1435714960098267, -0.28981801867485046}, {-1.1160051822662354, -0.2645573318004608}, {-1.0887410640716553, -0.1946900188922882},
            {-1.061100721359253, -0.14609213173389435}, {-1.0084608793258667, -0.08502703905105591}, {-0.9378349184989929, -0.01490721944719553},
            {-0.8357486128807068, 0.05732116848230362}, {-0.7898916006088257, 0.10894356667995453}, {-0.7022786736488342, 0.21263469755649567},
            {-0.6430915594100952, 0.2732110619544983}, {-0.5430570840835571, 0.3704082667827606}, {-0.4844106137752533, 0.44353628158569336},
            {-0.4357295036315918, 0.47629278898239136}, {-0.36894553899765015, 0.5445412397384644}, {-0.32173189520835876, 0.5781226754188538},
            {-0.26838046312332153, 0.620795726776123}, {-0.242606058716774, 0.6473331451416016}, {-0.20874285697937012, 0.7084746360778809},
            {-0.09277884662151337, 0.8272638916969299}, {-0.028089450672268867, 0.8890215158462524}, {0.016691898927092552, 0.9345527291297913},
            {0.05730509012937546, 0.9827704429626465}, {0.11536040157079697, 1.04026460647583}, {0.21615491807460785, 1.1102014780044556},
            {0.2735670208930969, 1.1534547805786133}, {0.3370110094547272, 1.2286189794540405}, {0.3777802288532257, 1.2922598123550415},
            {0.4199817478656769, 1.347459077835083}, {0.48012274503707886, 1.3970167636871338}, {0.5779113173484802, 1.4863139390945435},
            {1.5910969972610474, 0.47237688302993774}, {1.5079950094223022, 0.4275287091732025}, {1.4733177423477173, 0.4056212604045868},
            {1.4113112688064575, 0.333622545003891}, {1.3381381034851074, 0.29473432898521423}, {1.2606370449066162, 0.2316516637802124},
            {1.1555317640304565, 0.11730114370584488}, {1.0875968933105469, 0.06857618689537048}, {1.0097259283065796, -0.001877879141829908},
            {0.9603399038314819, -0.06472857296466827}, {0.9107681512832642, -0.1099410429596901}, {0.7950878143310547, -0.22662493586540222},
            {0.7255138158798218, -0.30090269446372986}, {0.688881516456604, -0.3477385342121124}, {0.6433224081993103, -0.3861510157585144},
            {0.6115292310714722, -0.42774802446365356}, {0.5735395550727844, -0.4638318419456482}, {0.47669288516044617, -0.5827385783195496},
            {0.44507336616516113, -0.6233129501342773}, {0.387230783700943, -0.6881033182144165}, {0.27006223797798157, -0.8419532775878906},
            {0.17447824776172638, -0.9532765746116638}, {0.14561527967453003, -1.0041385889053345}, {0.059466179460287094, -1.0075795650482178},
            {0.03963623195886612, -1.0880444049835205}, {-0.047746602445840836, -1.1276521682739258}, {-0.10712536424398422, -1.1768836975097656},
            {-0.12849126756191254, -1.2254655361175537}, {-0.17003276944160461, -1.330750823020935}, {-0.29389914870262146, -1.4000132083892822},
            {-0.3854202926158905, -1.387130618095398}, {-0.4780607521533966, -1.3009114265441895}, {-0.5682555437088013, -1.2937103509902954},
            {-0.6898995041847229, -1.2490521669387817}, {-0.7441661357879639, -1.194881558418274}, {-0.8286762833595276, -1.2018039226531982},
            {-0.9381610155105591, -1.1064698696136475}, {-0.9777572154998779, -1.0657685995101929}, {-1.0662246942520142, -1.0685185194015503},
            {-1.1408783197402954, -1.046222448348999}, {-1.198838710784912, -1.0126358270645142}, {-1.5083709955215454, -0.6532383561134338},
            {-1.5101680755615234, -0.5549001693725586}, {-1.5158392190933228, -0.4653123617172241}, {-1.53538179397583, -0.40096455812454224},
            {-1.6771008968353271, -0.3824208974838257}, {-1.759894609451294, -0.39008790254592896}, {-1.848076343536377, -0.33226528763771057},
            {-2.026582956314087, -0.15288995206356049}, {-2.1302809715270996, -0.04885220527648926}, {-2.224780797958374, 0.045223675668239594}};

      double[][] hullTwo = new double[][] {{-3.799445695334325, -0.42973217297844957}, {-3.7145963543689087, -0.4341958088088299},
            {-3.6625632690725736, -0.43974306161183463}, {-3.612129416163534, -0.44620689089580445}, {-3.473328397545693, -0.4617442861119512},
            {-3.3577920734258995, -0.43374396043043284}, {-3.28880990154688, -0.4249860241780795}, {-3.2068304426176986, -0.4326095159477257},
            {-3.178334839111675, -0.43501583713888237}, {-3.0482773221634845, -0.4306356073549211}, {-2.9618382728924737, -0.43090573594444936},
            {-2.8067482405018787, -0.438561807047094}, {-2.7196650100896056, -0.4220749337520513}, {-2.6607574134763263, -0.4268018527509013},
            {-2.6203961340312594, -0.4292186225775919}, {-2.428864343695836, -0.4374181742749026}, {-2.3684734264172165, -0.4507742683428645},
            {-2.2753118081921726, -0.4242756977199562}, {-2.1611980040393886, -0.4304097265983503}, {-2.079283238394495, -0.4312032653657578},
            {-1.997670956702263, -0.4282221512917811}, {-1.9198554977599176, -0.42800402620727873}, {-1.8488251150964112, -0.43164445706534604},
            {-1.7101627730119753, -0.425387734915453}, {-1.6445435163770472, -0.4306366563347439}, {-1.5189611336571311, -0.44065878187889007},
            {-1.5212652541453555, -0.529666466875236}, {-1.5223072022189625, -0.6126764510083067}, {-1.538651363284346, -0.6966447955779362},
            {-1.5394608555969183, -0.8312145905285679}, {-1.5160786092972716, -0.9164987269425008}, {-1.5228657102207435, -0.9881333554978543},
            {-1.5411247924793696, -1.0769742667174464}, {-1.5354429326834012, -1.1748308422985558}, {-1.5437927694926041, -1.252839469064518},
            {-1.5035082482008497, -1.3344956701139767}, {-1.2200895645941459, -1.3755699519437197}, {-1.1219328727738724, -1.389414328794912},
            {-1.0608145984426323, -1.468048732757835}, {-1.002543911652151, -1.468774517889839}, {-0.9234624074021378, -1.4848628770275838},
            {-0.7629671076236277, -1.4916363490744997}, {-0.7020867978703911, -1.5048433554975853}, {-0.6410671643399415, -1.504945932540387},
            {-0.5258838455897088, -1.5128143929819395}, {-0.443114782206266, -1.5274362533377315}, {-0.38574811639449247, -1.5257173605237904},
            {-0.3695025624602699, -1.6067646440276127}, {-0.32753148591454706, -1.6908660268833273}, {-0.3021749223604422, -1.7685983101537524},
            {-0.3054542277611656, -1.8270949020588392}, {-0.26841000635152223, -1.9086446698359065}, {-0.26808860681938507, -1.9723862241733818},
            {-0.28574721261572833, -2.067114357481371}, {-0.28950101212123447, -2.14263761625921}, {-0.6646741846492519, -2.12965834454841},
            {-0.9782585599627811, -2.1012791349883804}, {-1.2246651254445724, -2.0557575218167963}, {-1.3063710560790684, -2.041025098306046},
            {-1.3905811663112015, -2.0259502683989647}, {-1.4410981710511432, -2.015861889177745}, {-2.40703372203003, -1.8384337998984193},
            {-2.508025762793483, -1.8199352065902916}, {-2.9553441897849764, -1.738192310667629}, {-3.0184198540422833, -1.7274523178141417},
            {-3.208651414834753, -1.691268768903317}, {-3.675159670225745, -1.6043730453127263}, {-3.6934249343340024, -1.449128139289222},
            {-3.7396534353248487, -1.3736924469481533}, {-3.742669253229052, -1.3058103217463977}, {-3.7670030049563303, -1.2098165616609893},
            {-3.7691582532768964, -1.1346925671718382}, {-3.784283933259286, -1.0286430993986295}, {-3.7862868510305794, -0.9345062019105703},
            {-3.8132915079838856, -0.8608667191971495}, {-3.8043304894665138, -0.8101139758334739}, {-3.7971833266524024, -0.7475298426445478},
            {-3.8155070000929037, -0.6502645615646477}, {-3.8099417680550687, -0.5730421391584346}, {-3.8250805933151035, -0.47766611704632744}};

      ConcaveHullMergerListener listener = (visualize ? new ConcaveHullGraphicalMergerListener() : new EnsureNoLoopsListener());
      ArrayList<Point2D> mergedHulls = ConcaveHullMerger.mergeConcaveHulls(convertToPointArrayList(hullOne), convertToPointArrayList(hullTwo), listener);

      assertEquals(164, mergedHulls.size());

      if (visualize)
         ThreadTools.sleepForever();
   }

   @Test
   public void testMergeConcaveHullsTroublesomeWhenNearlyIdentical()
   {
      boolean visualize = false;

      double[][] hullOne = new double[][] {{-0.22528910636901855, 0.28605905175209045}, {-0.1752581000328064, 0.2756955623626709},
            {-0.10332929342985153, 0.24632740020751953}, {-0.05382351204752922, 0.28411903977394104}, {-0.010817715898156166, 0.3718038499355316},
            {8.111802744679153E-4, 0.29732465744018555}, {0.060906536877155304, 0.2479160577058792}, {0.03242022916674614, 0.17993474006652832},
            {0.050539299845695496, 0.10181768238544464}, {0.17091168463230133, 0.020121606066823006}, {0.17031608521938324, 0.020255591720342636},
            {0.1703745722770691, 0.020217331126332283}, {0.16976216435432434, 0.02038019895553589}, {0.11882562935352325, 0.031838782131671906},
            {0.07250593602657318, -0.02279113605618477}, {0.10450576990842819, -0.10478051751852036}, {0.14919164776802063, -0.15212774276733398},
            {0.11351815611124039, -0.19198139011859894}, {0.12179604172706604, -0.26129671931266785}, {0.1837044060230255, -0.29922211170196533},
            {0.08616393804550171, -0.35188886523246765}, {0.01468310970813036, -0.33285972476005554}, {0.04612547159194946, -0.27007755637168884},
            {0.028615305200219154, -0.23202171921730042}, {0.011710579507052898, -0.19585317373275757}, {-0.0401255339384079, -0.1556309312582016},
            {-0.09073073416948318, -0.20682352781295776}, {-0.14255684614181519, -0.1741705983877182}, {-0.17794781923294067, 0.024299168959259987},
            {-0.19740694761276245, 0.08807963132858276}, {-0.197884663939476, 0.2125331163406372}};

      double[][] hullTwo = new double[][] {{-0.22529910611381146, 0.2860550296305229}, {-0.1752687403766458, 0.2756884483461346},
            {-0.10334174892340853, 0.24631584104908202}, {-0.05383363214300615, 0.2841044210867823}, {-0.010822417171192271, 0.3717865733225872},
            {8.018761427599047E-4, 0.29730666230504005}, {0.06089417912462361, 0.2478943487742924}, {0.032403670232445704, 0.17991479171880903},
            {0.05051791321467289, 0.10179661442990506}, {0.17088524884767012, 0.020093099258163993}, {0.11879991783150554, 0.03181349421734921},
            {0.07247684848929238, -0.022813561309552602}, {0.10447161532706783, -0.10480492020306813}, {0.14915456700027793, -0.1521549069468984},
            {0.11347861247604732, -0.19200634960062485}, {0.12175221436983483, -0.2613221902344628}, {0.183658234704434, -0.2992514084861219},
            {0.08611451217227299, -0.35191213391688886}, {0.014634860030003893, -0.3328785759677305}, {0.046081101779333895, -0.27009835083730116},
            {0.028573287291988405, -0.2320414316282917}, {0.011670796866622882, -0.19587184150171666}, {-0.04016283069860461, -0.1556463956340615},
            {-0.09077119450640433, -0.20683586469025897}, {-0.1425952883834524, -0.17417973247544907}, {-0.17797399591242255, 0.02429222165093927},
            {-0.19742918259614284, 0.08807388647119463}, {-0.19789920767358, 0.2125274007666813}};

      ConcaveHullMergerListener listener = (visualize ? new ConcaveHullGraphicalMergerListener() : new EnsureNoLoopsListener());
      ArrayList<Point2D> mergedHulls = ConcaveHullMerger.mergeConcaveHulls(convertToPointArrayList(hullOne), convertToPointArrayList(hullTwo), listener);

      assertEquals(27, mergedHulls.size());

      if (visualize)
         ThreadTools.sleepForever();
   }

   // TODO: Get this test working. It crosses its own path.
   //@Test
   public void testMergeConcaveHullsTroublesomeSkinnyOne()
   {
      boolean visualize = false;

      double[][] hullOne = new double[][] {{0.03324427828192711, -0.06848959624767303}, {0.034265462309122086, -0.0691266655921936},
            {0.03324427828192711, -0.06848959624767303}, {0.03423566743731499, -0.0690872073173523}, {0.03472326323390007, -0.06941226869821548},
            {0.13587459921836853, -0.13251610100269318}, {0.13272781670093536, -0.19862303137779236}, {0.19089597463607788, -0.21636398136615753},
            {0.26549431681632996, -0.2942051291465759}, {0.2768421769142151, -0.3913511037826538}, {0.1343332976102829, -0.20247167348861694},
            {0.13426625728607178, -0.2023698091506958}, {0.18867763876914978, -0.22048182785511017}, {0.26151809096336365, -0.2962256968021393},
            {0.2766615152359009, -0.3901367485523224}, {0.13011109828948975, -0.1960570365190506}, {0.13001465797424316, -0.1959104984998703},
            {0.13001017272472382, -0.19590459764003754}, {0.12981820106506348, -0.20088918507099152}, {0.13426625728607178, -0.2023698091506958},
            {0.13011109828948975, -0.1960570365190506}, {0.12753614783287048, -0.19264696538448334}};

      double[][] hullTwo = new double[][] {{0.13592818607655313, -0.13242901683856703}, {0.13277958016400696, -0.19853586025089806},
            {0.19094724867835142, -0.21627841478840013}, {0.26554344369702665, -0.2941216202077965}, {0.27688862424314487, -0.39126790764752944},
            {0.03329963129665654, -0.06839968113853566}};

      ConcaveHullMergerListener listener = (visualize ? new ConcaveHullGraphicalMergerListener() : new EnsureNoLoopsListener());
      ArrayList<Point2D> mergedHulls = ConcaveHullMerger.mergeConcaveHulls(convertToPointArrayList(hullOne), convertToPointArrayList(hullTwo), listener);

      if (visualize)
         ThreadTools.sleepForever();
      assertEquals(48, mergedHulls.size());

   }

   @Test
   public void testDetectSelfIntersectingConcaveHulls()
   {
      // Non-intersecting square.
      Point2D pointA0 = new Point2D(0.0, 0.0);
      Point2D pointA1 = new Point2D(0.0, 1.0);
      Point2D pointA2 = new Point2D(1.0, 1.0);
      Point2D pointA3 = new Point2D(1.0, 0.0);

      ArrayList<Point2D> nonIntersectingConcaveHull = new ArrayList<Point2D>();
      nonIntersectingConcaveHull.add(pointA0);
      nonIntersectingConcaveHull.add(pointA1);
      nonIntersectingConcaveHull.add(pointA2);
      nonIntersectingConcaveHull.add(pointA3);
      assertFalse(ConcaveHullMerger.isConcaveHullSelfIntersecting(nonIntersectingConcaveHull));

      // Intersecting figure eight.
      ArrayList<Point2D> intersectingConcaveHull = new ArrayList<Point2D>();
      intersectingConcaveHull.add(pointA0);
      intersectingConcaveHull.add(pointA1);
      intersectingConcaveHull.add(pointA3);
      intersectingConcaveHull.add(pointA2);
      assertTrue(ConcaveHullMerger.isConcaveHullSelfIntersecting(intersectingConcaveHull));

      // Make sure all orders return true:
      intersectingConcaveHull.clear();
      intersectingConcaveHull.add(pointA2);
      intersectingConcaveHull.add(pointA0);
      intersectingConcaveHull.add(pointA1);
      intersectingConcaveHull.add(pointA3);
      assertTrue(ConcaveHullMerger.isConcaveHullSelfIntersecting(intersectingConcaveHull));

      intersectingConcaveHull.clear();
      intersectingConcaveHull.add(pointA3);
      intersectingConcaveHull.add(pointA2);
      intersectingConcaveHull.add(pointA0);
      intersectingConcaveHull.add(pointA1);
      assertTrue(ConcaveHullMerger.isConcaveHullSelfIntersecting(intersectingConcaveHull));

      intersectingConcaveHull.clear();
      intersectingConcaveHull.add(pointA1);
      intersectingConcaveHull.add(pointA3);
      intersectingConcaveHull.add(pointA2);
      intersectingConcaveHull.add(pointA0);
      assertTrue(ConcaveHullMerger.isConcaveHullSelfIntersecting(intersectingConcaveHull));

      // Null, Zero, One, Two, or Three Vertices are not intersecting.
      assertFalse(ConcaveHullMerger.isConcaveHullSelfIntersecting(null));

      intersectingConcaveHull.clear();
      assertFalse(ConcaveHullMerger.isConcaveHullSelfIntersecting(intersectingConcaveHull));

      intersectingConcaveHull.clear();
      intersectingConcaveHull.add(pointA1);
      assertFalse(ConcaveHullMerger.isConcaveHullSelfIntersecting(intersectingConcaveHull));

      intersectingConcaveHull.clear();
      intersectingConcaveHull.add(pointA1);
      intersectingConcaveHull.add(pointA3);
      assertFalse(ConcaveHullMerger.isConcaveHullSelfIntersecting(intersectingConcaveHull));

      intersectingConcaveHull.clear();
      intersectingConcaveHull.add(pointA1);
      intersectingConcaveHull.add(pointA3);
      intersectingConcaveHull.add(pointA2);
      assertFalse(ConcaveHullMerger.isConcaveHullSelfIntersecting(intersectingConcaveHull));

      // Backwards does not change things for non-intersecting.
      nonIntersectingConcaveHull.clear();
      nonIntersectingConcaveHull.add(pointA3);
      nonIntersectingConcaveHull.add(pointA2);
      nonIntersectingConcaveHull.add(pointA1);
      nonIntersectingConcaveHull.add(pointA0);
      assertFalse(ConcaveHullMerger.isConcaveHullSelfIntersecting(nonIntersectingConcaveHull));

      // And backwards does not change things for intersecting.
      intersectingConcaveHull.clear();
      intersectingConcaveHull.add(pointA2);
      intersectingConcaveHull.add(pointA3);
      intersectingConcaveHull.add(pointA1);
      intersectingConcaveHull.add(pointA0);
      assertTrue(ConcaveHullMerger.isConcaveHullSelfIntersecting(intersectingConcaveHull));
   }

   @Test
   public void testFindIntersection()
   {
      ArrayList<Point2D> hullAVertices = new ArrayList<Point2D>();

      Point2D pointA0 = new Point2D(0.0, 0.0);
      Point2D pointA1 = new Point2D(0.0, 1.0);
      Point2D pointA2 = new Point2D(1.0, 1.0);
      Point2D pointA3 = new Point2D(1.0, 0.0);

      hullAVertices.add(pointA0);
      hullAVertices.add(pointA1);
      hullAVertices.add(pointA2);
      hullAVertices.add(pointA3);

      Point2D firstEndPoint = new Point2D(0.5, -0.5);
      Point2D secondEndPoint = new Point2D(0.5, 0.5);
      LineSegment2D edge = new LineSegment2D(firstEndPoint, secondEndPoint);

      Pair<Integer, Point2D> intersection = ConcaveHullMerger.findClosestIntersection(edge, hullAVertices, -1);

      assertTrue(intersection.getRight().epsilonEquals(new Point2D(0.5, 0.0), 1e-7));
      assertEquals(0, intersection.getLeft());
   }

   private Point2D[] convertToPointArray(double[][] hull)
   {
      Point2D[] points = new Point2D[hull.length];

      for (int i = 0; i < hull.length; i++)
      {
         points[i] = new Point2D(hull[i][0], hull[i][1]);
      }

      return points;
   }

   private ArrayList<Point2D> convertToPointArrayList(double[][] hull)
   {
      ArrayList<Point2D> points = new ArrayList<>();

      for (int i = 0; i < hull.length; i++)
      {
         points.add(new Point2D(hull[i][0], hull[i][1]));
      }

      return points;
   }

   private void assertConcaveHullContains(Point2D[] concaveHull, Point2D point)
   {
      assertConcaveHullContains(concaveHull, point.getX(), point.getY());
   }

   private void assertConcaveHullContains(Point2D[] concaveHull, double x, double y)
   {
      double epsilon = 1e-7;

      for (Point2D point : concaveHull)
      {
         if (point.epsilonEquals(new Point2D(x, y), epsilon))
            return;
      }
      String errorMessage = "Concave Hull does not contain (" + x + ", " + y + "). \nConcave Hull = ";
      for (Point2D point : concaveHull)
      {
         errorMessage += point + "\n";
      }
      fail(errorMessage);
   }

   private void visualizePlanarRegions(PlanarRegion... regions)
   {
      PlanarRegionsList list = new PlanarRegionsList(regions);
      visualizePlanarRegions(list);
   }

   private void visualizePlanarRegions(PlanarRegionsList planarRegions)
   {
      JavaFXApplicationCreator.createAJavaFXApplication();

      PlanarRegionsGraphic regionsGraphic = new PlanarRegionsGraphic(false);
      regionsGraphic.generateMeshes(planarRegions);
      regionsGraphic.update();

      final CountDownLatch countDownLatch = new CountDownLatch(1);

      Platform.runLater(new Runnable()
      {
         @Override
         public void run()
         {
            //            double preferredWidth = 1000.0;
            //            double preferredHeight = 1000.0;

            View3DFactory view3dFactory = new View3DFactory(1200, 800);
            view3dFactory.addCameraController(0.05, 2000.0, true);
            view3dFactory.addWorldCoordinateSystem(0.3);
            view3dFactory.addDefaultLighting();
            view3dFactory.addNodeToView(regionsGraphic);

            Stage stage = new Stage();
            stage.setTitle(getClass().getSimpleName());
            stage.setMaximized(false);
            stage.setScene(view3dFactory.getScene());

            stage.centerOnScreen();
            stage.show();
            countDownLatch.countDown();
         }
      });

      try
      {
         countDownLatch.await();
      }
      catch (InterruptedException e)
      {
      }
   }

   private class EnsureNoLoopsListener implements ConcaveHullMergerListener
   {
      @Override
      public void originalHulls(ArrayList<Point2D> hullOne, ArrayList<Point2D> hullTwo)
      {
      }

      @Override
      public void preprocessedHull(ArrayList<Point2D> hullOne, ArrayList<Point2D> hullTwo)
      {
      }

      @Override
      public void foundStartingVertexAndWorkingHull(Point2D startingVertex, ArrayList<Point2D> workingHull, boolean workingHullIsOne)
      {
      }

      @Override
      public void consideringWorkingEdge(LineSegment2D workingEdge, boolean workingHullIsOne)
      {
      }

      @Override
      public void foundIntersectionPoint(Point2D intersectionPoint, boolean workingHullIsOne)
      {
      }

      @Override
      public void hullGotLooped(ArrayList<Point2D> hullOne, ArrayList<Point2D> hullTwo, ArrayList<Point2D> mergedVertices)
      {
         fail("Hull got looped!");
      }

      @Override
      public void hullIsInvalid(ArrayList<Point2D> invalidHull)
      {
         fail("Hull is invalid");
      }

      @Override
      public void hullsAreInvalid(ArrayList<Point2D> invalidHullA, ArrayList<Point2D> invalidHullB)
      {
         fail("Hulls are invalid");
      }

   }
}
