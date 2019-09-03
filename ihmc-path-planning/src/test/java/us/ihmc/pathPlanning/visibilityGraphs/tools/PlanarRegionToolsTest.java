package us.ihmc.pathPlanning.visibilityGraphs.tools;

import static us.ihmc.robotics.Assert.*;

import static us.ihmc.euclid.tools.EuclidCoreRandomTools.nextDouble;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.commons.RandomNumbers;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DBasics;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class PlanarRegionToolsTest
{
   private static final int ITERATIONS = 1000;
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testComputeMinHeightOfRegionAAboveRegionB()
   {
      double[][] verticesA = new double[][] {{0.0, 0.0}, {0.0, 1.0}, {1.0, 1.0}, {1.0, 0.0}};
      ConvexPolygon2D convexPolygonA = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(verticesA));
      RigidBodyTransform transformA = new RigidBodyTransform();
      double heightAbove = 3.0;
      transformA.setTranslation(0.0, 0.0, heightAbove);
      PlanarRegion regionA = new PlanarRegion(transformA, convexPolygonA);

      double[][] verticesB = new double[][] {{0.0, 0.0}, {0.0, 1.0}, {1.0, 1.0}, {1.0, 0.0}};
      ConvexPolygon2D convexPolygonB = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(verticesB));
      RigidBodyTransform transformB = new RigidBodyTransform();
      PlanarRegion regionB = new PlanarRegion(transformB, convexPolygonB);

      double minHeightOfRegionAAboveRegionB = PlanarRegionTools.computeMinHeightOfRegionAAboveRegionB(regionA, regionB);
      assertEquals(heightAbove, minHeightOfRegionAAboveRegionB, EPSILON);

      // Now rotate the bottom one, lifting the plane and decreasing the height above.
      transformB = new RigidBodyTransform();
      double rotationAngleB = -0.4789;
      transformB.setRotationEuler(0.0, rotationAngleB, 0.0);
      regionB = new PlanarRegion(transformB, convexPolygonB);

      minHeightOfRegionAAboveRegionB = PlanarRegionTools.computeMinHeightOfRegionAAboveRegionB(regionA, regionB);
      assertEquals(heightAbove + Math.sin(rotationAngleB) * 1.0, minHeightOfRegionAAboveRegionB, EPSILON);

      // Now rotate the top one, lowering the points and decreasing the height above also.
      transformA = new RigidBodyTransform();
      double rotationAngleA = 0.123;
      transformA.setTranslation(0.0, 0.0, heightAbove);
      transformA.setRotationEuler(0.0, rotationAngleA, 0.0);
      regionA = new PlanarRegion(transformA, convexPolygonA);

      minHeightOfRegionAAboveRegionB = PlanarRegionTools.computeMinHeightOfRegionAAboveRegionB(regionA, regionB);
      double expectedMinHeight = heightAbove - Math.cos(rotationAngleB) * Math.tan(rotationAngleA) * 1.0 + Math.sin(rotationAngleB) * 1.0;
      assertEquals(expectedMinHeight, minHeightOfRegionAAboveRegionB, EPSILON);

      // Have one on top be vertical:

      transformA = new RigidBodyTransform();
      rotationAngleA = -Math.PI / 2.0;
      transformA.setTranslation(0.0, 0.0, heightAbove);
      transformA.setRotationEuler(0.0, rotationAngleA, 0.0);
      regionA = new PlanarRegion(transformA, convexPolygonA);

      minHeightOfRegionAAboveRegionB = PlanarRegionTools.computeMinHeightOfRegionAAboveRegionB(regionA, regionB);
      expectedMinHeight = heightAbove;
      assertEquals(expectedMinHeight, minHeightOfRegionAAboveRegionB, EPSILON);

   }

   @Test
   public void testProjectPointToPlanes()
   {
      ConvexPolygon2D convexPolygon = createUnitSquarePolygon();
      RigidBodyTransform squarePose = new RigidBodyTransform();
      PlanarRegion square = new PlanarRegion(squarePose, convexPolygon);

      Point3D pointToProject = new Point3D(0.0, 0.0, 5.0);
      PlanarRegionsList regions = new PlanarRegionsList();
      regions.addPlanarRegion(square);
      Point3D projectedPoint = PlanarRegionTools.projectPointToPlanes(pointToProject, regions);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(0.0, 0.0, 0.0), projectedPoint, 1e-10);
   }

   private ConvexPolygon2D createUnitSquarePolygon()
   {
      ConvexPolygon2D convexPolygon = new ConvexPolygon2D();
      convexPolygon.addVertex(new Point2D(-0.5, -0.5));
      convexPolygon.addVertex(new Point2D(0.5, -0.5));
      convexPolygon.addVertex(new Point2D(0.5, 0.5));
      convexPolygon.addVertex(new Point2D(-0.5, 0.5));
      convexPolygon.update();

      return convexPolygon;
   }

   @Test
   public void testTruncatePlanarRegionIfIntersectingWithPlane() throws Exception
   {
      Point3D groundOrigin = new Point3D();
      Vector3D groundNormal = new Vector3D(0.0, 0.0, 1.0);

      Point3D squareOrigin = new Point3D(0.0, 0.0, -0.001);
      Vector3D squareNormal = new Vector3D(0.0, -1.0, 0.0);
      AxisAngle squareOrientation = EuclidGeometryTools.axisAngleFromZUpToVector3D(squareNormal);
      RigidBodyTransform squarePose = new RigidBodyTransform(squareOrientation, squareOrigin);

      double squareSide = 4.0;

      Point2D[] concaveHullVertices = {new Point2D(0.0, 0.0), new Point2D(0.0, squareSide), new Point2D(squareSide, squareSide), new Point2D(squareSide, 0.0)};
      List<ConvexPolygon2D> convexPolygons = new ArrayList<>();
      convexPolygons.add(new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(concaveHullVertices)));
      PlanarRegion verticalSquare = new PlanarRegion(squarePose, concaveHullVertices, convexPolygons);

      Point3D[] expectedVerticesInWorld = Arrays.stream(concaveHullVertices).map(p -> toWorld(p, squarePose)).toArray(Point3D[]::new);
      expectedVerticesInWorld[0].addZ(0.001);
      expectedVerticesInWorld[3].addZ(0.001);

      PlanarRegion truncatedSquare = PlanarRegionTools.truncatePlanarRegionIfIntersectingWithPlane(groundOrigin, groundNormal, verticalSquare, 0.05, null);
      RigidBodyTransform truncatedTransform = new RigidBodyTransform();
      truncatedSquare.getTransformToWorld(truncatedTransform);
      EuclidCoreTestTools.assertRigidBodyTransformGeometricallyEquals(squarePose, truncatedTransform, EPSILON);

      Point3D[] actualVerticesInWorld = Arrays.stream(truncatedSquare.getConcaveHull()).map(p -> toWorld(p, squarePose)).toArray(Point3D[]::new);

      assertEquals(expectedVerticesInWorld.length, actualVerticesInWorld.length);

      for (int i = 0; i < expectedVerticesInWorld.length; i++)
         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedVerticesInWorld[i], actualVerticesInWorld[i], EPSILON);
   }
   
   @Test
   public void testTruncatePlanarRegionIfIntersectingWithPlaneTwo() throws Exception
   {
      Point3D groundOrigin = new Point3D(4.25, 8.5, 0.0);
      Vector3D groundNormal = new Vector3D(0.0, 0.0, 1.0);

      Point3D squareOrigin = new Point3D(8.5, 8.5, 0.0);
      Vector3D squareNormal = new Vector3D(-0.1, 0.1, 0.9899);
      squareNormal.normalize();
      AxisAngle squareOrientation = EuclidGeometryTools.axisAngleFromZUpToVector3D(squareNormal);
      RigidBodyTransform squarePose = new RigidBodyTransform(squareOrientation, squareOrigin);

      double squareSide = 4.0;

      Point2D[] concaveHullVertices = {new Point2D(-squareSide/2.0, squareSide/2.0), new Point2D(squareSide/2.0, squareSide/2.0), new Point2D(squareSide/2.0, -squareSide/2.0), new Point2D(-squareSide/2.0, -squareSide/2.0)};
      List<ConvexPolygon2D> convexPolygons = new ArrayList<>();
      convexPolygons.add(new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(concaveHullVertices)));
      PlanarRegion rotatedSquare = new PlanarRegion(squarePose, concaveHullVertices, convexPolygons);

      PlanarRegion truncatedSquare = PlanarRegionTools.truncatePlanarRegionIfIntersectingWithPlane(groundOrigin, groundNormal, rotatedSquare, 0.05, null);
            
//      TODO: Finish this test up with some asserts and more cases.
//      System.out.println(truncatedSquare);

   }

   public static Point3D toWorld(Point2D point2D, Transform transformToWorld)
   {
      Point3D inWorld = new Point3D(point2D);
      transformToWorld.transform(inWorld);
      return inWorld;
   }

   @Test
   public void testIsInsidePolygon() throws Exception
   {
      Random random = new Random(324534L);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test with convex polygon
         List<? extends Point2DReadOnly> convexPolygon2D = EuclidGeometryRandomTools.nextCircleBasedConvexPolygon2D(random, 10.0, 10.0, 100);
         int hullSize = EuclidGeometryPolygonTools.inPlaceGiftWrapConvexHull2D(convexPolygon2D);
         boolean clockwiseOrdered = random.nextBoolean();
         if (!clockwiseOrdered)
            Collections.reverse(convexPolygon2D.subList(0, hullSize));

         Point2DReadOnly[] convexPolygon2DArray = convexPolygon2D.subList(0, hullSize).toArray(new Point2DReadOnly[convexPolygon2D.size()]);

         Point2D centroid = new Point2D();
         EuclidGeometryPolygonTools.computeConvexPolyong2DArea(convexPolygon2D, hullSize, clockwiseOrdered, centroid);
         int vertexIndex = random.nextInt(hullSize);
         int nextVertexIndex = EuclidGeometryPolygonTools.next(vertexIndex, hullSize);
         Point2DReadOnly vertex = convexPolygon2D.get(vertexIndex);
         Point2DReadOnly nextVertex = convexPolygon2D.get(nextVertexIndex);

         Point2D pointOnEdge = new Point2D();
         pointOnEdge.interpolate(vertex, nextVertex, random.nextDouble());

         double alphaOutside = nextDouble(random, 1.0, 3.0);
         Point2D outsidePoint = new Point2D();
         outsidePoint.interpolate(centroid, pointOnEdge, alphaOutside);
         assertFalse(PlanarRegionTools.isPointInsidePolygon(convexPolygon2DArray, outsidePoint));

         double alphaInside = nextDouble(random, 0.0, 1.0);
         Point2D insidePoint = new Point2D();
         insidePoint.interpolate(centroid, pointOnEdge, alphaInside);
         assertTrue(PlanarRegionTools.isPointInsidePolygon(convexPolygon2DArray, insidePoint));
      }
   }

   @Test
   public void testIsInsidePolygonBug1() throws Exception
   {
      Point2D[] polygon = {new Point2D(-0.3, 0.5), new Point2D(0.3, 0.5), new Point2D(0.3, -0.5), new Point2D(-0.3, -0.5)};
      Point2D pointToCheck = new Point2D(-2.0, 0.5);

      assertFalse(PlanarRegionTools.isPointInsidePolygon(polygon, pointToCheck));
   }

   @Test
   public void testProjectPointToPlanesVertically()
   {
      Random random = new Random(1738L);

      // first test stacked regions
      List<PlanarRegion> listOfPlanarRegions = new ArrayList<>();
      int numberOfRegions = 5;
      ConvexPolygon2D polygonInWorld = EuclidGeometryRandomTools.nextConvexPolygon2D(random, 10.0, 10);
      polygonInWorld.update();
      Point2D[] concaveHull = polygonInWorld.getPolygonVerticesView().toArray(new Point2D[0]);
      Point2DReadOnly centroidInWorld = polygonInWorld.getCentroid();

      List<ConvexPolygon2D> polygons = new ArrayList<>();
      polygons.add(polygonInWorld);

      double layerSeparation = 0.10;
      for (int i = 0; i < numberOfRegions; i++)
      {
         RigidBodyTransform transform = new RigidBodyTransform();
         transform.setTranslation(0.0, 0.0, i * layerSeparation);
         PlanarRegion planarRegion = new PlanarRegion(transform, concaveHull, polygons);
         listOfPlanarRegions.add(planarRegion);
      }

      // test with point at centroid
      Point3DReadOnly pointToProject = new Point3D(centroidInWorld);

      Point3DReadOnly projectedPoint = PlanarRegionTools.projectPointToPlanesVertically(pointToProject, listOfPlanarRegions);
      Point3D expectedPoint = new Point3D(pointToProject);
      expectedPoint.setZ(layerSeparation * (numberOfRegions - 1));

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedPoint, projectedPoint, 1e-6);

      // test with point outside bounds
      pointToProject = new Point3D(15.0, 15.0, 100.0);

      projectedPoint = PlanarRegionTools.projectPointToPlanesVertically(pointToProject, listOfPlanarRegions);
      assertNull(projectedPoint);

      // test two slightly overlapping regions
      ConvexPolygon2D polygonA = new ConvexPolygon2D();
      polygonA.addVertex(0.5, 0.5);
      polygonA.addVertex(0.5, -0.5);
      polygonA.addVertex(-0.5, 0.5);
      polygonA.addVertex(-0.5, -0.5);
      polygonA.update();
      ConvexPolygon2D polygonB = new ConvexPolygon2D();
      polygonB.addVertex(0.5, 0.5);
      polygonB.addVertex(0.5, -0.5);
      polygonB.addVertex(-0.5, 0.5);
      polygonB.addVertex(-0.5, -0.5);
      polygonB.update();
      polygons.clear();
      polygons.add(polygonA);
      polygons.add(polygonB);
      RigidBodyTransform transformA = new RigidBodyTransform();
      RigidBodyTransform transformB = new RigidBodyTransform();
      transformA.setTranslation(new Vector3D(0.4, 0.0, 0.0));
      transformB.setTranslation(new Vector3D(-0.4, 0.0, 0.1));

      PlanarRegion regionA = new PlanarRegion(transformA, polygonA);
      PlanarRegion regionB = new PlanarRegion(transformB, polygonB);
      listOfPlanarRegions.clear();
      listOfPlanarRegions.add(regionA);
      listOfPlanarRegions.add(regionB);

      // middle
      pointToProject = new Point3D();
      projectedPoint = PlanarRegionTools.projectPointToPlanesVertically(pointToProject, listOfPlanarRegions);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(0.0, 0.0, 0.1), projectedPoint, 1e-6);

      // on the edge
      pointToProject = new Point3D(0.1 - 1e-5, 0.0, 0.0);
      projectedPoint = PlanarRegionTools.projectPointToPlanesVertically(pointToProject, listOfPlanarRegions);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(0.1 - 1e-5, 0.0, 0.1), projectedPoint, 1e-6);

      // past edge
      pointToProject = new Point3D(0.2, 0.0, 0.0);
      projectedPoint = PlanarRegionTools.projectPointToPlanesVertically(pointToProject, listOfPlanarRegions);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(0.2, 0.0, 0.0), projectedPoint, 1e-6);

      pointToProject = new Point3D(-0.2, 0.0, 0.0);
      projectedPoint = PlanarRegionTools.projectPointToPlanesVertically(pointToProject, listOfPlanarRegions);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(-0.2, 0.0, 0.1), projectedPoint, 1e-6);
   }

   @Test
   public void testFilterPlanarRegionsWithBoundingCircle()
   {
      Random random = new Random(1738L);

      for (int iter = 0; iter < 100; iter++)
      {
         double maxRegionDimension = 5.0;
         double maxRegionDistanceForGuaranteedOutOfBounds = Math.sqrt(2.0 * maxRegionDimension * maxRegionDimension);

         int numberOfPoints = 5;
         ConvexPolygon2D planarRegionPolygonA = new ConvexPolygon2D();
         ConvexPolygon2D planarRegionPolygonB = new ConvexPolygon2D();
         Point2D[] concaveHull = new Point2D[2 * numberOfPoints];
         for (int i = 0; i < numberOfPoints; i++)
         {
            Point2D pointA = EuclidCoreRandomTools.nextPoint2D(random, maxRegionDimension);
            Point2D pointB = EuclidCoreRandomTools.nextPoint2D(random, maxRegionDimension);
            planarRegionPolygonA.addVertex(pointA);
            planarRegionPolygonB.addVertex(pointB);

            concaveHull[2 * i] = pointA;
            concaveHull[2 * i + 1] = pointB;
         }
         planarRegionPolygonA.update();
         planarRegionPolygonB.update();

         double maxRegionDistance = Math.max(findFurthestPointFromOrigin(planarRegionPolygonA), findFurthestPointFromOrigin(planarRegionPolygonB));

         List<ConvexPolygon2D> polygons = new ArrayList<>();
         polygons.add(planarRegionPolygonA);
         polygons.add(planarRegionPolygonB);

         int numberOfRegionsWithinDistance = random.nextInt(10);
         int numberOfRegionsOutsideOfDistance = random.nextInt(20);

         Point3D randomOrigin = EuclidCoreRandomTools.nextPoint3D(random, 10.0);

         List<PlanarRegion> regionsWithinDistanceExpected = new ArrayList<>();
         List<PlanarRegion> regionsOutsideDistance = new ArrayList<>();
         List<PlanarRegion> allRegions = new ArrayList<>();
         for (int i = 0; i < numberOfRegionsWithinDistance; i++)
         {
            RigidBodyTransform transform = new RigidBodyTransform();
            Vector3D translation = EuclidCoreRandomTools.nextVector3D(random, -0.2 * maxRegionDistance, 0.2 * maxRegionDistance);
            translation.add(randomOrigin);
            transform.setTranslation(translation);
            PlanarRegion planarRegion = new PlanarRegion(transform, concaveHull, polygons);
            regionsWithinDistanceExpected.add(planarRegion);
            allRegions.add(planarRegion);
         }

         for (int i = 0; i < numberOfRegionsOutsideOfDistance; i++)
         {
            RigidBodyTransform transform = new RigidBodyTransform();
            double xSign = RandomNumbers.nextBoolean(random, 0.5) ? 1.0 : -1.0;
            double ySign = RandomNumbers.nextBoolean(random, 0.5) ? 1.0 : -1.0;
            double zSign = RandomNumbers.nextBoolean(random, 0.5) ? 1.0 : -1.0;

            double xTranslation = xSign * RandomNumbers.nextDouble(random, maxRegionDistanceForGuaranteedOutOfBounds, 1e5);
            double yTranslation = ySign * RandomNumbers.nextDouble(random, maxRegionDistanceForGuaranteedOutOfBounds, 1e5);
            double zTranslation = zSign * RandomNumbers.nextDouble(random, maxRegionDistanceForGuaranteedOutOfBounds, 1e5);
            Vector3D translation = new Vector3D(xTranslation, yTranslation, zTranslation);
            translation.add(randomOrigin);

            transform.setTranslation(translation);

            PlanarRegion planarRegion = new PlanarRegion(transform, concaveHull, polygons);
            regionsOutsideDistance.add(planarRegion);
            allRegions.add(planarRegion);
         }

         List<PlanarRegion> regionsWithinDistance = PlanarRegionTools.filterPlanarRegionsWithBoundingCircle(new Point2D(randomOrigin), maxRegionDistance,
                                                                                                            allRegions);

         assertEquals(regionsWithinDistanceExpected.size(), regionsWithinDistance.size());
         for (int i = 0; i < regionsWithinDistance.size(); i++)
         {
            assertTrue(regionsWithinDistanceExpected.contains(regionsWithinDistance.get(i)));
            assertFalse(regionsOutsideDistance.contains(regionsWithinDistance.get(i)));
         }
      }
   }

   @Test
   public void testFilterPlanarRegionsWithBoundingCirclePointWithinBigRegion()
   {
      ConvexPolygon2D polygon2D = new ConvexPolygon2D();
      polygon2D.addVertex(10.0, 10.0);
      polygon2D.addVertex(10.0, -10.0);
      polygon2D.addVertex(-10.0, -10.0);
      polygon2D.addVertex(-10.0, 10.0);
      polygon2D.update();
      List<ConvexPolygon2D> polygons = new ArrayList<>();
      polygons.add(polygon2D);

      Point2D[] concaveHull = polygon2D.getPolygonVerticesView().toArray(new Point2D[0]);

      RigidBodyTransform transform = new RigidBodyTransform();
      PlanarRegion planarRegion = new PlanarRegion(transform, concaveHull, polygons);
      List<PlanarRegion> planarRegionList = new ArrayList<>();
      planarRegionList.add(planarRegion);

      // at middle of planar region
      List<PlanarRegion> regionsWithinDistance = PlanarRegionTools.filterPlanarRegionsWithBoundingCircle(new Point2D(), 1.0, planarRegionList);

      assertTrue(regionsWithinDistance.contains(planarRegion));

      // outside the planar region, but still within the distance
      regionsWithinDistance = PlanarRegionTools.filterPlanarRegionsWithBoundingCircle(new Point2D(10.5, 0.0), 1.0, planarRegionList);

      assertTrue(regionsWithinDistance.contains(planarRegion));
   }

   @Test
   public void testTrivialCase() throws Exception
   {
      // polygons forming a "|"-shaped region.
      List<ConvexPolygon2D> region1ConvexPolygons = new ArrayList<>();
      ConvexPolygon2D polygon1 = new ConvexPolygon2D();
      polygon1.addVertex(5.0, 1.0);
      polygon1.addVertex(5.0, -1.0);
      polygon1.addVertex(-5.0, -1.0);
      polygon1.addVertex(-5.0, 1.0);

      region1ConvexPolygons.add(polygon1);
      for (ConvexPolygon2D convexPolygon : region1ConvexPolygons)
         convexPolygon.update();

      // polygons forming a "--"-shaped region.
      List<ConvexPolygon2D> region2ConvexPolygons = new ArrayList<>();
      ConvexPolygon2D polygon2 = new ConvexPolygon2D();
      polygon2.addVertex(1.0, 5.0);
      polygon2.addVertex(1.0, -5.0);
      polygon2.addVertex(-1.0, -5.0);
      polygon2.addVertex(-1.0, 5.0);

      region2ConvexPolygons.add(polygon2);
      for (ConvexPolygon2D convexPolygon : region2ConvexPolygons)
         convexPolygon.update();

      RigidBodyTransform region1Transform = new RigidBodyTransform();
      RigidBodyTransform region2Transform = new RigidBodyTransform();

      region2Transform.setTranslation(0.0, 0.0, 1.0);

      PlanarRegion planarRegion1 = new PlanarRegion(region1Transform, region1ConvexPolygons);
      PlanarRegion planarRegion2 = new PlanarRegion(region2Transform, region2ConvexPolygons);
      List<PlanarRegion> planarRegions = new ArrayList<>();
      planarRegions.add(planarRegion1);
      planarRegions.add(planarRegion2);

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(planarRegions);

      List<PlanarRegion> result;

      ConvexPolygon2D convexPolygon = new ConvexPolygon2D();
      convexPolygon.addVertex(0.2, 0.2);
      convexPolygon.addVertex(0.2, -0.2);
      convexPolygon.addVertex(-0.2, -0.2);
      convexPolygon.addVertex(-0.2, 0.2);
      convexPolygon.update();

      // Do a bunch of trivial queries with findPlanarRegionsIntersectingPolygon(ConvexPolygon2d convexPolygon)
      result = PlanarRegionTools.findPlanarRegionsIntersectingPolygon(convexPolygon, planarRegionsList);
      assertEquals(2, result.size());
      result = PlanarRegionTools.findPlanarRegionsIntersectingPolygon(translateConvexPolygon(2.0, 0.0, convexPolygon), planarRegionsList);
      assertEquals(1, result.size());
      assertTrue(result.get(0).epsilonEquals(planarRegion1, 1.0e-10));
      result = PlanarRegionTools.findPlanarRegionsIntersectingPolygon(translateConvexPolygon(-2.0, 0.0, convexPolygon), planarRegionsList);
      assertEquals(1, result.size());
      assertTrue(result.get(0).epsilonEquals(planarRegion1, 1.0e-10));
      result = PlanarRegionTools.findPlanarRegionsIntersectingPolygon(translateConvexPolygon(0.0, 2.0, convexPolygon), planarRegionsList);
      assertEquals(1, result.size());
      assertTrue(result.get(0).epsilonEquals(planarRegion2, 1.0e-10));
      result = PlanarRegionTools.findPlanarRegionsIntersectingPolygon(translateConvexPolygon(0.0, -2.0, convexPolygon), planarRegionsList);
      assertEquals(1, result.size());
      assertTrue(result.get(0).epsilonEquals(planarRegion2, 1.0e-10));
      result = PlanarRegionTools.findPlanarRegionsIntersectingPolygon(translateConvexPolygon(2.0, 2.0, convexPolygon), planarRegionsList);
      assertNull(result);
      result = PlanarRegionTools.findPlanarRegionsIntersectingPolygon(translateConvexPolygon(2.0, -2.0, convexPolygon), planarRegionsList);
      assertNull(result);
      result = PlanarRegionTools.findPlanarRegionsIntersectingPolygon(translateConvexPolygon(-2.0, -2.0, convexPolygon), planarRegionsList);
      assertNull(result);
      result = PlanarRegionTools.findPlanarRegionsIntersectingPolygon(translateConvexPolygon(-2.0, 2.0, convexPolygon), planarRegionsList);
      assertNull(result);
   }

   static ConvexPolygon2DBasics translateConvexPolygon(double xTranslation, double yTranslation, ConvexPolygon2DReadOnly convexPolygon)
   {
      Vector2D translation = new Vector2D(xTranslation, yTranslation);
      return convexPolygon.translateCopy(translation);
   }

   @Test
   public void testFilterPlanarRegionsWithBoundingCapsule()
   {
      Random random = new Random(1738L);

      for (int iter = 0; iter < 100; iter++)
      {
         double maxRegionDimension = 5.0;
         double maxRegionDistanceForGuaranteedOutOfBounds = Math.sqrt(2.0 * maxRegionDimension * maxRegionDimension);

         int numberOfPoints = 5;
         ConvexPolygon2D planarRegionPolygonA = new ConvexPolygon2D();
         ConvexPolygon2D planarRegionPolygonB = new ConvexPolygon2D();
         Point2D[] concaveHull = new Point2D[2 * numberOfPoints];
         for (int i = 0; i < numberOfPoints; i++)
         {
            Point2D pointA = EuclidCoreRandomTools.nextPoint2D(random, maxRegionDimension);
            Point2D pointB = EuclidCoreRandomTools.nextPoint2D(random, maxRegionDimension);
            planarRegionPolygonA.addVertex(pointA);
            planarRegionPolygonB.addVertex(pointB);

            concaveHull[2 * i] = pointA;
            concaveHull[2 * i + 1] = pointB;
         }
         planarRegionPolygonA.update();
         planarRegionPolygonB.update();

         double maxRegionDistance = Math.max(findFurthestPointFromOrigin(planarRegionPolygonA), findFurthestPointFromOrigin(planarRegionPolygonB));

         List<ConvexPolygon2D> polygons = new ArrayList<>();
         polygons.add(planarRegionPolygonA);
         polygons.add(planarRegionPolygonB);

         int numberOfRegionsWithinDistance = random.nextInt(10);
         int numberOfRegionsOutsideOfDistance = random.nextInt(20);

         Point3D randomOriginStart = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Point3D randomOriginEnd = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         LineSegment3D randomSegment = new LineSegment3D(randomOriginStart, randomOriginEnd);
         Point3D midpoint = new Point3D();
         midpoint.interpolate(randomOriginStart, randomOriginEnd, 0.5);

         List<PlanarRegion> regionsWithinDistanceExpected = new ArrayList<>();
         List<PlanarRegion> regionsOutsideDistance = new ArrayList<>();
         List<PlanarRegion> allRegions = new ArrayList<>();
         for (int i = 0; i < numberOfRegionsWithinDistance; i++)
         {
            RigidBodyTransform transform = new RigidBodyTransform();
            Vector3D translation = EuclidCoreRandomTools.nextVector3D(random, -0.2 * maxRegionDistance, 0.2 * maxRegionDistance);
            translation.add(midpoint);
            transform.setTranslation(translation);
            PlanarRegion planarRegion = new PlanarRegion(transform, concaveHull, polygons);
            regionsWithinDistanceExpected.add(planarRegion);
            allRegions.add(planarRegion);
         }

         for (int i = 0; i < numberOfRegionsOutsideOfDistance; i++)
         {
            RigidBodyTransform transform = new RigidBodyTransform();
            double xSign = RandomNumbers.nextBoolean(random, 0.5) ? 1.0 : -1.0;
            double ySign = RandomNumbers.nextBoolean(random, 0.5) ? 1.0 : -1.0;
            double zSign = RandomNumbers.nextBoolean(random, 0.5) ? 1.0 : -1.0;

            double xTranslation = xSign * RandomNumbers.nextDouble(random, maxRegionDistanceForGuaranteedOutOfBounds, 1e5);
            double yTranslation = ySign * RandomNumbers.nextDouble(random, maxRegionDistanceForGuaranteedOutOfBounds, 1e5);
            double zTranslation = zSign * RandomNumbers.nextDouble(random, maxRegionDistanceForGuaranteedOutOfBounds, 1e5);
            Vector3D translation = new Vector3D(xTranslation, yTranslation, zTranslation);
            double distanceStart = randomOriginStart.distance(new Point3D(translation));
            double distanceEnd = randomOriginStart.distance(new Point3D(translation));

            if (distanceStart < distanceEnd)
               translation.add(randomOriginStart);
            else
               translation.add(randomOriginEnd);

            transform.setTranslation(translation);

            PlanarRegion planarRegion = new PlanarRegion(transform, concaveHull, polygons);
            regionsOutsideDistance.add(planarRegion);
            allRegions.add(planarRegion);
         }

         List<PlanarRegion> regionsWithinDistance = PlanarRegionTools.filterPlanarRegionsWithBoundingCapsule(randomSegment, maxRegionDistance, allRegions);

         assertEquals(regionsWithinDistanceExpected.size(), regionsWithinDistance.size());
         for (int i = 0; i < regionsWithinDistance.size(); i++)
         {
            assertTrue(regionsWithinDistanceExpected.contains(regionsWithinDistance.get(i)));
            assertFalse(regionsOutsideDistance.contains(regionsWithinDistance.get(i)));
         }
      }
   }

   @Test
   public void testFilterPlanarRegionsWithBoundingCapsulePointWithinBigRegion()
   {
      ConvexPolygon2D polygon2D = new ConvexPolygon2D();
      polygon2D.addVertex(10.0, 10.0);
      polygon2D.addVertex(10.0, -10.0);
      polygon2D.addVertex(-10.0, -10.0);
      polygon2D.addVertex(-10.0, 10.0);
      polygon2D.update();
      List<ConvexPolygon2D> polygons = new ArrayList<>();
      polygons.add(polygon2D);

      Point2D[] concaveHull = polygon2D.getPolygonVerticesView().toArray(new Point2D[0]);

      RigidBodyTransform transform = new RigidBodyTransform();
      PlanarRegion planarRegion = new PlanarRegion(transform, concaveHull, polygons);
      List<PlanarRegion> planarRegionList = new ArrayList<>();
      planarRegionList.add(planarRegion);

      // at middle of planar region
      List<PlanarRegion> regionsWithinDistance = PlanarRegionTools.filterPlanarRegionsWithBoundingCapsule(new Point3D(0.1, 0.0, 0.0),
                                                                                                          new Point3D(-0.1, 0.0, 0.0), 1.0, planarRegionList);

      assertTrue(regionsWithinDistance.contains(planarRegion));

      // outside the planar region, but still within the distance
      regionsWithinDistance = PlanarRegionTools.filterPlanarRegionsWithBoundingCapsule(new Point3D(10.5, 0.1, 0.0), new Point3D(10.5, -0.1, 0.0), 1.0,
                                                                                       planarRegionList);

      assertTrue(regionsWithinDistance.contains(planarRegion));
   }

   @Test
   public void testIsRegionAOverlapingWithRegionB()
   {
      //TODO: +++JerryPratt: Get this to pass by fixing isRegionAOverlapingWithRegionB()
      ConvexPolygon2D polygonA = new ConvexPolygon2D();
      polygonA.addVertex(1.0, 1.0);
      polygonA.addVertex(1.0, -1.0);
      polygonA.addVertex(-1.0, -1.0);
      polygonA.addVertex(-1.0, 1.0);
      polygonA.update();

      ConvexPolygon2D polygonB = new ConvexPolygon2D();
      polygonB.addVertex(3.1, 1.0);
      polygonB.addVertex(3.1, -1.0);
      polygonB.addVertex(1.1, -1.0);
      polygonB.addVertex(1.1, 1.0);
      polygonB.update();

      PlanarRegion regionA = new PlanarRegion(new RigidBodyTransform(), polygonA);
      PlanarRegion regionB = new PlanarRegion(new RigidBodyTransform(), polygonB);

      double epsilonForCheck = 0.0;
      assertFalse(PlanarRegionTools.isRegionAOverlapingWithRegionB(regionA, regionB, epsilonForCheck));
      assertFalse(PlanarRegionTools.isRegionAOverlapingWithRegionB(regionB, regionA, epsilonForCheck));
      epsilonForCheck = 0.099;
      assertFalse(PlanarRegionTools.isRegionAOverlapingWithRegionB(regionA, regionB, epsilonForCheck));
      assertFalse(PlanarRegionTools.isRegionAOverlapingWithRegionB(regionB, regionA, epsilonForCheck));
      epsilonForCheck = 0.101;
      assertTrue(PlanarRegionTools.isRegionAOverlapingWithRegionB(regionA, regionB, epsilonForCheck));
      assertTrue(PlanarRegionTools.isRegionAOverlapingWithRegionB(regionB, regionA, epsilonForCheck));
      epsilonForCheck = 100.0;
      assertTrue(PlanarRegionTools.isRegionAOverlapingWithRegionB(regionA, regionB, epsilonForCheck));
      assertTrue(PlanarRegionTools.isRegionAOverlapingWithRegionB(regionB, regionA, epsilonForCheck));

      RigidBodyTransform transformC = new RigidBodyTransform();
      transformC.setRotationEuler(0.0, Math.PI / 2.0, 0.0);
      PlanarRegion regionC = new PlanarRegion(transformC, polygonA);
      epsilonForCheck = 0.0;
      assertFalse(PlanarRegionTools.isRegionAOverlapingWithRegionB(regionB, regionC, epsilonForCheck));
      assertFalse(PlanarRegionTools.isRegionAOverlapingWithRegionB(regionC, regionB, epsilonForCheck));
      epsilonForCheck = 1.098;
      assertFalse(PlanarRegionTools.isRegionAOverlapingWithRegionB(regionB, regionC, epsilonForCheck));
      assertFalse(PlanarRegionTools.isRegionAOverlapingWithRegionB(regionC, regionB, epsilonForCheck));
      epsilonForCheck = 1.102;
      assertTrue(PlanarRegionTools.isRegionAOverlapingWithRegionB(regionB, regionC, epsilonForCheck));
      assertTrue(PlanarRegionTools.isRegionAOverlapingWithRegionB(regionC, regionB, epsilonForCheck));
      epsilonForCheck = 100.0;
      assertTrue(PlanarRegionTools.isRegionAOverlapingWithRegionB(regionB, regionC, epsilonForCheck));
      assertTrue(PlanarRegionTools.isRegionAOverlapingWithRegionB(regionC, regionB, epsilonForCheck));
   }

   @Test
   public void testFindPlanarRegionsContainingPointByProjectionOntoXYPlane() throws Exception
   {
      // polygons forming a "|"-shaped region.
      List<ConvexPolygon2D> region1ConvexPolygons = new ArrayList<>();
      ConvexPolygon2D polygon1 = new ConvexPolygon2D();
      polygon1.addVertex(5.0, 1.0);
      polygon1.addVertex(5.0, -1.0);
      polygon1.addVertex(-5.0, -1.0);
      polygon1.addVertex(-5.0, 1.0);

      region1ConvexPolygons.add(polygon1);
      for (ConvexPolygon2D convexPolygon : region1ConvexPolygons)
         convexPolygon.update();


      // polygons forming a "--"-shaped region.
      List<ConvexPolygon2D> region2ConvexPolygons = new ArrayList<>();
      ConvexPolygon2D polygon2 = new ConvexPolygon2D();
      polygon2.addVertex(1.0, 5.0);
      polygon2.addVertex(1.0, -5.0);
      polygon2.addVertex(-1.0, -5.0);
      polygon2.addVertex(-1.0, 5.0);

      region2ConvexPolygons.add(polygon2);
      for (ConvexPolygon2D convexPolygon : region2ConvexPolygons)
         convexPolygon.update();

      RigidBodyTransform region1Transform = new RigidBodyTransform();
      RigidBodyTransform region2Transform = new RigidBodyTransform();

      region2Transform.setTranslation(0.0, 0.0, 1.0);

      PlanarRegion planarRegion1 = new PlanarRegion(region1Transform, region1ConvexPolygons);
      PlanarRegion planarRegion2 = new PlanarRegion(region2Transform, region2ConvexPolygons);
      List<PlanarRegion> planarRegions = new ArrayList<>();
      planarRegions.add(planarRegion1);
      planarRegions.add(planarRegion2);


      Point2D point2d = new Point2D();
      List<PlanarRegion> result;

      // Do a bunch of trivial queries with findPlanarRegionsContainingPointByProjectionOntoXYPlane(double x, double y)
      result = PlanarRegionTools.findPlanarRegionsContainingPointByProjectionOntoXYPlane(planarRegions, 0.0, 0.0);
      assertEquals(2, result.size());
      result = PlanarRegionTools.findPlanarRegionsContainingPointByProjectionOntoXYPlane(planarRegions, 2.0, 0.0);
      assertEquals(1, result.size());
      assertTrue(result.get(0).epsilonEquals(planarRegion1, 1.0e-10));
      result = PlanarRegionTools.findPlanarRegionsContainingPointByProjectionOntoXYPlane(planarRegions, -2.0, 0.0);
      assertEquals(1, result.size());
      assertTrue(result.get(0).epsilonEquals(planarRegion1, 1.0e-10));
      result = PlanarRegionTools.findPlanarRegionsContainingPointByProjectionOntoXYPlane(planarRegions, 0.0, 2.0);
      assertEquals(1, result.size());
      assertTrue(result.get(0).epsilonEquals(planarRegion2, 1.0e-10));
      result = PlanarRegionTools.findPlanarRegionsContainingPointByProjectionOntoXYPlane(planarRegions, 0.0, -2.0);
      assertEquals(1, result.size());
      assertTrue(result.get(0).epsilonEquals(planarRegion2, 1.0e-10));
      result = PlanarRegionTools.findPlanarRegionsContainingPointByProjectionOntoXYPlane(planarRegions, 2.0, 2.0);
      assertNull(result);
      result = PlanarRegionTools.findPlanarRegionsContainingPointByProjectionOntoXYPlane(planarRegions, 2.0, -2.0);
      assertNull(result);
      result = PlanarRegionTools.findPlanarRegionsContainingPointByProjectionOntoXYPlane(planarRegions, -2.0, -2.0);
      assertNull(result);
      result = PlanarRegionTools.findPlanarRegionsContainingPointByProjectionOntoXYPlane(planarRegions, -2.0, 2.0);
      assertNull(result);

      // Do a bunch of trivial queries with findPlanarRegionsContainingPointByProjectionOntoXYPlane(Point2d point)
      point2d.set(0.0, 0.0);
      result = PlanarRegionTools.findPlanarRegionsContainingPointByProjectionOntoXYPlane(planarRegions, point2d);
      assertEquals(2, result.size());

      point2d.set(2.0, 0.0);
      result = PlanarRegionTools.findPlanarRegionsContainingPointByProjectionOntoXYPlane(planarRegions, point2d);
      assertEquals(1, result.size());
      assertTrue(result.get(0).epsilonEquals(planarRegion1, 1.0e-10));
      point2d.set(-2.0, 0.0);
      result = PlanarRegionTools.findPlanarRegionsContainingPointByProjectionOntoXYPlane(planarRegions, point2d);
      assertEquals(1, result.size());
      assertTrue(result.get(0).epsilonEquals(planarRegion1, 1.0e-10));
      point2d.set(0.0, 2.0);
      result = PlanarRegionTools.findPlanarRegionsContainingPointByProjectionOntoXYPlane(planarRegions, point2d);
      assertEquals(1, result.size());
      assertTrue(result.get(0).epsilonEquals(planarRegion2, 1.0e-10));
      point2d.set(0.0, -2.0);
      result = PlanarRegionTools.findPlanarRegionsContainingPointByProjectionOntoXYPlane(planarRegions, point2d);
      assertEquals(1, result.size());
      assertTrue(result.get(0).epsilonEquals(planarRegion2, 1.0e-10));
      point2d.set(2.0, 2.0);
      result = PlanarRegionTools.findPlanarRegionsContainingPointByProjectionOntoXYPlane(planarRegions, point2d);
      assertNull(result);
      point2d.set(2.0, -2.0);
      result = PlanarRegionTools.findPlanarRegionsContainingPointByProjectionOntoXYPlane(planarRegions, point2d);
      assertNull(result);
      point2d.set(-2.0, -2.0);
      result = PlanarRegionTools.findPlanarRegionsContainingPointByProjectionOntoXYPlane(planarRegions, point2d);
      assertNull(result);
      point2d.set(-2.0, 2.0);
      result = PlanarRegionTools.findPlanarRegionsContainingPointByProjectionOntoXYPlane(planarRegions, point2d);
      assertNull(result);

      Point3D point3d = new Point3D();
      double epsilon = 1.0e-3;

      // Do a bunch of trivial queries with findPlanarRegionsContainingPoint(Point3D point, double epsilon)
      point3d.set(0.0, 0.0, 0.0);
      result = PlanarRegionTools.findPlanarRegionsContainingPoint(planarRegions, point3d, epsilon);
      assertEquals(1, result.size());
      assertTrue(result.get(0).epsilonEquals(planarRegion1, 1.0e-10));
      point3d.set(0.0, 0.0, 1.0);
      result = PlanarRegionTools.findPlanarRegionsContainingPoint(planarRegions, point3d, epsilon);
      assertEquals(1, result.size());
      assertTrue(result.get(0).epsilonEquals(planarRegion2, 1.0e-10));
      point3d.set(0.0, 0.0, 0.5);
      result = PlanarRegionTools.findPlanarRegionsContainingPoint(planarRegions, point3d, epsilon);
      assertNull(result);
      result = PlanarRegionTools.findPlanarRegionsContainingPoint(planarRegions, point3d, 0.51);
      assertEquals(2, result.size());

      ConvexPolygon2D convexPolygon = new ConvexPolygon2D();
      convexPolygon.addVertex(0.2, 0.2);
      convexPolygon.addVertex(0.2, -0.2);
      convexPolygon.addVertex(-0.2, -0.2);
      convexPolygon.addVertex(-0.2, 0.2);
      convexPolygon.update();

   }

   @Test
   public void testIsPlanarRegionAAbovePlanarRegionB()
   {
      ConvexPolygon2D polygonA = new ConvexPolygon2D();
      polygonA.addVertex(1.0, 1.0);
      polygonA.addVertex(1.0, -1.0);
      polygonA.addVertex(-1.0, -1.0);
      polygonA.addVertex(-1.0, 1.0);
      polygonA.update();

      ConvexPolygon2D polygonB = new ConvexPolygon2D();
      polygonB.addVertex(3.0, 1.0);
      polygonB.addVertex(3.0, -1.0);
      polygonB.addVertex(1.0, -1.0);
      polygonB.addVertex(1.0, 1.0);
      polygonB.update();

      PlanarRegion regionA = new PlanarRegion(new RigidBodyTransform(), polygonA);
      PlanarRegion regionB = new PlanarRegion(new RigidBodyTransform(), polygonB);

      double epsilon = 0.01;

      assertFalse(PlanarRegionTools.isPlanarRegionAAbovePlanarRegionB(regionA, regionB, epsilon));
      assertFalse(PlanarRegionTools.isPlanarRegionAAbovePlanarRegionB(regionB, regionA, epsilon));

      RigidBodyTransform transformOne = new RigidBodyTransform();
      transformOne.setTranslation(0.0, 0.0, 0.99 * epsilon);
      regionA = new PlanarRegion(transformOne, polygonA);
      regionB = new PlanarRegion(new RigidBodyTransform(), polygonB);
      assertFalse(PlanarRegionTools.isPlanarRegionAAbovePlanarRegionB(regionA, regionB, epsilon));
      assertFalse(PlanarRegionTools.isPlanarRegionAAbovePlanarRegionB(regionB, regionA, epsilon));

      transformOne = new RigidBodyTransform();
      transformOne.setTranslation(0.0, 0.0, 1.01 * epsilon);
      regionA = new PlanarRegion(transformOne, polygonA);
      regionB = new PlanarRegion(new RigidBodyTransform(), polygonB);
      assertTrue(PlanarRegionTools.isPlanarRegionAAbovePlanarRegionB(regionA, regionB, epsilon));
      assertFalse(PlanarRegionTools.isPlanarRegionAAbovePlanarRegionB(regionB, regionA, epsilon));

      transformOne = new RigidBodyTransform();
      transformOne.setTranslation(0.0, 0.0, 0.0);
      transformOne.setRotationEuler(0.0, Math.PI / 4.0, 0.0);
      RigidBodyTransform transformTwo = new RigidBodyTransform();
      transformTwo.setTranslation(-10.0, 0.0, 0.0);
      regionA = new PlanarRegion(transformOne, polygonA);
      regionB = new PlanarRegion(transformTwo, polygonB);
      assertTrue(PlanarRegionTools.isPlanarRegionAAbovePlanarRegionB(regionA, regionB, epsilon));
      assertFalse(PlanarRegionTools.isPlanarRegionAAbovePlanarRegionB(regionB, regionA, epsilon));

      transformTwo = new RigidBodyTransform();
      transformTwo.setTranslation(10.0, 0.0, 0.0);
      regionA = new PlanarRegion(transformOne, polygonA);
      regionB = new PlanarRegion(transformTwo, polygonB);
      assertTrue(PlanarRegionTools.isPlanarRegionAAbovePlanarRegionB(regionA, regionB, epsilon));
      assertTrue(PlanarRegionTools.isPlanarRegionAAbovePlanarRegionB(regionB, regionA, epsilon));

      transformTwo = new RigidBodyTransform();
      transformTwo.setTranslation(0.0, 0.0, 0.0);
      regionA = new PlanarRegion(transformOne, polygonA);
      regionB = new PlanarRegion(transformTwo, polygonB);
      assertTrue(PlanarRegionTools.isPlanarRegionAAbovePlanarRegionB(regionA, regionB, epsilon));
      assertTrue(PlanarRegionTools.isPlanarRegionAAbovePlanarRegionB(regionB, regionA, epsilon));
   }

   @Test
   public void testGetLocalBoundingBox2DInLocal()
   {
      ConvexPolygon2D polygonA = new ConvexPolygon2D();
      polygonA.addVertex(4.1, 5.0);
      polygonA.addVertex(4.0, 3.0);
      polygonA.addVertex(2.0, 3.1);
      polygonA.addVertex(2.1, 5.1);
      polygonA.update();

      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      transformToWorld.setRotationYawPitchRoll(1.2, 3.4, 5.6);
      transformToWorld.setTranslation(-1.0, 2.2, 3.4);

      PlanarRegion regionA = new PlanarRegion(transformToWorld, polygonA);
      BoundingBox2D boundingBox2D = PlanarRegionTools.getLocalBoundingBox2DInLocal(regionA);
      assertEquals(boundingBox2D, new BoundingBox2D(2.0, 3.0, 4.1, 5.1));
   }

   private static double findFurthestPointFromOrigin(ConvexPolygon2D polygon)
   {
      Point2D origin = new Point2D();
      double distance = 0.0;
      for (int i = 0; i < polygon.getNumberOfVertices(); i++)
      {
         distance = Math.max(distance, polygon.getVertex(i).distance(origin));
      }

      return distance;
   }

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForClass(PlanarRegionTools.class, PlanarRegionToolsTest.class);
   }
}