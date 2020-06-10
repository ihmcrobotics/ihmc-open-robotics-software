package us.ihmc.robotics.geometry;

import static us.ihmc.euclid.tools.EuclidCoreRandomTools.nextDouble;
import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertNull;
import static us.ihmc.robotics.Assert.assertTrue;

import java.util.*;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.commons.RandomNumbers;
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
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class PlanarRegionToolsTest
{
   private static final int ITERATIONS = 1000;
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testProjectInZToPlanarRegion()
   {
      Random random = new Random(1738L);
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D randomPointToProject = EuclidCoreRandomTools.nextPoint3D(random, 100.0);
         PlanarRegion randomRegion = new PlanarRegion(EuclidCoreRandomTools.nextRigidBodyTransform(random), new ArrayList<>());

         Point3DReadOnly projectedRandomPoint = PlanarRegionTools.projectInZToPlanarRegion(randomPointToProject, randomRegion);

         Vector3DReadOnly surfaceNormalInWorld = randomRegion.getNormal();

         RigidBodyTransformReadOnly transformToWorld = randomRegion.getTransformToWorld();

         Point3D planarRegionReferencePointInWorld = new Point3D(0.0, 0.0, 0.0);
         transformToWorld.transform(planarRegionReferencePointInWorld);

         Vector3DReadOnly verticalLine = new Vector3D(0.0, 0.0, 1.0);

         Point3D expectedProjectedPoint = EuclidGeometryTools.intersectionBetweenLine3DAndPlane3D(planarRegionReferencePointInWorld, surfaceNormalInWorld,
                                                                                                  randomPointToProject, verticalLine);

         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedProjectedPoint, projectedRandomPoint, 1e-8);
      }
   }

   @Test
   public void testAverageCentroid2DInLocal()
   {
      ConvexPolygon2D bigSquare = new ConvexPolygon2D();
      bigSquare.addVertex(10.0, 10.0);
      bigSquare.addVertex(10.0, 0.0);
      bigSquare.addVertex(-10.0, 0.0);
      bigSquare.addVertex(-10.0, 10.0);
      bigSquare.update();

      ConvexPolygon2D smallSquare = new ConvexPolygon2D();
      smallSquare.addVertex(5.0, -5.0);
      smallSquare.addVertex(5.0, 0.0);
      smallSquare.addVertex(-5.0, 0.0);
      smallSquare.addVertex(-5.0, -5.0);
      smallSquare.update();

      List<ConvexPolygon2D> squares = new ArrayList<>();
      squares.add(smallSquare);
      squares.add(bigSquare);
      PlanarRegion planarRegion = new PlanarRegion(new RigidBodyTransform(), squares);

      double totalArea = bigSquare.getArea() + smallSquare.getArea();
      assertEquals(totalArea, PlanarRegionTools.computePlanarRegionArea(planarRegion), EPSILON);

      Point2D expectedCentroid = new Point2D(bigSquare.getCentroid());
      expectedCentroid.scale(bigSquare.getArea() / totalArea);

      expectedCentroid.scaleAdd(smallSquare.getArea() / totalArea, smallSquare.getCentroid(), expectedCentroid);

      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(expectedCentroid, PlanarRegionTools.getCentroid2DInLocal(planarRegion), EPSILON);
   }

   @Test
   public void isPointInsidePlanarRegion()
   {
      List<Point2DReadOnly> vertices = new ArrayList<>();
      vertices.add(new Point2D(0.0, 0.0));
      vertices.add(new Point2D(0.0, 1.0));
      vertices.add(new Point2D(1.0, 1.0));
      vertices.add(new Point2D(1.0, 0.0));
      ConvexPolygon2D convexPolygonB = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(vertices));
      RigidBodyTransform transformB = new RigidBodyTransform();
      PlanarRegion regionB = new PlanarRegion(transformB, convexPolygonB);

      vertices.forEach(point -> assertTrue(regionB.isPointInside(point)));
      vertices.forEach(point -> assertTrue(regionB.isPointInWorld2DInside(new Point3D(point))));
   }

   @Test
   public void testComputeMinHeightOfRegionAAboveRegionB()
   {
      double[][] verticesA = new double[][] {{0.0, 0.0}, {0.0, 1.0}, {1.0, 1.0}, {1.0, 0.0}};
      ConvexPolygon2D convexPolygonA = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(verticesA));
      RigidBodyTransform transformA = new RigidBodyTransform();
      double heightAbove = 3.0;
      transformA.getTranslation().set(0.0, 0.0, heightAbove);
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
      transformB.getRotation().setEuler(0.0, rotationAngleB, 0.0);
      regionB = new PlanarRegion(transformB, convexPolygonB);

      minHeightOfRegionAAboveRegionB = PlanarRegionTools.computeMinHeightOfRegionAAboveRegionB(regionA, regionB);
      assertEquals(heightAbove + Math.sin(rotationAngleB) * 1.0, minHeightOfRegionAAboveRegionB, EPSILON);

      // Now rotate the top one, lowering the points and decreasing the height above also.
      transformA = new RigidBodyTransform();
      double rotationAngleA = 0.123;
      transformA.getTranslation().set(0.0, 0.0, heightAbove);
      transformA.getRotation().setEuler(0.0, rotationAngleA, 0.0);
      regionA = new PlanarRegion(transformA, convexPolygonA);

      minHeightOfRegionAAboveRegionB = PlanarRegionTools.computeMinHeightOfRegionAAboveRegionB(regionA, regionB);
      double expectedMinHeight = heightAbove - Math.cos(rotationAngleB) * Math.tan(rotationAngleA) * 1.0 + Math.sin(rotationAngleB) * 1.0;
      assertEquals(expectedMinHeight, minHeightOfRegionAAboveRegionB, EPSILON);

      // Have one on top be vertical:

      transformA = new RigidBodyTransform();
      rotationAngleA = -Math.PI / 2.0;
      transformA.getTranslation().set(0.0, 0.0, heightAbove);
      transformA.getRotation().setEuler(0.0, rotationAngleA, 0.0);
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
         EuclidGeometryPolygonTools.computeConvexPolygon2DArea(convexPolygon2D, hullSize, clockwiseOrdered, centroid);
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
         transform.getTranslation().set(0.0, 0.0, i * layerSeparation);
         PlanarRegion planarRegion = new PlanarRegion(transform, Arrays.asList(concaveHull), polygons);
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
      transformA.getTranslation().set(new Vector3D(0.4, 0.0, 0.0));
      transformB.getTranslation().set(new Vector3D(-0.4, 0.0, 0.1));

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
         List<Point2D> concaveHull = new ArrayList<>();
         for (int i = 0; i < numberOfPoints; i++)
         {
            Point2D pointA = EuclidCoreRandomTools.nextPoint2D(random, maxRegionDimension);
            Point2D pointB = EuclidCoreRandomTools.nextPoint2D(random, maxRegionDimension);
            planarRegionPolygonA.addVertex(pointA);
            planarRegionPolygonB.addVertex(pointB);

            concaveHull.add(pointA);
            concaveHull.add(pointB);
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
            transform.getTranslation().set(translation);
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

            transform.getTranslation().set(translation);

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
      PlanarRegion planarRegion = new PlanarRegion(transform, Arrays.asList(concaveHull), polygons);
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

      region2Transform.getTranslation().set(0.0, 0.0, 1.0);

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
         List<Point2D> concaveHull = new ArrayList<>();
         for (int i = 0; i < numberOfPoints; i++)
         {
            Point2D pointA = EuclidCoreRandomTools.nextPoint2D(random, maxRegionDimension);
            Point2D pointB = EuclidCoreRandomTools.nextPoint2D(random, maxRegionDimension);
            planarRegionPolygonA.addVertex(pointA);
            planarRegionPolygonB.addVertex(pointB);

            concaveHull.add(pointA);
            concaveHull.add(pointB);
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
            transform.getTranslation().set(translation);
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

            transform.getTranslation().set(translation);

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
      PlanarRegion planarRegion = new PlanarRegion(transform, Arrays.asList(concaveHull), polygons);
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
      //TODO: +++JerryPratt: Get this to pass by fixing isRegionAOverlappingWithRegionB()
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
      assertFalse(PlanarRegionTools.isRegionAOverlappingWithRegionB(regionA, regionB, epsilonForCheck));
      assertFalse(PlanarRegionTools.isRegionAOverlappingWithRegionB(regionB, regionA, epsilonForCheck));
      epsilonForCheck = 0.099;
      assertFalse(PlanarRegionTools.isRegionAOverlappingWithRegionB(regionA, regionB, epsilonForCheck));
      assertFalse(PlanarRegionTools.isRegionAOverlappingWithRegionB(regionB, regionA, epsilonForCheck));
      epsilonForCheck = 0.101;
      assertTrue(PlanarRegionTools.isRegionAOverlappingWithRegionB(regionA, regionB, epsilonForCheck));
      assertTrue(PlanarRegionTools.isRegionAOverlappingWithRegionB(regionB, regionA, epsilonForCheck));
      epsilonForCheck = 100.0;
      assertTrue(PlanarRegionTools.isRegionAOverlappingWithRegionB(regionA, regionB, epsilonForCheck));
      assertTrue(PlanarRegionTools.isRegionAOverlappingWithRegionB(regionB, regionA, epsilonForCheck));

      RigidBodyTransform transformC = new RigidBodyTransform();
      transformC.getRotation().setEuler(0.0, Math.PI / 2.0, 0.0);
      PlanarRegion regionC = new PlanarRegion(transformC, polygonA);
      epsilonForCheck = 0.0;
      assertFalse(PlanarRegionTools.isRegionAOverlappingWithRegionB(regionB, regionC, epsilonForCheck));
      assertFalse(PlanarRegionTools.isRegionAOverlappingWithRegionB(regionC, regionB, epsilonForCheck));
      epsilonForCheck = 1.098;
      assertFalse(PlanarRegionTools.isRegionAOverlappingWithRegionB(regionB, regionC, epsilonForCheck));
      assertFalse(PlanarRegionTools.isRegionAOverlappingWithRegionB(regionC, regionB, epsilonForCheck));
      epsilonForCheck = 1.102;
      assertTrue(PlanarRegionTools.isRegionAOverlappingWithRegionB(regionB, regionC, epsilonForCheck));
      assertTrue(PlanarRegionTools.isRegionAOverlappingWithRegionB(regionC, regionB, epsilonForCheck));
      epsilonForCheck = 100.0;
      assertTrue(PlanarRegionTools.isRegionAOverlappingWithRegionB(regionB, regionC, epsilonForCheck));
      assertTrue(PlanarRegionTools.isRegionAOverlappingWithRegionB(regionC, regionB, epsilonForCheck));
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

      region2Transform.getTranslation().set(0.0, 0.0, 1.0);

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
      transformOne.getTranslation().set(0.0, 0.0, 0.99 * epsilon);
      regionA = new PlanarRegion(transformOne, polygonA);
      regionB = new PlanarRegion(new RigidBodyTransform(), polygonB);
      assertFalse(PlanarRegionTools.isPlanarRegionAAbovePlanarRegionB(regionA, regionB, epsilon));
      assertFalse(PlanarRegionTools.isPlanarRegionAAbovePlanarRegionB(regionB, regionA, epsilon));

      transformOne = new RigidBodyTransform();
      transformOne.getTranslation().set(0.0, 0.0, 1.01 * epsilon);
      regionA = new PlanarRegion(transformOne, polygonA);
      regionB = new PlanarRegion(new RigidBodyTransform(), polygonB);
      assertTrue(PlanarRegionTools.isPlanarRegionAAbovePlanarRegionB(regionA, regionB, epsilon));
      assertFalse(PlanarRegionTools.isPlanarRegionAAbovePlanarRegionB(regionB, regionA, epsilon));

      transformOne = new RigidBodyTransform();
      transformOne.getTranslation().set(0.0, 0.0, 0.0);
      transformOne.getRotation().setEuler(0.0, Math.PI / 4.0, 0.0);
      RigidBodyTransform transformTwo = new RigidBodyTransform();
      transformTwo.getTranslation().set(-10.0, 0.0, 0.0);
      regionA = new PlanarRegion(transformOne, polygonA);
      regionB = new PlanarRegion(transformTwo, polygonB);
      assertTrue(PlanarRegionTools.isPlanarRegionAAbovePlanarRegionB(regionA, regionB, epsilon));
      assertFalse(PlanarRegionTools.isPlanarRegionAAbovePlanarRegionB(regionB, regionA, epsilon));

      transformTwo = new RigidBodyTransform();
      transformTwo.getTranslation().set(10.0, 0.0, 0.0);
      regionA = new PlanarRegion(transformOne, polygonA);
      regionB = new PlanarRegion(transformTwo, polygonB);
      assertTrue(PlanarRegionTools.isPlanarRegionAAbovePlanarRegionB(regionA, regionB, epsilon));
      assertTrue(PlanarRegionTools.isPlanarRegionAAbovePlanarRegionB(regionB, regionA, epsilon));

      transformTwo = new RigidBodyTransform();
      transformTwo.getTranslation().set(0.0, 0.0, 0.0);
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
      transformToWorld.getRotation().setYawPitchRoll(1.2, 3.4, 5.6);
      transformToWorld.getTranslation().set(-1.0, 2.2, 3.4);

      PlanarRegion regionA = new PlanarRegion(transformToWorld, polygonA);
      BoundingBox2D boundingBox2D = PlanarRegionTools.getLocalBoundingBox2DInLocal(regionA);
      assertEquals(boundingBox2D, new BoundingBox2D(2.0, 3.0, 4.1, 5.1));
   }

   @Test
   public void testSimpleConcaveHull()
   {
      ConvexPolygon2D bigSquare = new ConvexPolygon2D();
      bigSquare.addVertex(10.0, 10.0);
      bigSquare.addVertex(10.0, 0.0);
      bigSquare.addVertex(-10.0, 0.0);
      bigSquare.addVertex(-10.0, 10.0);
      bigSquare.update();

      ConvexPolygon2D smallSquare = new ConvexPolygon2D();
      smallSquare.addVertex(5.0, -5.0);
      smallSquare.addVertex(5.0, 0.0);
      smallSquare.addVertex(-5.0, 0.0);
      smallSquare.addVertex(-5.0, -5.0);
      smallSquare.update();

      ConvexPolygon2D convexHull = new ConvexPolygon2D();
      bigSquare.getPolygonVerticesView().forEach(convexHull::addVertex);
      smallSquare.getPolygonVerticesView().forEach(convexHull::addVertex);
      convexHull.update();

      List<Point2D> concaveHull = new ArrayList<>();
      concaveHull.add(new Point2D(10.0, 10.0));
      concaveHull.add(new Point2D(10.0, 0.0));
      concaveHull.add(new Point2D(5.0, 0.0));
      concaveHull.add(new Point2D(5.0, -5.0));
      concaveHull.add(new Point2D(-5.0, -5.0));
      concaveHull.add(new Point2D(-5.0, 0.0));
      concaveHull.add(new Point2D(-10.0, 0.0));
      concaveHull.add(new Point2D(-10.0, 10.0));

      List<Point2D> emptyZone1OfConvexHull = new ArrayList<>();
      emptyZone1OfConvexHull.add(new Point2D(10.0, 0.0));
      emptyZone1OfConvexHull.add(new Point2D(5.0, -5.0));
      emptyZone1OfConvexHull.add(new Point2D(5.0, 0.0));

      List<Point2D> emptyZone2OfConvexHull = new ArrayList<>();
      emptyZone2OfConvexHull.add(new Point2D(-10.0, 0.0));
      emptyZone2OfConvexHull.add(new Point2D(-5.0, 0.0));
      emptyZone2OfConvexHull.add(new Point2D(-5.0, -5.0));
      ConvexPolygon2D emptyZone2Hull = new ConvexPolygon2D();
      emptyZone2OfConvexHull.forEach(emptyZone2Hull::addVertex);
      emptyZone2Hull.update();


      // test some easy points
      assertTrue(PlanarRegionTools.isPointInsideConcaveHull(concaveHull, new Point2D(5.0, 0.0)));
      assertTrue(PlanarRegionTools.isPointInsideConcaveHull(concaveHull, new Point2D(-2.5, 0.0)));
      assertTrue(PlanarRegionTools.isPointInsideConcaveHull(concaveHull, new Point2D()));

      // test some edge points, which are the vertices
      concaveHull.forEach(point -> assertTrue(PlanarRegionTools.isPointInsideConcaveHull(concaveHull, point)));
      // check the midpoints of each edge
      assertTrue(PlanarRegionTools.isPointInsideConcaveHull(concaveHull, new Point2D(0.0, 10.0)));
      assertTrue(PlanarRegionTools.isPointInsideConcaveHull(concaveHull, new Point2D(10.0, 5.0)));
      assertTrue(PlanarRegionTools.isPointInsideConcaveHull(concaveHull, new Point2D(7.5, 0.0)));
      assertTrue(PlanarRegionTools.isPointInsideConcaveHull(concaveHull, new Point2D(5.0, -2.5)));
      assertTrue(PlanarRegionTools.isPointInsideConcaveHull(concaveHull, new Point2D(0.0, -5.0)));
      assertTrue(PlanarRegionTools.isPointInsideConcaveHull(concaveHull, new Point2D(-5.0, -2.5)));
      assertTrue(PlanarRegionTools.isPointInsideConcaveHull(concaveHull, new Point2D(-7.5, 0.0)));
      assertTrue(PlanarRegionTools.isPointInsideConcaveHull(concaveHull, new Point2D(-10.0, 5.0)));

      // test a whole bunch of interior points
      Random random = new Random(1738L);
      for (int iter = 0; iter < ITERATIONS; iter++)
      {
         String message = "Iter " + iter;
         Point2DReadOnly pointInBigSquare = getRandomInteriorPoint(random, bigSquare);
         Point2DReadOnly pointInSmallSquare = getRandomInteriorPoint(random, smallSquare);
         Point2DReadOnly pointInEmptyZone1 = getRandomInteriorPoint(random, emptyZone1OfConvexHull);
         Point2DReadOnly pointInEmptyZone2 = getRandomInteriorPoint(random, emptyZone2Hull.getPolygonVerticesView());

         assertTrue(message, PlanarRegionTools.isPointInsidePolygon(bigSquare.getPolygonVerticesView(), pointInBigSquare));
         assertTrue(PlanarRegionTools.isPointInsidePolygon(smallSquare.getPolygonVerticesView(), pointInSmallSquare));
         assertTrue(PlanarRegionTools.isPointInsidePolygon(emptyZone1OfConvexHull, pointInEmptyZone1));
         assertTrue(PlanarRegionTools.isPointInsidePolygon(emptyZone2OfConvexHull, pointInEmptyZone2));

         assertTrue(PlanarRegionTools.isPointInsideConcaveHull(bigSquare.getPolygonVerticesView(), pointInBigSquare));
         assertTrue(PlanarRegionTools.isPointInsideConcaveHull(smallSquare.getPolygonVerticesView(), pointInSmallSquare));
         assertTrue(PlanarRegionTools.isPointInsideConcaveHull(emptyZone1OfConvexHull, pointInEmptyZone1));
         assertTrue(PlanarRegionTools.isPointInsideConcaveHull(emptyZone2OfConvexHull, pointInEmptyZone2));

         assertTrue(PlanarRegionTools.isPointInsideConcaveHull(convexHull.getPolygonVerticesView(), pointInBigSquare));
         assertTrue(PlanarRegionTools.isPointInsideConcaveHull(convexHull.getPolygonVerticesView(), pointInSmallSquare));
         assertTrue(PlanarRegionTools.isPointInsideConcaveHull(convexHull.getPolygonVerticesView(), pointInEmptyZone1));
         assertTrue(PlanarRegionTools.isPointInsideConcaveHull(convexHull.getPolygonVerticesView(), pointInEmptyZone2));

         assertTrue(message, PlanarRegionTools.isPointInsideConcaveHull(concaveHull, pointInBigSquare));
         assertTrue(message, PlanarRegionTools.isPointInsideConcaveHull(concaveHull, pointInSmallSquare));
         assertFalse(message, PlanarRegionTools.isPointInsideConcaveHull(concaveHull, pointInEmptyZone1));
         assertFalse(message, PlanarRegionTools.isPointInsideConcaveHull(concaveHull, pointInEmptyZone2));
      }
   }

   @Test
   public void testRayIntersectsWithEdge()
   {
      // slope an intercept formula,
      double slope = 4.0;
      double intercept = -1.0;
      Point2D firstPointOfSegment = new Point2D(0.5, 1.0);
      Point2D secondPointOfSegment = new Point2D(1.0, 3.0);

      assertEquals(firstPointOfSegment.getY(), firstPointOfSegment.getX() * slope + intercept);
      assertEquals(secondPointOfSegment.getY(), secondPointOfSegment.getX() * slope + intercept);

      Point2D startPoint = new Point2D(0.25, 0.5);
      assertFalse(PlanarRegionTools.rayIntersectsWithEdge(firstPointOfSegment, secondPointOfSegment, startPoint.getX(), startPoint.getY()));

      startPoint = new Point2D(2.0, 1.0);
      assertFalse(PlanarRegionTools.rayIntersectsWithEdge(firstPointOfSegment, secondPointOfSegment, startPoint.getX(), startPoint.getY()));

      startPoint = new Point2D(0.25, 2.0);
      assertTrue(PlanarRegionTools.rayIntersectsWithEdge(firstPointOfSegment, secondPointOfSegment, startPoint.getX(), startPoint.getY()));

      startPoint = new Point2D(2.0, 2.0);
      assertFalse(PlanarRegionTools.rayIntersectsWithEdge(firstPointOfSegment, secondPointOfSegment, startPoint.getX(), startPoint.getY()));

      startPoint = new Point2D(0.25, 3.5);
      assertFalse(PlanarRegionTools.rayIntersectsWithEdge(firstPointOfSegment, secondPointOfSegment, startPoint.getX(), startPoint.getY()));

      startPoint = new Point2D(2.0, 3.5);
      assertFalse(PlanarRegionTools.rayIntersectsWithEdge(firstPointOfSegment, secondPointOfSegment, startPoint.getX(), startPoint.getY()));



   }


   @Test
   public void testIsPointInsideConcaveHull()
   {
      Random random = new Random(1738L);
      for (int iterA = 0; iterA < ITERATIONS; iterA++)
      {
         ConvexPolygon2D randomPolygon = EuclidGeometryRandomTools.nextConvexPolygon2D(random, 10.0, 20);
         double perimeter = getPerimeter(randomPolygon);
         for (int iterB = 0; iterB < ITERATIONS; iterB++)
         {
            Point2D randomPoint = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
            assertEquals(randomPolygon.isPointInside(randomPoint), PlanarRegionTools.isPointInsideConcaveHull(randomPolygon.getPolygonVerticesView(), randomPoint));

            Point2DReadOnly interiorPoint = getRandomInteriorPoint(random, randomPolygon);
            assertTrue(PlanarRegionTools.isPointInsideConcaveHull(randomPolygon.getPolygonVerticesView(), interiorPoint));

            double distanceAlongEdge = ((double) iterB) / ITERATIONS * perimeter;
            Point2DReadOnly pointAlongPerimeter = getPointAlongEdge(distanceAlongEdge, randomPolygon);
            assertTrue(randomPolygon.pointIsOnPerimeter(pointAlongPerimeter));

//            assertTrue(PlanarRegionTools.isPointInsideConcaveHull(randomPolygon.getPolygonVerticesView(), pointAlongPerimeter));

         }
      }
   }

   private static Point2DReadOnly getRandomInteriorPoint(Random random, ConvexPolygon2DReadOnly polygon)
   {
      return getRandomInteriorPoint(random, polygon.getPolygonVerticesView());
   }

   private static Point2DReadOnly getRandomInteriorPoint(Random random, List<? extends Point2DReadOnly> points)
   {
      Point2D point = new Point2D();
      double summedValue = 0.0;
      double maxValue = 1.0;
      for (int i = 0; i < points.size(); i++)
      {
         double ratio = RandomNumbers.nextDouble(random, 0.0, maxValue);
         if (i == points.size() - 1)
            ratio = maxValue;

         point.scaleAdd(ratio, points.get(i), point);
         summedValue += ratio;
         maxValue -= ratio;
      }

      return point;
   }

   private static Point2DReadOnly getPointAlongEdge(double distanceAround, ConvexPolygon2DReadOnly polygon)
   {
      Point2D point = null;
      double traveledDistance = 0;
      for (int i = 0; i < polygon.getNumberOfVertices(); i++)
      {
         double length = polygon.getVertex(i).distance(polygon.getNextVertex(i));
         if (distanceAround - traveledDistance >= length)
            traveledDistance += length;
         else
         {
            double alpha = (distanceAround - traveledDistance) / length;
            point = new Point2D();
            point.interpolate(polygon.getVertex(i), polygon.getNextVertex(i), alpha);
            break;
         }
      }

      return point;
   }

   private static double getPerimeter(ConvexPolygon2DReadOnly polygon2DReadOnly)
   {
      double distance = 0.0;
      for (int i = 0; i < polygon2DReadOnly.getNumberOfVertices(); i++)
      {
         distance += polygon2DReadOnly.getVertex(i).distance(polygon2DReadOnly.getNextVertex(i));
      }

      return distance;
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
