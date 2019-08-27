package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.pawSnapping;


import gnu.trove.list.array.TDoubleArrayList;
import org.junit.jupiter.api.Test;

import us.ihmc.commonWalkingControlModules.polygonWiggling.PolygonWiggler;
import us.ihmc.commonWalkingControlModules.polygonWiggling.WiggleParameters;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DBasics;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.DefaultPawStepPlannerParameters;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.Random;

import static us.ihmc.robotics.Assert.assertTrue;

class PawNodePlanarRegionSnapAndWigglerTest
{
   private static final int iters = 5000;

   @Test
   void testIssue20190603_WigglerViolateDeltaInsideRegionOnly()
   {
      DataSet dataSet = DataSetIOTools.loadDataSet("20190603_224047_PlanarRegionWigglerViolateDeltaInside");
      
      PlanarRegion troublesomeRegion = dataSet.getPlanarRegionsList().getPlanarRegionsAsList().stream().filter(region -> region.isPointInside(new Point3D(-0.12, -0.51, -0.49), 0.01)).findFirst().get();

      ConvexPolygon2D planeToWiggleInto = troublesomeRegion.getConvexHull();

      double deltaInside = 0.06;
      double maxXY = 0.1;

      for (int vertexIndex = 0; vertexIndex < planeToWiggleInto.getNumberOfVertices(); vertexIndex++)
      {
         for (double alpha = 0; alpha <= 1.0; alpha += 0.05)
         {
            ConvexPolygon2D polygonToWiggle = new ConvexPolygon2D();
            Point2D pointOnEdge = new Point2D();
            pointOnEdge.interpolate(planeToWiggleInto.getVertex(vertexIndex), planeToWiggleInto.getNextVertex(vertexIndex), alpha);
            LineSegment2DBasics edge = new LineSegment2D();
            planeToWiggleInto.getEdge(vertexIndex, edge);
            Vector2DBasics normal = edge.direction(true);
            normal = EuclidGeometryTools.perpendicularVector2D(normal);
            normal.scale(5.0e-3);
            polygonToWiggle.addVertex(pointOnEdge);
            polygonToWiggle.update();
            WiggleParameters parameters = new WiggleParameters();
            parameters.deltaInside = deltaInside;
            parameters.maxX = maxXY;
            parameters.maxY = maxXY;
            parameters.minX = -maxXY;
            parameters.minY = -maxXY;
            parameters.maxYaw = 0.0;
            parameters.minYaw = -0.0;
            
            ConvexPolygon2D wigglePolygon = PolygonWiggler.wigglePolygon(polygonToWiggle, planeToWiggleInto, parameters);

            double distanceInside = planeToWiggleInto.signedDistance(wigglePolygon.getCentroid());
            assertTrue("desired = " + -0.025 + " actual = " + distanceInside, distanceInside < -0.025);
//            assertEquals(-parameters.deltaInside, planeToWiggleInto.signedDistance(wigglePolygon.getCentroid()), 1.0e-3);
         }
      }
   }

   @Test
   void testIssue20190603_WigglerViolateDeltaInsideSinglePoint()
   {
      DataSet dataSet = DataSetIOTools.loadDataSet("20190603_224047_PlanarRegionWigglerViolateDeltaInside");

      PlanarRegion troublesomeRegion = dataSet.getPlanarRegionsList().getPlanarRegionsAsList().stream().filter(region -> region.isPointInside(new Point3D(-0.12, -0.51, -0.49), 0.01)).findFirst().get();

      ConvexPolygon2D planeToWiggleInto = troublesomeRegion.getConvexHull();


            ConvexPolygon2D polygonToWiggle = new ConvexPolygon2D();
            Point2D pointOnEdge = new Point2D(0.1621632681, -0.006395009);
            polygonToWiggle.addVertex(pointOnEdge);
            polygonToWiggle.update();
            WiggleParameters parameters = new WiggleParameters();
            parameters.deltaInside = 0.06;
            parameters.maxX = 0.05;
            parameters.maxY = 0.05;
            parameters.minX = -0.05;
            parameters.minY = -0.05;
            parameters.maxYaw = 0.0;
            parameters.minYaw = -0.0;

            ConvexPolygon2D wigglePolygon = PolygonWiggler.wigglePolygon(polygonToWiggle, planeToWiggleInto, parameters);

            assertTrue(planeToWiggleInto.signedDistance(wigglePolygon.getCentroid()) < -0.025);
            //            assertEquals(-parameters.deltaInside, planeToWiggleInto.signedDistance(wigglePolygon.getCentroid()), 1.0e-3);
   }

   @Test
   void testIssue20190603_WigglerViolateDeltaInside()
   {
      DataSet dataSet = DataSetIOTools.loadDataSet("20190603_224047_PlanarRegionWigglerViolateDeltaInside");
      Point3D suspiciousPoint = new Point3D(-0.12, -0.51, -0.49);
      Random random = new Random(1738L);
      testPoint(random, suspiciousPoint, dataSet, iters);
//      testABunchOfPoints(dataSet, 5000);
   }


   private static void testABunchOfPoints(DataSet dataSet, int numberOfPoints)
   {
      PlanarRegionsList planarRegionsList = dataSet.getPlanarRegionsList();

      Random random = new Random(1738L);

      int pointsTested = 0;
      while (pointsTested < numberOfPoints)
      {
         int regionNumberToCheck = RandomNumbers.nextInt(random, 0, planarRegionsList.getNumberOfPlanarRegions() - 1);
         PlanarRegion planarRegion = planarRegionsList.getPlanarRegion(regionNumberToCheck);
         if (planarRegion.getNormal().angle(new Vector3D(0.0, 0.0, 1.0)) < Math.toRadians(45.0))
            continue; // don't check these guys

         int convexPolygonToCheck = RandomNumbers.nextInt(random, 0, planarRegion.getNumberOfConvexPolygons() - 1);
         Point2DReadOnly randomPointInPolygon = getRandomInteriorPointOfPolygon(random, planarRegion.getConvexPolygon(convexPolygonToCheck));
         Point3D randomPointInWorld = new Point3D(randomPointInPolygon);
         planarRegion.transformFromLocalToWorld(randomPointInWorld);

         testPoint(random, randomPointInWorld, dataSet, 1);

         pointsTested++;
      }
   }

   private static void testPoint(Random random, Point3DReadOnly containingPointOfSuspicion, DataSet dataSet, int iters)
   {
      boolean useConvexHull = true;
      double distanceInside = 0.06;
      boolean careAboutGridCell = false;
      double maxWiggle = 0.05;

      DefaultPawStepPlannerParameters plannerParameters = new DefaultPawStepPlannerParameters()
      {
         @Override
         public double getMaximumXYWiggleDistance()
         {
            return maxWiggle;
         }
      };
      PawNodePlanarRegionSnapAndWiggler snapAndWiggler = new PawNodePlanarRegionSnapAndWiggler(plannerParameters, () -> distanceInside, () -> useConvexHull,
                                                                                               careAboutGridCell);

      PlanarRegion troublesomeRegion = dataSet.getPlanarRegionsList().getPlanarRegionsAsList().stream().filter(region -> region.isPointInside(containingPointOfSuspicion, 0.01)).findFirst().get();
      snapAndWiggler.setPlanarRegions(dataSet.getPlanarRegionsList());
      ConvexPolygon2D planeToWiggleInto = troublesomeRegion.getConvexHull();


      for (int vertexIndex = 0; vertexIndex < planeToWiggleInto.getNumberOfVertices(); vertexIndex++)
      {
         for (double alpha = 0.0; alpha <= 1.0; alpha += 0.05)
         {
            Point2D pointOnEdgeInLocal = new Point2D();
            pointOnEdgeInLocal.interpolate(planeToWiggleInto.getVertex(vertexIndex), planeToWiggleInto.getNextVertex(vertexIndex), alpha);
//            LineSegment2DBasics edge = new LineSegment2D();
//            planeToWiggleInto.getEdge(vertexIndex, edge);
//            Vector2DBasics normal = edge.direction(true);
//            normal = EuclidGeometryTools.perpendicularVector2D(normal);
//            normal.scale(5.0e-3);

            Point3D pointOnEdgeInWorld = new Point3D(pointOnEdgeInLocal);
            troublesomeRegion.transformFromLocalToWorld(pointOnEdgeInWorld);

            String premessage = "Vertex index = " + vertexIndex + "\nAlpha = " + alpha + "\n";

            assertTrue(premessage + "Pre-wiggle is out of the region.\n Point in local\n" + pointOnEdgeInLocal, troublesomeRegion.isPointInside(pointOnEdgeInLocal, 1e-8));
            assertTrue(premessage + "Pre-wiggle is out of the region with the world check.\n Point in world\n" + pointOnEdgeInWorld, troublesomeRegion.isPointInWorld2DInside(pointOnEdgeInWorld, 1e-8));


            int xIndex = PawNode.snapToGrid(pointOnEdgeInWorld.getX());
            int yIndex = PawNode.snapToGrid(pointOnEdgeInWorld.getY());
            double gridX = PawNode.gridSizeXY * xIndex;
            double gridY = PawNode.gridSizeXY * yIndex;

            Point3D gridPointInLocal = new Point3D(gridX, gridY, 0.0);
            troublesomeRegion.transformFromWorldToLocal(gridPointInLocal);

            // filters out snapping points that aren't inside via da grid.
            if (!troublesomeRegion.isPointInWorld2DInside(new Point3D(gridX, gridY, 0.0), 1e-8))
               continue;

            PawNodeSnapData snapData = snapAndWiggler.snapPawNode(xIndex, yIndex);

            Point3D snappedPointInWorld = new Point3D(gridX, gridY, 0.0);
            snapData.getSnapTransform().transform(snappedPointInWorld);

            Point3DReadOnly alternatelySnappedPointInWorld = runDifferentCheck(new Point2D(gridPointInLocal), troublesomeRegion, distanceInside, maxWiggle);
            EuclidCoreTestTools.assertPoint3DGeometricallyEquals(premessage + "Checking the way it was wiggled\n", alternatelySnappedPointInWorld, snappedPointInWorld, 1e-5);

            Point3D snappedPointInLocal = new Point3D(snappedPointInWorld);
            Point3D alternatelySnappedPointInLocal = new Point3D(alternatelySnappedPointInWorld);
            troublesomeRegion.transformFromWorldToLocal(snappedPointInLocal);
            troublesomeRegion.transformFromWorldToLocal(alternatelySnappedPointInLocal);

            Point2D pointToCheck = new Point2D(snappedPointInLocal);
            Point2D alternatelyPointToCheck = new Point2D(alternatelySnappedPointInLocal);


            assertTrue(premessage + "We wiggled out of the region with the world check.\n Point in world\n" + alternatelySnappedPointInWorld, troublesomeRegion.isPointInWorld2DInside(alternatelySnappedPointInWorld));
            assertTrue(premessage + "We wiggled out of the region with the world check.\n Point in world\n" + snappedPointInWorld, troublesomeRegion.isPointInWorld2DInside(snappedPointInWorld));
            assertTrue(premessage + "We wiggled out of the region!", troublesomeRegion.isPointInside(alternatelyPointToCheck));
            assertTrue(premessage + "We wiggled out of the region!", troublesomeRegion.isPointInside(pointToCheck));
            double achievedDistance = planeToWiggleInto.signedDistance(pointToCheck);
            double alternateltyAchievedDistance = planeToWiggleInto.signedDistance(alternatelyPointToCheck);
            assertTrue(premessage + "Point\n" + alternatelyPointToCheck + "\nis not far enough inside region\n" + planeToWiggleInto + "\ndistance inside: " + alternateltyAchievedDistance,
                       alternateltyAchievedDistance < -0.025);
            assertTrue(premessage + "Point\n" + pointToCheck + "\nis not far enough inside region\n" + planeToWiggleInto + "\ndistance inside: " + achievedDistance,
                       achievedDistance < -0.025);
         }
      }

      for (int iter = 0; iter < iters; iter++)
      {
         Point2DReadOnly pointInLocalToWiggle = getRandomInteriorPointOfPolygon(random, planeToWiggleInto);

         Point3D pointInWorldToWiggle = new Point3D(pointInLocalToWiggle);
         troublesomeRegion.transformFromLocalToWorld(pointInWorldToWiggle);
         Point3D pointInLocalAgain = new Point3D(pointInWorldToWiggle);
         troublesomeRegion.transformFromWorldToLocal(pointInLocalAgain);

         String premessage = "Iter = " + iter + "\n";

         assertTrue(premessage + "Pre-wiggle is out of the region.\n Point in local\n" + pointInLocalToWiggle, troublesomeRegion.isPointInside(pointInLocalToWiggle, 1e-8));
         assertTrue(premessage + "Pre-wiggle is out of the region with the world check.\n Point in world\n" + pointInWorldToWiggle, troublesomeRegion.isPointInWorld2DInside(pointInWorldToWiggle, 1e-8));


         int xIndex = PawNode.snapToGrid(pointInWorldToWiggle.getX());
         int yIndex = PawNode.snapToGrid(pointInWorldToWiggle.getY());
         double gridX = PawNode.gridSizeXY * xIndex;
         double gridY = PawNode.gridSizeXY * yIndex;

         Point3D gridPointInLocal = new Point3D(gridX, gridY, 0.0);
         troublesomeRegion.transformFromWorldToLocal(gridPointInLocal);

         // filters out snapping points that aren't inside via da grid.
         if (!troublesomeRegion.isPointInWorld2DInside(new Point3D(gridX, gridY, 0.0), 1e-8))
            continue;

         PawNodeSnapData snapData = snapAndWiggler.snapPawNode(xIndex, yIndex);

         Point3D snappedPointInWorld = new Point3D(gridX, gridY, 0.0);
         snapData.getSnapTransform().transform(snappedPointInWorld);

         Point3D snappedPointInLocal = new Point3D(snappedPointInWorld);
         troublesomeRegion.transformFromWorldToLocal(snappedPointInLocal);

         Point2D pointToCheck = new Point2D(snappedPointInLocal);

         assertTrue(premessage + "We wiggled out of the region with the world check.\n Point in world\n" + snappedPointInWorld, troublesomeRegion.isPointInWorld2DInside(snappedPointInWorld));
         assertTrue(premessage + "We wiggled out of the region!", troublesomeRegion.isPointInside(pointToCheck));
         double achievedDistance = planeToWiggleInto.signedDistance(pointToCheck);
         assertTrue(premessage + "Point\n" + pointToCheck + "\nis not far enough inside region\n" + planeToWiggleInto + "\ndistance inside: " + achievedDistance,
                    achievedDistance < -0.025);
      }
   }

   private static Point3DReadOnly runDifferentCheck(Point2D pointInLocalToWiggle, PlanarRegion regionToWiggleInto, double distanceInside, double maxWiggle)
   {
      ConvexPolygon2D polygonToWiggle = new ConvexPolygon2D();
      polygonToWiggle.addVertex(pointInLocalToWiggle);
      polygonToWiggle.update();
      WiggleParameters parameters = new WiggleParameters();
      parameters.deltaInside = distanceInside;
      parameters.maxX = maxWiggle;
      parameters.maxY = maxWiggle;
      parameters.minX = -maxWiggle;
      parameters.minY = -maxWiggle;
      parameters.maxYaw = 0.0;
      parameters.minYaw = -0.0;

      ConvexPolygon2D wigglePolygon = PolygonWiggler.wigglePolygon(polygonToWiggle, regionToWiggleInto.getConvexHull(), parameters);
      Point3D centroidInWorld = new Point3D(wigglePolygon.getCentroid());
      regionToWiggleInto.transformFromLocalToWorld(centroidInWorld);

      return centroidInWorld;
   }

   private static Point2DReadOnly getRandomInteriorPointOfPolygon(Random randomSeed, ConvexPolygon2DReadOnly polygon)
   {
      TDoubleArrayList vertexWeights = new TDoubleArrayList();
      for (int i = 0; i < polygon.getNumberOfVertices(); i++)
      {
         vertexWeights.add(RandomNumbers.nextDouble(randomSeed, 0.0, 1.0));
      }

      double sum = vertexWeights.sum();
      for (int i = 0; i < vertexWeights.size(); i++)
      {
         vertexWeights.set(i, vertexWeights.get(i) / sum);
      }

      Point2D interiorPoint = new Point2D();
      for (int i = 0; i < polygon.getNumberOfVertices(); i++)
      {
         interiorPoint.scaleAdd(vertexWeights.get(i), polygon.getVertex(i), interiorPoint);
      }

      return interiorPoint;
   }
}
