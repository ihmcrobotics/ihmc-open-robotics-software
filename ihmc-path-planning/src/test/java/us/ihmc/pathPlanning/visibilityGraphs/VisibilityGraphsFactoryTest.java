package us.ihmc.pathPlanning.visibilityGraphs;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.Connection;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.NavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityMap;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.NavigableExtrusionDistanceCalculator;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.ObstacleExtrusionDistanceCalculator;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.ObstacleRegionFilter;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.PlanarRegionFilter;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphsParameters;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegion;

public class VisibilityGraphsFactoryTest
{

   private static final double EPSILON = 1e-6;

   @Test(timeout = 30000)
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   public void testCreateNavigableRegion()
   {
      RigidBodyTransform transformToWorld = new RigidBodyTransform();

      Point2D pointA = new Point2D(0.99, 0.99);
      Point2D pointB = new Point2D(0.99, 2.01);
      Point2D pointC = new Point2D(2.01, 2.01);
      Point2D pointD = new Point2D(2.01, 0.99);

      PlanarRegion planarRegion = createPlanarRegion(transformToWorld, pointA, pointB, pointC, pointD);
      List<PlanarRegion> otherRegions = new ArrayList<>();

      double orthogonalAngle = 1.0;
      double clusterResolution = 0.501;

      ObstacleRegionFilter obstacleRegionFilter = createNoObstaclesFilter();
      PlanarRegionFilter filter = createAllRegionsValidFilter();
      NavigableExtrusionDistanceCalculator navigableCalculator = createConstantNavigableExtrusionDistanceCalculator(0.01);
      ObstacleExtrusionDistanceCalculator obstacleCalculator = createConstantObstacleExtrusionDistanceCalculator(0.02);

      NavigableRegion navigableRegion = VisibilityGraphsFactory.createNavigableRegion(planarRegion, otherRegions, orthogonalAngle, clusterResolution,
                                                                                      obstacleRegionFilter, filter, navigableCalculator, obstacleCalculator);

      VisibilityMap visibilityMapInLocal = navigableRegion.getVisibilityMapInLocal();
      Set<Connection> connections = visibilityMapInLocal.getConnections();

      assertEquals(8 * 7 / 2, connections.size());

      Point2D pointAIn = new Point2D(1.0, 1.0);
      Point2D pointBIn = new Point2D(1.0, 2.0);
      Point2D pointCIn = new Point2D(2.0, 2.0);
      Point2D pointDIn = new Point2D(2.0, 1.0);

      Point2D pointEIn = new Point2D(1.0, 1.5);
      Point2D pointFIn = new Point2D(1.5, 2.0);
      Point2D pointGIn = new Point2D(2.0, 1.5);
      Point2D pointHIn = new Point2D(1.5, 1.0);

      assertTrue(containsAllConnectionPairs(connections, pointAIn, pointBIn, pointCIn, pointDIn, pointEIn, pointFIn, pointGIn, pointHIn));
   }

   private ObstacleExtrusionDistanceCalculator createConstantObstacleExtrusionDistanceCalculator(double extrusionDistance)
   {
      ObstacleExtrusionDistanceCalculator obstacleCalculator = new ObstacleExtrusionDistanceCalculator()
      {
         @Override
         public double computeExtrusionDistance(Point2DReadOnly pointToExtrude, double obstacleHeight)
         {
            return extrusionDistance;
         }
      };
      return obstacleCalculator;
   }

   private NavigableExtrusionDistanceCalculator createConstantNavigableExtrusionDistanceCalculator(double navigableDistance)
   {
      NavigableExtrusionDistanceCalculator navigableCalculator = new NavigableExtrusionDistanceCalculator()
      {
         @Override
         public double computeExtrusionDistance(PlanarRegion navigableRegionToBeExtruded)
         {
            return navigableDistance;
         }
      };
      return navigableCalculator;
   }

   private PlanarRegionFilter createAllRegionsValidFilter()
   {
      PlanarRegionFilter filter = new PlanarRegionFilter()
      {
         @Override
         public boolean isPlanarRegionRelevant(PlanarRegion planarRegion)
         {
            return true;
         }
      };
      return filter;
   }

   private ObstacleRegionFilter createNoObstaclesFilter()
   {
      ObstacleRegionFilter obstacleRegionFilter = new ObstacleRegionFilter()
      {
         @Override
         public boolean isRegionValidObstacle(PlanarRegion query, PlanarRegion navigableRegion)
         {
            return false;
         }
      };
      return obstacleRegionFilter;
   }

   @Test(timeout = 30000)
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   public void testCreateNavigableRegions()
   {
      VisibilityGraphsParameters parameters = createVisibilityGraphParametersForTests();
      List<PlanarRegion> allRegions = new ArrayList<>();

      RigidBodyTransform transformToWorld = new RigidBodyTransform();

      Point2D pointA = new Point2D(-0.01, -0.01);
      Point2D pointB = new Point2D(-0.01, 1.01);
      Point2D pointC = new Point2D(1.01, 1.01);
      Point2D pointD = new Point2D(1.01, -0.01);

      PlanarRegion planarRegionOne = createPlanarRegion(transformToWorld, pointA, pointB, pointC, pointD);
      allRegions.add(planarRegionOne);

      Point2D pointE = new Point2D(1.49, -0.01);
      Point2D pointF = new Point2D(1.49, 1.01);
      Point2D pointG = new Point2D(2.51, 1.01);
      Point2D pointH = new Point2D(2.51, -0.01);

      PlanarRegion planarRegionTwo = createPlanarRegion(transformToWorld, pointE, pointF, pointG, pointH);
      allRegions.add(planarRegionTwo);

      allRegions = PlanarRegionTools.ensureClockwiseOrder(allRegions);

      List<NavigableRegion> navigableRegions = VisibilityGraphsFactory.createNavigableRegions(allRegions, parameters);

      assertEquals(2, navigableRegions.size());

      NavigableRegion navigableRegionOne = navigableRegions.get(0);
      NavigableRegion navigableRegionTwo = navigableRegions.get(1);

      VisibilityMap visibilityMapOne = navigableRegionOne.getVisibilityMapInWorld();
      VisibilityMap visibilityMapTwo = navigableRegionTwo.getVisibilityMapInWorld();

      Set<Connection> connectionsOne = visibilityMapOne.getConnections();
      Set<Connection> connectionsTwo = visibilityMapTwo.getConnections();

      printConnections(connectionsOne);
   }

   private VisibilityGraphsParameters createVisibilityGraphParametersForTests()
   {
      //      return new DefaultVisibilityGraphParameters();
      VisibilityGraphsParameters parameters = new VisibilityGraphsParameters()
      {

         @Override
         public double getMaxInterRegionConnectionLength()
         {
            return 0.55;
         }

         @Override
         public double getNormalZThresholdForAccessibleRegions()
         {
            return 0.75;
         }

         @Override
         public double getExtrusionDistance()
         {
            return 0.4;
         }

         @Override
         public double getExtrusionDistanceIfNotTooHighToStep()
         {
            return 0.05;
         }

         @Override
         public double getTooHighToStepDistance()
         {
            return 0.4;
         }

         @Override
         public double getClusterResolution()
         {
            return 0.51; //0.2;
         }

         @Override
         public double getPlanarRegionMinArea()
         {
            return 0.05;
         }

         @Override
         public int getPlanarRegionMinSize()
         {
            return 2;
         }

         @Override
         public NavigableExtrusionDistanceCalculator getNavigableExtrusionDistanceCalculator()
         {
            return new NavigableExtrusionDistanceCalculator()
            {
               @Override
               public double computeExtrusionDistance(PlanarRegion navigableRegionToBeExtruded)
               {
                  return 0.01; //0.02;
               }
            };
         }
      };

      return parameters;
   }

   private void printConnections(Set<Connection> connections)
   {
      System.out.println("Number of Connections: " + connections.size());
      System.out.println(connections);
   }

   private boolean containsAllConnectionPairs(Set<Connection> connections, Point2D... points)
   {
      if (connections.size() != (points.length * (points.length - 1)) / 2)
         return false;

      for (Point2D pointA : points)
      {
         for (Point2D pointB : points)
         {
            if (pointA == pointB)
               continue;

            if (!doesConnectionsContain(connections, pointA, pointB))
               return false;
         }
      }
      return true;
   }

   private boolean doesConnectionsContain(Set<Connection> connections, Point2D pointA, Point2D pointB)
   {
      for (Connection connection : connections)
      {
         if (connection.getSourcePoint2D().epsilonEquals(pointA, EPSILON) && connection.getTargetPoint2D().epsilonEquals(pointB, EPSILON))
            return true;

         if (connection.getSourcePoint2D().epsilonEquals(pointB, EPSILON) && connection.getTargetPoint2D().epsilonEquals(pointA, EPSILON))
            return true;
      }

      return false;

   }

   private PlanarRegion createPlanarRegion(RigidBodyTransform transformToWorld, Point2D... points)
   {
      Vertex2DSupplier vertex2DSupplier = Vertex2DSupplier.asVertex2DSupplier(points);
      ConvexPolygon2D polygon = new ConvexPolygon2D(vertex2DSupplier);

      PlanarRegion planarRegion = new PlanarRegion(transformToWorld, polygon);

      planarRegion.update();
      return planarRegion;
   }

}
