package us.ihmc.pathPlanning.visibilityGraphs;

import static us.ihmc.pathPlanning.visibilityGraphs.tools.VisibilityTools.isPointVisibleForStaticMaps;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.Connection;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.ConnectionPoint3D;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.InterRegionVisibilityMap;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.NavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.SingleSourceVisibilityMap;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphsParameters;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityMapHolder;
import us.ihmc.pathPlanning.visibilityGraphs.tools.ClusterTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.OcclusionTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegion;

public class NavigableRegionsManager
{
   final static boolean debug = false;
   final static boolean useCustomDijkstraSearch = true;

   private NavigableRegions navigableRegions;

   final static int START_GOAL_ID = 0;

   private SingleSourceVisibilityMap startMap, goalMap;

   private final VisibilityGraphsParameters parameters;

   private InterRegionVisibilityMap interRegionVisibilityMap;

   public NavigableRegionsManager()
   {
      this(null, null);
   }

   public NavigableRegionsManager(VisibilityGraphsParameters parameters)
   {
      this(parameters, null);
   }

   public NavigableRegionsManager(List<PlanarRegion> regions)
   {
      this(null, regions);
   }

   public NavigableRegionsManager(VisibilityGraphsParameters parameters, List<PlanarRegion> regions)
   {
      this.navigableRegions = new NavigableRegions(parameters, regions);

      this.parameters = parameters == null ? new DefaultVisibilityGraphParameters() : parameters;
   }

   public List<NavigableRegion> getNavigableRegionsList()
   {
      return navigableRegions.getNaviableRegionsList();
   }

   public void setPlanarRegions(List<PlanarRegion> planarRegions)
   {
      navigableRegions.setPlanarRegions(planarRegions);
   }

   public List<Point3DReadOnly> calculateBodyPath(final Point3DReadOnly start, final Point3DReadOnly goal)
   {
      if (start == null)
      {
         PrintTools.error("Start is null!");
         return null;
      }

      if (goal == null)
      {
         PrintTools.error("Goal is null!");
         return null;
      }

      if (debug)
         PrintTools.info("Starting to calculate body path");

      navigableRegions.filterPlanarRegionsWithBoundingCapsule(start, goal, parameters.getExplorationDistanceFromStartGoal());

      long startBodyPathComputation = System.currentTimeMillis();

      //FIXME: +++JEP 181203. We have a bug where the the path can cross over a hole when there is an object on top of the hole. Need to fix that.

      //TODO: Do this stuff lazily, rather than all up front for efficiency.
      navigableRegions.createNavigableRegions();

      //Note: This has to be done before inter regions if the inter regions are computed using inner regions.
      // Otherwise, the ordering does not matter.
      //      VisibilityGraphsFactory.createStaticVisibilityMapsForNavigableRegions(navigableRegions);

      List<NavigableRegion> navigableRegionsList = navigableRegions.getNaviableRegionsList();
      interRegionVisibilityMap = VisibilityGraphsFactory.createInterRegionVisibilityMap(navigableRegionsList, parameters.getInterRegionConnectionFilter());
      VisibilityGraphsFactory.createStaticVisibilityMapsForNavigableRegions(navigableRegionsList);

      double searchHostEpsilon = parameters.getSearchHostRegionEpsilon();
      startMap = VisibilityGraphsFactory.createSingleSourceVisibilityMap(start, navigableRegionsList, searchHostEpsilon,
                                                                         interRegionVisibilityMap.getVisibilityMapInLocal());
      goalMap = VisibilityGraphsFactory.createSingleSourceVisibilityMap(goal, navigableRegionsList, searchHostEpsilon,
                                                                        interRegionVisibilityMap.getVisibilityMapInLocal());

      if (goalMap == null)
      {
         goalMap = VisibilityGraphsFactory.connectToClosestPoints(new ConnectionPoint3D(goal, START_GOAL_ID), 1, navigableRegionsList, START_GOAL_ID);
      }

      if (startMap != null)
      {
         if (startMap.getHostRegion() == goalMap.getHostRegion())
         {
            if (isPointVisibleForStaticMaps(startMap.getHostRegion().getAllClusters(), startMap.getSourceInLocal2D(), goalMap.getSourceInLocal2D()))
            {
               startMap.addConnectionInWorld(new Connection(start, startMap.getMapId(), goal, goalMap.getMapId()));
            }
         }
      }
      else
      {
         startMap = VisibilityGraphsFactory.connectToFallbackMap(start, START_GOAL_ID, 1.0e-3, interRegionVisibilityMap.getVisibilityMapInLocal());

         if (startMap == null)
            startMap = VisibilityGraphsFactory.connectToClosestPoints(new ConnectionPoint3D(start, START_GOAL_ID), 1, navigableRegionsList, START_GOAL_ID);
      }

      if (startMap == null)
         return null;

      List<VisibilityMapHolder> visibilityMapHolders = new ArrayList<>();
      visibilityMapHolders.addAll(navigableRegionsList);
      visibilityMapHolders.add(startMap);
      visibilityMapHolders.add(goalMap);
      visibilityMapHolders.add(interRegionVisibilityMap);

      ConnectionPoint3D startConnection = new ConnectionPoint3D(start, START_GOAL_ID);
      ConnectionPoint3D goalConnection = new ConnectionPoint3D(goal, START_GOAL_ID);

      List<Point3DReadOnly> path = parameters.getPathPlanner().calculatePath(startConnection, goalConnection, visibilityMapHolders);

      if (debug)
      {
         if (path != null)
         {
            PrintTools.info("Total time to find solution was: " + (System.currentTimeMillis() - startBodyPathComputation) + "ms");
         }
         else
         {
            PrintTools.info("NO BODY PATH SOLUTION WAS FOUND!" + (System.currentTimeMillis() - startBodyPathComputation) + "ms");
         }
      }

      return path;
   }

   public List<Point3DReadOnly> calculateBodyPathWithOcclusions(Point3DReadOnly start, Point3DReadOnly goal)
   {
      List<Point3DReadOnly> path = calculateBodyPath(start, goal);

      if (path == null)
      {
         List<NavigableRegion> navigableRegionsList = navigableRegions.getNavigableRegions();

         if (!OcclusionTools.isTheGoalIntersectingAnyObstacles(navigableRegionsList.get(0), start, goal))
         {
            if (debug)
            {
               PrintTools.info("StraightLine available");
            }

            path = new ArrayList<>();
            path.add(start);
            path.add(goal);

            return path;
         }

         NavigableRegion regionContainingPoint = PlanarRegionTools.getNavigableRegionContainingThisPoint(start, navigableRegionsList);
         List<Cluster> intersectingClusters = OcclusionTools.getListOfIntersectingObstacles(regionContainingPoint.getObstacleClusters(), start, goal);
         Cluster closestCluster = ClusterTools.getTheClosestCluster(start, intersectingClusters);
         Point3D closestExtrusion = ClusterTools.getTheClosestVisibleExtrusionPoint(1.0, start, goal, closestCluster.getNavigableExtrusionsInWorld(),
                                                                                    regionContainingPoint.getHomePlanarRegion());

         path = calculateBodyPath(start, closestExtrusion);
         path.add(goal);

         return path;
      }
      else
      {
         return path;
      }
   }

   public VisibilityMapHolder getStartMap()
   {
      return startMap;
   }

   public VisibilityMapHolder getGoalMap()
   {
      return goalMap;
   }

   public InterRegionVisibilityMap getInterRegionConnections()
   {
      return interRegionVisibilityMap;
   }

}
