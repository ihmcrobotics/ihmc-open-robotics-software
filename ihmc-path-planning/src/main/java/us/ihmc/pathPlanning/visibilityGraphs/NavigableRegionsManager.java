package us.ihmc.pathPlanning.visibilityGraphs;

import static us.ihmc.pathPlanning.visibilityGraphs.tools.VisibilityTools.isPointVisibleForStaticMaps;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

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
import us.ihmc.pathPlanning.visibilityGraphs.tools.JGraphTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.OcclusionTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegion;

public class NavigableRegionsManager
{
   final static boolean debug = false;

   final static int START_GOAL_ID = 0;

   private List<PlanarRegion> regions;
   private SingleSourceVisibilityMap startMap, goalMap;
   private List<NavigableRegion> navigableRegions;

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
      this.parameters = parameters == null ? new DefaultVisibilityGraphParameters() : parameters;
      setPlanarRegions(regions);
   }

   public void setPlanarRegions(List<PlanarRegion> regions)
   {
      if (regions != null)
      {
         regions = PlanarRegionTools.ensureClockwiseOrder(regions);
         regions = regions.stream().filter(parameters.getPlanarRegionFilter()::isPlanarRegionRelevant).collect(Collectors.toList());
      }

      this.regions = regions;
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

      regions = PlanarRegionTools.filterPlanarRegionsWithBoundingCapsule(start, goal, parameters.getExplorationDistanceFromStartGoal(), regions);

      long startBodyPathComputation = System.currentTimeMillis();

      navigableRegions = VisibilityGraphsFactory.createNavigableRegions(regions, parameters);
      interRegionVisibilityMap = VisibilityGraphsFactory.createInterRegionVisibilityMap(navigableRegions, parameters.getInterRegionConnectionFilter());
      double searchHostEpsilon = parameters.getSearchHostRegionEpsilon();
      startMap = VisibilityGraphsFactory.createSingleSourceVisibilityMap(start, navigableRegions, searchHostEpsilon,
                                                                         interRegionVisibilityMap.getVisibilityMapInLocal());
      goalMap = VisibilityGraphsFactory.createSingleSourceVisibilityMap(goal, navigableRegions, searchHostEpsilon,
                                                                        interRegionVisibilityMap.getVisibilityMapInLocal());

      if (goalMap == null)
         return null;

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
      }

      if (startMap == null)
         return null;

      List<VisibilityMapHolder> visibilityMapHolders = new ArrayList<>();
      visibilityMapHolders.addAll(navigableRegions);
      visibilityMapHolders.add(startMap);
      visibilityMapHolders.add(goalMap);
      visibilityMapHolders.add(interRegionVisibilityMap);

      ConnectionPoint3D startConnection = new ConnectionPoint3D(start, START_GOAL_ID);
      ConnectionPoint3D goalConnection = new ConnectionPoint3D(goal, START_GOAL_ID);
      List<Point3DReadOnly> path = JGraphTools.calculatePathOnVisibilityGraph(startConnection, goalConnection, visibilityMapHolders);

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
         if (!OcclusionTools.isTheGoalIntersectingAnyObstacles(navigableRegions.get(0), start, goal))
         {
            if(debug)
            {
               PrintTools.info("StraightLine available");
            }

            path = new ArrayList<>();
            path.add(start);
            path.add(goal);

            return path;
         }

         NavigableRegion regionContainingPoint = PlanarRegionTools.getNavigableRegionContainingThisPoint(start, navigableRegions);
         List<Cluster> intersectingClusters = OcclusionTools.getListOfIntersectingObstacles(regionContainingPoint.getObstacleClusters(), start, goal);
         Cluster closestCluster = ClusterTools.getTheClosestCluster(start, intersectingClusters);
         Point3D closestExtrusion = ClusterTools.getTheClosestVisibleExtrusionPoint(1.0, start, goal, closestCluster.getNavigableExtrusionsInWorld3D(),
                                                                                    regionContainingPoint.getHomeRegion());

         path = calculateBodyPath(start, closestExtrusion);
         path.add(goal);

         return path;
      }
      else
      {
         return path;
      }
   }

   public Point3D[][] getNavigableExtrusions()
   {
      Point3D[][] allNavigableExtrusions = new Point3D[navigableRegions.size()][];

      for (int i = 0; i < navigableRegions.size(); i++)
      {
         NavigableRegion localPlanner = navigableRegions.get(i);
         Point3D[] navigableExtrusions = new Point3D[localPlanner.getAllClusters().size()];

         for (Cluster cluster : localPlanner.getAllClusters())
         {
            for (int j = 0; j < cluster.getNumberOfNavigableExtrusions(); j++)
            {
               navigableExtrusions[j] = cluster.getNavigableExtrusionInWorld3D(j);
            }
         }

         allNavigableExtrusions[i] = navigableExtrusions;
      }

      return allNavigableExtrusions;
   }

   public VisibilityMapHolder getStartMap()
   {
      return startMap;
   }

   public VisibilityMapHolder getGoalMap()
   {
      return goalMap;
   }

   public List<NavigableRegion> getNavigableRegions()
   {
      return navigableRegions;
   }

   public InterRegionVisibilityMap getInterRegionConnections()
   {
      return interRegionVisibilityMap;
   }
}
