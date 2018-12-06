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
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityMapSolution;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityMapWithNavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphsParameters;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityMapHolder;
import us.ihmc.pathPlanning.visibilityGraphs.tools.ClusterTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.OcclusionTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegion;

public class ComputeAllVisibilityConnectonsNavigableRegionsManager
{
   private final static boolean debug = false;
   private final VisibilityGraphsParameters parameters;

   private final VisibilityMapSolution visibilityMapSolution = new VisibilityMapSolution();
   
   public ComputeAllVisibilityConnectonsNavigableRegionsManager()
   {
      this(null, null);
   }

   public ComputeAllVisibilityConnectonsNavigableRegionsManager(VisibilityGraphsParameters parameters)
   {
      this(parameters, null);
   }

   public ComputeAllVisibilityConnectonsNavigableRegionsManager(List<PlanarRegion> regions)
   {
      this(null, regions);
   }

   public ComputeAllVisibilityConnectonsNavigableRegionsManager(VisibilityGraphsParameters parameters, List<PlanarRegion> regions)
   {
      visibilityMapSolution.setNavigableRegions(new NavigableRegions(parameters, regions));      
      this.parameters = parameters == null ? new DefaultVisibilityGraphParameters() : parameters;
   }

   private static ArrayList<VisibilityMapWithNavigableRegion> createListOfVisibilityMapsWithNavigableRegions(NavigableRegions navigableRegions)
   {
      ArrayList<VisibilityMapWithNavigableRegion> list = new ArrayList<>();

      List<NavigableRegion> naviableRegionsList = navigableRegions.getNaviableRegionsList();

      for (NavigableRegion navigableRegion : naviableRegionsList)
      {
         VisibilityMapWithNavigableRegion visibilityMapWithNavigableRegion = new VisibilityMapWithNavigableRegion(navigableRegion);
         list.add(visibilityMapWithNavigableRegion);
      }

      return list;
   }

   public List<VisibilityMapWithNavigableRegion> getNavigableRegionsList()
   {
      return visibilityMapSolution.getVisibilityMapsWithNavigableRegions();
   }

   public void setPlanarRegions(List<PlanarRegion> planarRegions)
   {
      visibilityMapSolution.getNavigableRegions().setPlanarRegions(planarRegions);
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

      NavigableRegions navigableRegions = visibilityMapSolution.getNavigableRegions();
      
      navigableRegions.filterPlanarRegionsWithBoundingCapsule(start, goal, parameters.getExplorationDistanceFromStartGoal());

      long startBodyPathComputation = System.currentTimeMillis();

      navigableRegions.createNavigableRegions();

      //FIXME: +++JEP 181203. We have a bug where the the path can cross over a hole when there is an object on top of the hole. Need to fix that.

      //TODO: Do this stuff lazily, rather than all up front for efficiency.
      
      //Note: This has to be done before inter regions if the inter regions are computed using inner regions.
      // Otherwise, the ordering does not matter.
      //      VisibilityGraphsFactory.createStaticVisibilityMapsForNavigableRegions(navigableRegions);

      ArrayList<VisibilityMapWithNavigableRegion> visibilityMapsWithNavigableRegions = createListOfVisibilityMapsWithNavigableRegions(navigableRegions);
      
      visibilityMapSolution.setVisibilityMapsWithNavigableRegions(visibilityMapsWithNavigableRegions);
      
      InterRegionVisibilityMap interRegionVisibilityMap = VisibilityGraphsFactory.createInterRegionVisibilityMap(visibilityMapsWithNavigableRegions,
                                                             parameters.getInterRegionConnectionFilter());
      visibilityMapSolution.setInterRegionVisibilityMap(interRegionVisibilityMap);
      
      VisibilityGraphsFactory.createStaticVisibilityMapsForNavigableRegions(visibilityMapsWithNavigableRegions);

      double searchHostEpsilon = parameters.getSearchHostRegionEpsilon();
      SingleSourceVisibilityMap startMap = VisibilityGraphsFactory.createSingleSourceVisibilityMap(start, visibilityMapsWithNavigableRegions, searchHostEpsilon,
                                                                         interRegionVisibilityMap.getVisibilityMapInLocal());
      SingleSourceVisibilityMap goalMap = VisibilityGraphsFactory.createSingleSourceVisibilityMap(goal, visibilityMapsWithNavigableRegions, searchHostEpsilon,
                                                                        interRegionVisibilityMap.getVisibilityMapInLocal());

      int START_GOAL_ID = 0;

      if (goalMap == null)
      {
         goalMap = VisibilityGraphsFactory.connectToClosestPoints(new ConnectionPoint3D(goal, START_GOAL_ID), 1, visibilityMapsWithNavigableRegions,
                                                                  START_GOAL_ID);
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
            startMap = VisibilityGraphsFactory.connectToClosestPoints(new ConnectionPoint3D(start, START_GOAL_ID), 1, visibilityMapsWithNavigableRegions,
                                                                      START_GOAL_ID);
      }

      visibilityMapSolution.setStartMap(startMap);
      visibilityMapSolution.setGoalMap(goalMap);
      
      if (startMap == null)
         return null;

      List<VisibilityMapHolder> visibilityMapHolders = new ArrayList<>();
      visibilityMapHolders.addAll(visibilityMapsWithNavigableRegions);
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
         ArrayList<VisibilityMapWithNavigableRegion> visibilityMapsWithNavigableRegions = createListOfVisibilityMapsWithNavigableRegions(visibilityMapSolution.getNavigableRegions());

         if (!OcclusionTools.isTheGoalIntersectingAnyObstacles(visibilityMapsWithNavigableRegions.get(0), start, goal))
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

         VisibilityMapWithNavigableRegion regionContainingPoint = PlanarRegionTools.getNavigableRegionContainingThisPoint(start,
                                                                                                                          visibilityMapsWithNavigableRegions);
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
      return visibilityMapSolution.getStartMap();
   }

   public VisibilityMapHolder getGoalMap()
   {
      return visibilityMapSolution.getGoalMap();

   }

   public InterRegionVisibilityMap getInterRegionConnections()
   {
      return visibilityMapSolution.getInterRegionVisibilityMap();

   }

}
