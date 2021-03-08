package us.ihmc.quadrupedFootstepPlanning.pathPlanning;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.visibilityGraphs.NavigableRegionsManager;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityGraphHolder;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityMapWithNavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityMapHolder;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.BodyPathPostProcessor;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.PathOrientationCalculator;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawStepPlanningResult;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.List;

public class VisibilityGraphPawPathPlanner extends AbstractWaypointsForPawStepPlanner
{
   private final NavigableRegionsManager navigableRegionsManager;
   private final PathOrientationCalculator orientationCalculator;

   private final VisibilityGraphHolder bodyPathPlannerGraph = new VisibilityGraphHolder();

   public VisibilityGraphPawPathPlanner(VisibilityGraphsParametersReadOnly visibilityGraphsParameters, YoRegistry registry)
   {
      this("", visibilityGraphsParameters, registry);
   }

   public VisibilityGraphPawPathPlanner(String prefix, VisibilityGraphsParametersReadOnly visibilityGraphsParameters, YoRegistry registry)
   {
      this(prefix, visibilityGraphsParameters, null, registry);
   }

   public VisibilityGraphPawPathPlanner(String prefix, VisibilityGraphsParametersReadOnly visibilityGraphsParameters,
                                        BodyPathPostProcessor postProcessor, YoRegistry registry)
   {
      super(prefix, registry);
      this.navigableRegionsManager = new NavigableRegionsManager(visibilityGraphsParameters, null, postProcessor);
      this.orientationCalculator = new PathOrientationCalculator(visibilityGraphsParameters);
   }

   public PawStepPlanningResult planWaypoints()
   {
      waypoints.clear();

      if (planarRegionsList == null)
      {
         waypoints.add(new Pose3D(bodyStartPose));
         waypoints.add(new Pose3D(bodyGoalPose));
      }
      else
      {
         Point3DReadOnly startPos = PlanarRegionTools.projectPointToPlanesVertically(bodyStartPose.getPosition(), planarRegionsList);
         Point3DReadOnly goalPos = PlanarRegionTools.projectPointToPlanesVertically(bodyGoalPose.getPosition(), planarRegionsList);

         // FIXME: DO WE WANT TO DO EITHER OF THESE?
         if (startPos == null)
         {
            LogTools.info("adding plane at start pose");
            startPos = new Point3D(bodyStartPose.getPosition());
            addPlanarRegionAtHeight(bodyStartPose.getPosition(), 1738);
         }
         if (goalPos == null)
         {
            LogTools.info("adding plane at goal pose");
            goalPos = new Point3D(bodyGoalPose.getPosition());
            addPlanarRegionAtHeight(bodyGoalPose.getPosition(), 1739);
         }

         if (debug)
         {
            LogTools.info("Starting to plan using " + getClass().getSimpleName());
            LogTools.info("Body start pose: " + startPos);
            LogTools.info("Body goal pose:  " + goalPos);
         }

         navigableRegionsManager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

         try
         {
            List<Point3DReadOnly> pathPoints = navigableRegionsManager.calculateBodyPath(startPos, goalPos);
            List<? extends Pose3DReadOnly> path = orientationCalculator.computePosesFromPath(pathPoints, navigableRegionsManager.getVisibilityMapSolution(),
                                                                                             bodyStartPose.getOrientation(), bodyGoalPose.getOrientation());

            waypoints.addAll(path);
         }
         catch (Exception e)
         {
            e.printStackTrace();
            yoResult.set(PawStepPlanningResult.PLANNER_FAILED);
            return yoResult.getEnumValue();
         }
      }

      if (waypoints.size() < 2)
         yoResult.set(PawStepPlanningResult.PLANNER_FAILED);
      else
         yoResult.set(PawStepPlanningResult.SUB_OPTIMAL_SOLUTION);
      return yoResult.getEnumValue();
   }

   public VisibilityGraphHolder getPlannerStatistics()
   {
      packVisibilityGraphStatistics(bodyPathPlannerGraph);
      return bodyPathPlannerGraph;
   }

   public void cancelPlanning()
   {
   }

   public void setTimeout(double timeout)
   {
   }

   // TODO hack to add start and goal planar regions


   private void packVisibilityGraphStatistics(VisibilityGraphHolder statistics)
   {
      VisibilityMapHolder startMap = navigableRegionsManager.getStartMap();
      VisibilityMapHolder goalMap = navigableRegionsManager.getGoalMap();
      VisibilityMapHolder interRegionsMap = navigableRegionsManager.getInterRegionConnections();
      List<VisibilityMapWithNavigableRegion> visibilityMapWithNavigableRegions = navigableRegionsManager.getNavigableRegionsList();

      statistics.setStartVisibilityMapInWorld(startMap.getMapId(), startMap.getVisibilityMapInWorld());
      statistics.setGoalVisibilityMapInWorld(goalMap.getMapId(), goalMap.getVisibilityMapInWorld());
      statistics.setInterRegionsVisibilityMapInWorld(interRegionsMap.getMapId(), interRegionsMap.getVisibilityMapInWorld());
      statistics.addNavigableRegions(visibilityMapWithNavigableRegions);
   }
}
