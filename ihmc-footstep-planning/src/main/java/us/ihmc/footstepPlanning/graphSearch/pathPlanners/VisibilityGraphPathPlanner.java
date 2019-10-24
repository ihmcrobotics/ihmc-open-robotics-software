package us.ihmc.footstepPlanning.graphSearch.pathPlanners;

import java.util.List;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.statistics.VisibilityGraphStatistics;
import us.ihmc.pathPlanning.visibilityGraphs.NavigableRegionsManager;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityMapWithNavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityMapHolder;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.ObstacleAndCliffAvoidanceProcessor;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.PathOrientationCalculator;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class VisibilityGraphPathPlanner extends AbstractWaypointsForFootstepsPlanner
{
   private final NavigableRegionsManager navigableRegionsManager;
   private final PathOrientationCalculator pathOrientationCalculator;

   private final VisibilityGraphStatistics visibilityGraphStatistics = new VisibilityGraphStatistics();

   public VisibilityGraphPathPlanner(FootstepPlannerParametersReadOnly footstepPlannerParameters, VisibilityGraphsParametersReadOnly visibilityGraphsParameters,
                                     YoVariableRegistry parentRegistry)
   {
      this("", footstepPlannerParameters, visibilityGraphsParameters, parentRegistry);
   }

   public VisibilityGraphPathPlanner(FootstepPlannerParametersReadOnly footstepPlannerParameters, VisibilityGraphsParametersReadOnly visibilityGraphsParameters,
                                     ObstacleAndCliffAvoidanceProcessor pathPostProcessor, YoVariableRegistry parentRegistry)
   {
      this("", footstepPlannerParameters, visibilityGraphsParameters, pathPostProcessor, parentRegistry);
   }

   public VisibilityGraphPathPlanner(String prefix, FootstepPlannerParametersReadOnly footstepPlannerParameters, VisibilityGraphsParametersReadOnly visibilityGraphsParameters,
                                     YoVariableRegistry parentRegistry)
   {
      this(prefix, footstepPlannerParameters, visibilityGraphsParameters, null, parentRegistry);
   }

   public VisibilityGraphPathPlanner(String prefix, FootstepPlannerParametersReadOnly footstepPlannerParameters, VisibilityGraphsParametersReadOnly visibilityGraphsParameters,
                                     ObstacleAndCliffAvoidanceProcessor postProcessor, YoVariableRegistry parentRegistry)
   {
      super(prefix, footstepPlannerParameters, parentRegistry);

      this.navigableRegionsManager = new NavigableRegionsManager(visibilityGraphsParameters, null, postProcessor);
      this.pathOrientationCalculator = new PathOrientationCalculator(visibilityGraphsParameters);
   }

   public FootstepPlanningResult planWaypoints()
   {
      waypoints.clear();

      if (planarRegionsList == null)
      {
         waypoints.add(new Pose3D(bodyStartPose));
         waypoints.add(new Pose3D(bodyGoalPose));
      }
      else
      {
         Point3DReadOnly startPosition = PlanarRegionTools.projectPointToPlanesVertically(bodyStartPose.getPosition(), planarRegionsList);
         Point3DReadOnly goalPosition = PlanarRegionTools.projectPointToPlanesVertically(bodyGoalPose.getPosition(), planarRegionsList);
         navigableRegionsManager.setPlanarRegions(planarRegionsList.getPlanarRegionsAsList());

         if (startPosition == null)
         {
            LogTools.info("adding plane at start foot");
            startPosition = new Point3D(bodyStartPose.getX(), bodyStartPose.getY(), 0.0);
            addPlanarRegionAtZeroHeight(bodyStartPose.getX(), bodyStartPose.getY());
         }
         if (goalPosition == null)
         {
            LogTools.info("adding plane at goal pose");
            goalPosition = new Point3D(bodyGoalPose.getX(), bodyGoalPose.getY(), 0.0);
            addPlanarRegionAtZeroHeight(bodyGoalPose.getX(), bodyGoalPose.getY());
         }

         if (debug)
         {
            LogTools.info("Starting to plan using " + getClass().getSimpleName());
            LogTools.info("Body start pose: " + startPosition);
            LogTools.info("Body goal pose:  " + goalPosition);
         }

         try
         {
            List<Point3DReadOnly> path = navigableRegionsManager.calculateBodyPath(startPosition, goalPosition);
            List<? extends Pose3DReadOnly> posePath = pathOrientationCalculator.computePosesFromPath(path, navigableRegionsManager.getVisibilityMapSolution(),
                                                                                                     bodyStartPose.getOrientation(), bodyGoalPose.getOrientation());

            waypoints.addAll(posePath);
         }
         catch (Exception e)
         {
            e.printStackTrace();
            yoResult.set(FootstepPlanningResult.PLANNER_FAILED);
            return yoResult.getEnumValue();
         }
      }

      yoResult.set(FootstepPlanningResult.SUB_OPTIMAL_SOLUTION);
      return yoResult.getEnumValue();
   }

   public VisibilityGraphStatistics getPlannerStatistics()
   {
      packVisibilityGraphStatistics(visibilityGraphStatistics);
      return visibilityGraphStatistics;
   }

   public void cancelPlanning()
   {
   }

   public void setTimeout(double timeout)
   {
   }

   // TODO hack to add start and goal planar regions
   private void addPlanarRegionAtZeroHeight(double xLocation, double yLocation)
   {
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(0.3, 0.3);
      polygon.addVertex(-0.3, 0.3);
      polygon.addVertex(0.3, -0.3);
      polygon.addVertex(-0.3, -0.25);
      polygon.update();

      PlanarRegion planarRegion = new PlanarRegion(new RigidBodyTransform(new AxisAngle(), new Vector3D(xLocation, yLocation, 0.0)), polygon);
      planarRegionsList.addPlanarRegion(planarRegion);
   }

   private void packVisibilityGraphStatistics(VisibilityGraphStatistics statistics)
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
