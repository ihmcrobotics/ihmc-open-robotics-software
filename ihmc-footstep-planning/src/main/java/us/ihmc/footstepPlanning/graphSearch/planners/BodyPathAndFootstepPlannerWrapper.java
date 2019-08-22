package us.ihmc.footstepPlanning.graphSearch.planners;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.graphSearch.pathPlanners.WaypointsForFootstepsPlanner;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.pathPlanning.bodyPathPlanner.BodyPathPlanner;
import us.ihmc.pathPlanning.bodyPathPlanner.WaypointDefinedBodyPathPlanner;
import us.ihmc.pathPlanning.statistics.ListOfStatistics;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphsParameters;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.PathOrientationCalculator;
import us.ihmc.pathPlanning.visibilityGraphs.tools.BodyPathPlan;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoFramePoint3D;

import java.util.ArrayList;
import java.util.List;

public class BodyPathAndFootstepPlannerWrapper implements BodyPathAndFootstepPlanner
{
   private static final boolean DEBUG = false;
   private static final RobotSide defaultStartNodeSide = RobotSide.LEFT;

   private static final double defaultTimeout = 5.0;

   protected final YoVariableRegistry registry;

   private final YoDouble timeout;

   private final YoBoolean hasPath;
   private final YoDouble timeSpentBeforeFootstepPlanner;
   private final YoDouble timeSpentInFootstepPlanner;
   private final YoEnum<FootstepPlanningResult> yoResult;

   private final FootstepPlannerParametersReadOnly parameters;

   protected final BodyPathPlanner bodyPathPlanner = new WaypointDefinedBodyPathPlanner();
   protected WaypointsForFootstepsPlanner waypointPathPlanner;
   protected FootstepPlanner footstepPlanner;

   private PlanarRegionsList planarRegionsList;

   private final boolean visualizing;
   private static final int bodyPathPointsForVisualization = 100;
   private final List<YoFramePoint3D> bodyPathPoints = new ArrayList<>();

   private final ListOfStatistics listOfStatistics = new ListOfStatistics();

   public BodyPathAndFootstepPlannerWrapper(String prefix, FootstepPlannerParametersReadOnly parameters,
                                            YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      registry = new YoVariableRegistry(prefix + getClass().getSimpleName());
      this.parameters = parameters;

      timeout = new YoDouble("timeout", registry);

      hasPath = new YoBoolean("hasPath", registry);
      timeSpentBeforeFootstepPlanner = new YoDouble("timeSpentBeforeFootstepPlanner", registry);
      timeSpentInFootstepPlanner = new YoDouble("timeSpentInFootstepPlanner", registry);
      yoResult = new YoEnum<>("planningResult", registry, FootstepPlanningResult.class);

      timeout.set(defaultTimeout);
      visualizing = graphicsListRegistry != null;
      if (visualizing)
      {
         setupVisualization(prefix, graphicsListRegistry, registry);
      }

      parentRegistry.addChild(registry);
   }


   private void setupVisualization(String prefix, YoGraphicsListRegistry graphicsListRegistry, YoVariableRegistry registry)
   {
      YoGraphicsList yoGraphicsList = new YoGraphicsList(prefix + "VisGraph");

      for (int i = 0; i < bodyPathPointsForVisualization; i++)
      {
         YoFramePoint3D point = new YoFramePoint3D(prefix + "BodyPathPoint" + i, ReferenceFrame.getWorldFrame(), registry);
         point.setToNaN();
         bodyPathPoints.add(point);
         YoGraphicPosition pointVisualization = new YoGraphicPosition(prefix + "BodyPathPoint" + i, point, 0.02, YoAppearance.Yellow());
         yoGraphicsList.add(pointVisualization);
      }

      graphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
   }

   @Override
   public void setInitialStanceFoot(FramePose3D stanceFootPose, RobotSide side)
   {
      if (side == null)
      {
         if (DEBUG)
            PrintTools.info("Start node needs a side, but trying to set it to null. Setting it to " + defaultStartNodeSide);

         side = defaultStartNodeSide;
      }

      waypointPathPlanner.setInitialStanceFoot(stanceFootPose, side);

      footstepPlanner.setInitialStanceFoot(stanceFootPose, side);

      hasPath.set(false);
   }

   @Override
   public void setGoal(FootstepPlannerGoal goal)
   {
      waypointPathPlanner.setGoal(goal);
      footstepPlanner.setGoal(goal);

      hasPath.set(false);
   }

   @Override
   public void setTimeout(double timeout)
   {
      this.timeout.set(timeout);
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      waypointPathPlanner.setPlanarRegionsList(planarRegionsList);
      footstepPlanner.setPlanarRegions(planarRegionsList);
      this.planarRegionsList = planarRegionsList;
   }

   @Override
   public double getPlanningDuration()
   {
      return timeSpentBeforeFootstepPlanner.getDoubleValue() + timeSpentInFootstepPlanner.getDoubleValue();
   }

   @Override
   public void setPlanningHorizonLength(double planningHorizon)
   {
      footstepPlanner.setPlanningHorizonLength(planningHorizon);

      hasPath.set(false);
   }

   @Override
   public FootstepPlanningResult planPath()
   {
      long startTime = System.currentTimeMillis();

      FootstepPlanningResult pathResult = waypointPathPlanner.planWaypoints();

      if (pathResult == FootstepPlanningResult.PLANNER_FAILED)
      {
         double seconds = (System.currentTimeMillis() - startTime) / 1000.0;
         timeSpentBeforeFootstepPlanner.set(seconds);
         timeSpentInFootstepPlanner.set(0.0);
         yoResult.set(pathResult);
         return yoResult.getEnumValue();
      }

      List<Pose3D> waypoints = waypointPathPlanner.getWaypoints();

      if (waypoints.size() < 2)
      {
         if (parameters.getReturnBestEffortPlan())
         {
            waypointPathPlanner.computeBestEffortPlan(footstepPlanner.getPlanningHorizonLength());
         }
         else
         {
            double seconds = (System.currentTimeMillis() - startTime) / 1000.0;
            timeSpentBeforeFootstepPlanner.set(seconds);
            timeSpentInFootstepPlanner.set(0.0);
            yoResult.set(FootstepPlanningResult.PLANNER_FAILED);
            return yoResult.getEnumValue();
         }
      }

      bodyPathPlanner.setPoseWaypoints(waypoints);

      if (visualizing)
      {
         updateBodyPathVisualization();
      }

      double seconds = (System.currentTimeMillis() - startTime) / 1000.0;
      timeSpentBeforeFootstepPlanner.set(seconds);

      hasPath.set(true);

      yoResult.set(FootstepPlanningResult.SUB_OPTIMAL_SOLUTION);
      return yoResult.getEnumValue();
   }

   @Override
   public FootstepPlanningResult plan()
   {
      if (!hasPath.getBooleanValue())
      {
         FootstepPlanningResult pathResult = planPath();
         if (!pathResult.validForExecution())
            return pathResult;
      }

      footstepPlanner.setTimeout(timeout.getDoubleValue() - timeSpentBeforeFootstepPlanner.getDoubleValue());

      long startTime = System.currentTimeMillis();
      yoResult.set(footstepPlanner.plan());
      double seconds = (System.currentTimeMillis() - startTime) / 1000.0;
      timeSpentInFootstepPlanner.set(seconds);

      if (DEBUG)
      {
         PrintTools.info("Visibility graph with A* planner finished. Result: " + yoResult.getEnumValue());
      }

      return yoResult.getEnumValue();
   }

   private void updateBodyPathVisualization()
   {
      Pose2D tempPose = new Pose2D();
      for (int i = 0; i < bodyPathPointsForVisualization; i++)
      {
         double percent = (double) i / (double) (bodyPathPointsForVisualization - 1);
         bodyPathPlanner.getPointAlongPath(percent, tempPose);
         Point3D position = new Point3D();
         position.set(tempPose.getPosition());
         Point3DReadOnly projectedPoint = PlanarRegionTools.projectPointToPlanesVertically(position, planarRegionsList);
         if (projectedPoint != null)
         {
            bodyPathPoints.get(i).set(projectedPoint);
         }
         else
         {
            bodyPathPoints.get(i).setToNaN();
         }
      }
   }

   @Override
   public void cancelPlanning()
   {
      footstepPlanner.cancelPlanning();
   }

   @Override
   public FootstepPlan getPlan()
   {
      return footstepPlanner.getPlan();
   }

   @Override
   public BodyPathPlan getPathPlan()
   {
      return bodyPathPlanner.getPlan();
   }

   @Override
   public ListOfStatistics getPlannerStatistics()
   {
      listOfStatistics.clear();

      listOfStatistics.addStatistics(waypointPathPlanner.getPlannerStatistics());

      return listOfStatistics;
   }
}
