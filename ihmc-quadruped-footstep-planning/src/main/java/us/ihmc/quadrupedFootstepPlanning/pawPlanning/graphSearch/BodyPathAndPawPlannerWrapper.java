package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch;

import controller_msgs.msg.dds.QuadrupedGroundPlaneMessage;
import org.apache.commons.math3.util.Precision;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.bodyPathPlanner.BodyPathPlanHolder;
import us.ihmc.pathPlanning.bodyPathPlanner.WaypointDefinedBodyPathPlanHolder;
import us.ihmc.pathPlanning.visibilityGraphs.tools.BodyPathPlan;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.*;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawStepPlannerParametersReadOnly;
import us.ihmc.quadrupedFootstepPlanning.pathPlanning.WaypointsForPawStepPlanner;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoFramePoint3D;

import java.util.ArrayList;
import java.util.List;

public class BodyPathAndPawPlannerWrapper implements BodyPathAndPawPlanner
{
   private static final boolean DEBUG = true;

   private static final double defaultTimeout = 5.0;
   private static final double defaultBestEffortTimeout = 0.0;

   protected final YoVariableRegistry registry;

   private final YoDouble timeout;
   private final YoDouble bestEffortTimeout;

   private final YoBoolean hasPath;
   private final YoDouble timeSpentBeforePawPlanner;
   private final YoDouble timeSpentInPawPlanner;
   private final YoEnum<PawStepPlanningResult> yoResult;

   protected final BodyPathPlanHolder bodyPathPlanner = new WaypointDefinedBodyPathPlanHolder();
   protected WaypointsForPawStepPlanner waypointPathPlanner;
   protected PawStepPlanner pawStepPlanner;

   private PlanarRegionsList planarRegionsList;

   private final boolean visualizing;
   private static final int bodyPathPointsForVisualization = 100;
   private final List<YoFramePoint3D> bodyPathPoints = new ArrayList<>();

   public BodyPathAndPawPlannerWrapper(String prefix, PawStepPlannerParametersReadOnly parameters, YoVariableRegistry parentRegistry,
                                       YoGraphicsListRegistry graphicsListRegistry)
   {
      registry = new YoVariableRegistry(prefix + getClass().getSimpleName());

      timeout = new YoDouble("timeout", registry);
      bestEffortTimeout = new YoDouble("bestEffortTimeout", registry);

      hasPath = new YoBoolean("hasPath", registry);
      timeSpentBeforePawPlanner = new YoDouble("timeSpentBeforePawPlanner", registry);
      timeSpentInPawPlanner = new YoDouble("timeSpentInPawPlanner", registry);
      yoResult = new YoEnum<>("planningResult", registry, PawStepPlanningResult.class);

      timeout.set(defaultTimeout);
      bestEffortTimeout.set(defaultBestEffortTimeout);
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
   public WaypointsForPawStepPlanner getWaypointPathPlanner()
   {
      return waypointPathPlanner;
   }

   @Override
   public PawStepPlanner getPawStepPlanner()
   {
      return pawStepPlanner;
   }

   @Override
   public void setStart(PawStepPlannerStart start)
   {
      waypointPathPlanner.setInitialBodyPose(start.getTargetPose());

      pawStepPlanner.setStart(start);

      hasPath.set(false);
   }

   @Override
   public void setGoal(PawStepPlannerGoal goal)
   {
      waypointPathPlanner.setGoal(goal);

      pawStepPlanner.setGoal(goal);

      hasPath.set(false);
   }

   @Override
   public void setTimeout(double timeout)
   {
      this.timeout.set(timeout);
   }

   @Override
   public void setBestEffortTimeout(double timeout)
   {
      this.bestEffortTimeout.set(timeout);
   }

   @Override
   public void setGroundPlane(QuadrupedGroundPlaneMessage message)
   {
   }

   @Override
   public void setPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
      waypointPathPlanner.setPlanarRegionsList(planarRegionsList);
      pawStepPlanner.setPlanarRegionsList(planarRegionsList);
      this.planarRegionsList = planarRegionsList;
   }

   @Override
   public double getPlanningDuration()
   {
      return timeSpentBeforePawPlanner.getDoubleValue() + timeSpentInPawPlanner.getDoubleValue();
   }

   @Override
   public void setPlanningHorizonLength(double planningHorizon)
   {
      pawStepPlanner.setPlanningHorizonLength(planningHorizon);
      hasPath.set(false);
   }

   @Override
   public PawStepPlanningResult planPath()
   {
      long startTime = System.currentTimeMillis();

      PawStepPlanningResult pathResult = waypointPathPlanner.planWaypoints();

      if (pathResult == PawStepPlanningResult.PLANNER_FAILED)
      {
         double seconds = (System.currentTimeMillis() - startTime) / 1000.0;
         timeSpentBeforePawPlanner.set(seconds);
         timeSpentInPawPlanner.set(0.0);
         yoResult.set(pathResult);
         return yoResult.getEnumValue();
      }

      List<Pose3DReadOnly> waypoints = waypointPathPlanner.getWaypoints();

      if (waypoints.size() < 2)
      {
//         if (parameters.getReturnBestEffortPlan())
//         {
//            waypointPathPlanner.computeBestEffortPlan(pawPlanner.getPlanningHorizonLength());
//         }
//         else
//         {
            double seconds = (System.currentTimeMillis() - startTime) / 1000.0;
            timeSpentBeforePawPlanner.set(seconds);
            timeSpentInPawPlanner.set(0.0);
            yoResult.set(PawStepPlanningResult.PLANNER_FAILED);
            return yoResult.getEnumValue();
//         }
      }

      bodyPathPlanner.setPoseWaypoints(waypoints);

      if (visualizing)
      {
         updateBodyPathVisualization();
      }

      double seconds = (System.currentTimeMillis() - startTime) / 1000.0;
      timeSpentBeforePawPlanner.set(seconds);

      hasPath.set(true);

      yoResult.set(PawStepPlanningResult.SUB_OPTIMAL_SOLUTION);
      return yoResult.getEnumValue();
   }

   @Override
   public PawStepPlanningResult plan()
   {
      if (!hasPath.getBooleanValue())
      {
         PawStepPlanningResult pathResult = planPath();
         if (!pathResult.validForExecution())
            return pathResult;
      }

      double bestEffortTimeout = this.bestEffortTimeout.getDoubleValue() - timeSpentBeforePawPlanner.getDoubleValue();
      pawStepPlanner.setTimeout(timeout.getDoubleValue() - timeSpentBeforePawPlanner.getDoubleValue());
      pawStepPlanner.setBestEffortTimeout(bestEffortTimeout);

      long startTime = System.currentTimeMillis();
      yoResult.set(pawStepPlanner.plan());
      double seconds = (System.currentTimeMillis() - startTime) / 1000.0;
      timeSpentInPawPlanner.set(seconds);

      if (DEBUG)
      {
         LogTools.info("Visibility graph with A* planner finished. Result: " + yoResult.getEnumValue());
         System.out.println("   Finished planning body path after " + Precision.round(timeSpentBeforePawPlanner.getDoubleValue(), 2) + " seconds.");
         System.out.println("   Finished planning paw path after " + Precision.round(timeSpentInPawPlanner.getDoubleValue(), 2) + " seconds.");
      }

      return yoResult.getEnumValue();
   }

   private void updateBodyPathVisualization()
   {
      Pose3D tempPose = new Pose3D();
      for (int i = 0; i < bodyPathPointsForVisualization; i++)
      {
         double percent = (double) i / (double) (bodyPathPointsForVisualization - 1);
         bodyPathPlanner.getPointAlongPath(percent, tempPose);
         bodyPathPoints.get(i).set(tempPose.getPosition());
      }
   }

   @Override
   public void cancelPlanning()
   {
      pawStepPlanner.cancelPlanning();
   }

   @Override
   public PawStepPlan getPlan()
   {
      return pawStepPlanner.getPlan();
   }

   @Override
   public BodyPathPlan getPathPlan()
   {
      return bodyPathPlanner.getPlan();
   }

//   @Override
//   public ListOfStatistics getPlannerStatistics()
//   {
//      listOfStatistics.clear();
//
//      listOfStatistics.addStatistics(waypointPathPlanner.getPlannerStatistics());
//
//      return listOfStatistics;
//   }
}
