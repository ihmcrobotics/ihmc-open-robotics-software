package us.ihmc.footstepPlanning.graphSearch.planners;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.pathPlanners.SplinePathPlanner;
import us.ihmc.footstepPlanning.graphSearch.pathPlanners.WaypointsForFootstepsPlanner;
import us.ihmc.pathPlanning.bodyPathPlanner.WaypointDefinedBodyPathPlanner;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class SplinePathWithAStarPlanner implements BodyPathAndFootstepPlanner
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private static final boolean debug = false;
   private static final RobotSide defaultStartNodeSide = RobotSide.LEFT;
   private final YoDouble timeout = new YoDouble("timeout", registry);

   private final YoBoolean hasPath = new YoBoolean("hasPath", registry);
   private final YoDouble timeSpentBeforeFootstepPlanner = new YoDouble("timeSpentBeforeFootstepPlanner", registry);
   private final YoDouble timeSpentInFootstepPlanner = new YoDouble("timeSpentInFootstepPlanner", registry);
   private final YoDouble planningHorizonLength = new YoDouble("planningHorizonLength", registry);
   private final YoEnum<FootstepPlanningResult> yoResult = new YoEnum<>("planningResult", registry, FootstepPlanningResult.class);

   private final WaypointsForFootstepsPlanner pathPlanner;
   private final WaypointDefinedBodyPathPlanner bodyPath;
   private final FootstepPlanner footstepPlanner;


   public SplinePathWithAStarPlanner(FootstepPlannerParameters parameters, SideDependentList<ConvexPolygon2D> footPolygons, YoVariableRegistry parentRegistry)
   {
      pathPlanner = new SplinePathPlanner(parameters, registry);
      bodyPath = new WaypointDefinedBodyPathPlanner();

      footstepPlanner = new BodyPathBasedAStarPlanner(parameters, footPolygons, parameters.getCostParameters().getBodyPathBasedHeuristicsWeight(), registry);

      planningHorizonLength.set(1.0);

      parentRegistry.addChild(registry);
   }

   @Override
   public void setInitialStanceFoot(FramePose3D stanceFootPose, RobotSide side)
   {
      if (side == null)
      {
         if (debug)
            PrintTools.info("Start node needs a side, but trying to set it to null. Setting it to " + defaultStartNodeSide);

         side = defaultStartNodeSide;
      }

      pathPlanner.setInitialStanceFoot(stanceFootPose, side);
      footstepPlanner.setInitialStanceFoot(stanceFootPose, side);

      hasPath.set(false);
   }

   @Override
   public void setGoal(FootstepPlannerGoal goal)
   {
      pathPlanner.setGoal(goal);
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
      bodyPath.setPlanarRegionsList(planarRegionsList);
      footstepPlanner.setPlanarRegions(planarRegionsList);
   }

   @Override
   public double getPlanningDuration()
   {
      return footstepPlanner.getPlanningDuration();
   }

   @Override
   public void setPlanningHorizonLength(double planningHorizonLength)
   {
      this.planningHorizonLength.set(planningHorizonLength);

      hasPath.set(false);
   }

   @Override
   public FootstepPlanningResult planPath()
   {
      long startTime = System.currentTimeMillis();

      pathPlanner.planWaypoints();
      bodyPath.setWaypoints(pathPlanner.getWaypoints());
      bodyPath.compute();

      double seconds = (System.currentTimeMillis() - startTime) / 1000.0;
      timeSpentBeforeFootstepPlanner.set(seconds);

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

      return yoResult.getEnumValue();
   }

   @Override
   public FootstepPlan getPlan()
   {
      return footstepPlanner.getPlan();
   }
}
