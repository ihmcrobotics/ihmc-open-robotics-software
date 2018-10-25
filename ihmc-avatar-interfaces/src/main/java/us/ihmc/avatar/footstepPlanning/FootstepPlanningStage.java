package us.ihmc.avatar.footstepPlanning;

import controller_msgs.msg.dds.*;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.SnapAndWiggleBasedNodeChecker;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.FootstepNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.ParameterBasedNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.planners.AStarFootstepPlanner;
import us.ihmc.footstepPlanning.graphSearch.planners.BodyPathBasedFootstepPlanner;
import us.ihmc.footstepPlanning.graphSearch.planners.DepthFirstFootstepPlanner;
import us.ihmc.footstepPlanning.graphSearch.planners.VisibilityGraphWithAStarPlanner;
import us.ihmc.footstepPlanning.graphSearch.stepCost.ConstantFootstepCost;
import us.ihmc.footstepPlanning.simplePlanners.PlanThenSnapPlanner;
import us.ihmc.footstepPlanning.simplePlanners.TurnWalkTurnPlanner;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.pathPlanning.statistics.PlannerStatistics;
import us.ihmc.pathPlanning.visibilityGraphs.tools.BodyPathPlan;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.Optional;

public class FootstepPlanningStage implements FootstepPlanner, Runnable
{
   private static final boolean debug = false;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final EnumMap<FootstepPlannerType, FootstepPlanner> plannerMap = new EnumMap<>(FootstepPlannerType.class);
   private final YoEnum<FootstepPlannerType> activePlannerEnum;

   private final YoBoolean donePlanningPath = new YoBoolean("donePlanningPath", registry);
   private final YoBoolean donePlanningSteps = new YoBoolean("donePlanningSteps", registry);

   private final YoDouble toolboxTime = new YoDouble("ToolboxTime", registry);
   private final YoInteger planId = new YoInteger("planId", registry);
   private final YoBoolean initialize = new YoBoolean("initialize" + registry.getName(), registry);

   private double dt;

   private final FootstepPlannerParameters footstepPlanningParameters;
   private IHMCRealtimeROS2Publisher<TextToSpeechPacket> textToSpeechPublisher;

   public FootstepPlanningStage(RobotContactPointParameters<RobotSide> contactPointParameters, FootstepPlannerParameters footstepPlannerParameters,
                                YoEnum<FootstepPlannerType> activePlanner, YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry,
                                double dt)
   {
      this.dt = dt;
      this.footstepPlanningParameters = footstepPlannerParameters;
      this.activePlannerEnum = activePlanner;

      SideDependentList<ConvexPolygon2D> contactPointsInSoleFrame;
      if (contactPointParameters == null)
         contactPointsInSoleFrame = PlannerTools.createDefaultFootPolygons();
      else
         contactPointsInSoleFrame = createFootPolygonsFromContactPoints(contactPointParameters);

      plannerMap.put(FootstepPlannerType.PLANAR_REGION_BIPEDAL, createPlanarRegionBipedalPlanner(contactPointsInSoleFrame));
      plannerMap.put(FootstepPlannerType.PLAN_THEN_SNAP, new PlanThenSnapPlanner(new TurnWalkTurnPlanner(), contactPointsInSoleFrame));
      plannerMap.put(FootstepPlannerType.A_STAR, createAStarPlanner(contactPointsInSoleFrame));
      plannerMap
            .put(FootstepPlannerType.SIMPLE_BODY_PATH, new BodyPathBasedFootstepPlanner(footstepPlanningParameters, contactPointsInSoleFrame, parentRegistry));
      plannerMap.put(FootstepPlannerType.VIS_GRAPH_WITH_A_STAR,
                     new VisibilityGraphWithAStarPlanner(footstepPlanningParameters, contactPointsInSoleFrame, graphicsListRegistry, parentRegistry));

      donePlanningSteps.set(true);
      planId.set(FootstepPlanningRequestPacket.NO_PLAN_ID);
   }

   private AStarFootstepPlanner createAStarPlanner(SideDependentList<ConvexPolygon2D> footPolygons)
   {
      FootstepNodeExpansion expansion = new ParameterBasedNodeExpansion(footstepPlanningParameters);
      AStarFootstepPlanner planner = AStarFootstepPlanner.createPlanner(footstepPlanningParameters, null, footPolygons, expansion, registry);
      return planner;
   }

   private DepthFirstFootstepPlanner createPlanarRegionBipedalPlanner(SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame)
   {
      FootstepNodeSnapAndWiggler snapper = new FootstepNodeSnapAndWiggler(footPolygonsInSoleFrame, footstepPlanningParameters, null);
      SnapAndWiggleBasedNodeChecker nodeChecker = new SnapAndWiggleBasedNodeChecker(footPolygonsInSoleFrame, null, footstepPlanningParameters);
      ConstantFootstepCost stepCostCalculator = new ConstantFootstepCost(1.0);

      DepthFirstFootstepPlanner footstepPlanner = new DepthFirstFootstepPlanner(footstepPlanningParameters, snapper, nodeChecker, stepCostCalculator, registry);
      footstepPlanner.setFeetPolygons(footPolygonsInSoleFrame, footPolygonsInSoleFrame);
      footstepPlanner.setMaximumNumberOfNodesToExpand(Integer.MAX_VALUE);
      footstepPlanner.setExitAfterInitialSolution(false);

      return footstepPlanner;
   }

   protected boolean donePlanningSteps()
   {
      return donePlanningSteps.getBooleanValue();
   }

   protected boolean donePlanningPath()
   {
      return donePlanningPath.getBooleanValue();
   }

   private FootstepPlanner getPlanner()
   {
      return plannerMap.get(activePlannerEnum.getEnumValue());
   }

   public void setInitialStanceFoot(FramePose3D stanceFootPose, RobotSide side)
   {
      getPlanner().setInitialStanceFoot(stanceFootPose, side);
   }

   public void setGoal(FootstepPlannerGoal goal)
   {
      getPlanner().setGoal(goal);
   }

   public void setTimeout(double timeout)
   {
      getPlanner().setTimeout(timeout);
   }

   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      getPlanner().setPlanarRegions(planarRegionsList);
   }

   public void setPlanningHorizonLength(double planningHorizonLength)
   {
      getPlanner().setPlanningHorizonLength(planningHorizonLength);
   }

   public double getPlanningDuration()
   {
      return getPlanner().getPlanningDuration();
   }

   public PlannerStatistics<?> getPlannerStatistics()
   {
      return getPlanner().getPlannerStatistics();
   }

   public BodyPathPlan getPathPlan()
   {
      return getPlanner().getPathPlan();
   }

   public FootstepPlan getPlan()
   {
      return getPlanner().getPlan();
   }

   @Override
   public FootstepPlanningResult planPath()
   {
      donePlanningPath.set(false);

      FootstepPlanningResult result = getPlanner().planPath();

      donePlanningPath.set(true);

      return result;
   }

   @Override
   public FootstepPlanningResult plan()
   {
      donePlanningSteps.set(false);

      FootstepPlanningResult result = getPlanner().plan();

      donePlanningPath.set(true);

      return result;
   }

   @Override
   public void run()
   {
      if (Thread.interrupted())
         return;

      update();
      toolboxTime.add(dt);
   }

   /**
    * Get the initialization state of this toolbox controller:
    * <ul>
    * <li>{@code true}: this toolbox controller has been initialized properly and is ready for doing
    * some computation!
    * <li>{@code false}: this toolbox controller has either not been initialized yet or the
    * initialization process failed.
    * </ul>
    *
    * @return the initialization state of this toolbox controller.
    */
   public boolean hasBeenInitialized()
   {
      return !initialize.getValue();
   }

   /**
    * Request this toolbox controller to run {@link #initialize} on the next call of
    * {@link #update()}.
    */
   public void requestInitialize()
   {
      initialize.set(true);
   }

   public void update()
   {
      if (!hasBeenInitialized())
      {
         if (!initialize()) // Return until the initialization succeeds
            return;
         initialize.set(false);
      }

      updateInternal();
   }

   private void updateInternal()
   {
      toolboxTime.add(dt);
      if (toolboxTime.getDoubleValue() > 20.0)
      {
         if (debug)
            PrintTools.info("Hard timeout at " + toolboxTime.getDoubleValue());
         donePlanningSteps.set(true);
         donePlanningPath.set(true);
         return;
      }

      sendMessageToUI("Starting To Plan: " + planId.getIntegerValue() + ", " + activePlannerEnum.getEnumValue().toString());

      FootstepPlanningResult status = planPath();

      if (status.validForExecution())
      {
         getPathPlan();

         status = plan();
      }

      getPlan();

      sendMessageToUI("Result: " + planId.getIntegerValue() + ", " + status.toString());


   }

   protected boolean initialize()
   {
      donePlanningSteps.set(false);
      donePlanningPath.set(false);

      toolboxTime.set(0.0);

      return true;
   }

   private void sendMessageToUI(String message)
   {
      textToSpeechPublisher.publish(MessageTools.createTextToSpeechPacket(message));
   }

   private static SideDependentList<ConvexPolygon2D> createFootPolygonsFromContactPoints(RobotContactPointParameters<RobotSide> contactPointParameters)
   {
      SideDependentList<ConvexPolygon2D> footPolygons = new SideDependentList<>();
      for (RobotSide side : RobotSide.values)
      {
         ArrayList<Point2D> footPoints = contactPointParameters.getFootContactPoints().get(side);
         ConvexPolygon2D scaledFoot = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(footPoints));
         footPolygons.set(side, scaledFoot);
      }

      return footPolygons;
   }

}
