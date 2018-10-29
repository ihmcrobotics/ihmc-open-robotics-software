package us.ihmc.avatar.footstepPlanning;

import controller_msgs.msg.dds.*;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple2D.Point2D;
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
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.providers.EnumProvider;
import us.ihmc.yoVariables.providers.IntegerProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class FootstepPlanningStage implements FootstepPlanner, Runnable
{
   private static final boolean debug = false;

   private final YoVariableRegistry registry;

   private final AtomicReference<FootstepPlanningResult> pathPlanResult = new AtomicReference<>();
   private final AtomicReference<FootstepPlanningResult> stepPlanResult = new AtomicReference<>();

   private final AtomicReference<BodyPathPlan> pathPlan = new AtomicReference<>();
   private final AtomicReference<FootstepPlan> stepPlan = new AtomicReference<>();

   private final EnumMap<FootstepPlannerType, FootstepPlanner> plannerMap = new EnumMap<>(FootstepPlannerType.class);
   private final EnumProvider<FootstepPlannerType> activePlannerEnum;
   private final IntegerProvider planId;

   private final YoBoolean donePlanningPath;
   private final YoBoolean donePlanningSteps;

   private final YoDouble stageTime;
   private final YoBoolean initialize;

   private final YoInteger sequenceId;

   private final int stageId;
   private final double dt;

   private final List<PlannerCompletionCallback> completionCallbackList = new ArrayList<>();

   private final FootstepPlannerParameters footstepPlanningParameters;
   private IHMCRealtimeROS2Publisher<TextToSpeechPacket> textToSpeechPublisher;

   public FootstepPlanningStage(int stageId, RobotContactPointParameters<RobotSide> contactPointParameters, FootstepPlannerParameters footstepPlannerParameters,
                                EnumProvider<FootstepPlannerType> activePlanner, IntegerProvider planId, YoGraphicsListRegistry graphicsListRegistry, double dt)
   {
      this.stageId = stageId;
      this.footstepPlanningParameters = footstepPlannerParameters;
      this.planId = planId;
      this.dt = dt;
      this.activePlannerEnum = activePlanner;

      registry = new YoVariableRegistry(stageId + getClass().getSimpleName());

      donePlanningPath = new YoBoolean(stageId + "_DonePlanningPath", registry);
      donePlanningSteps = new YoBoolean(stageId + "_DonePlanningSteps", registry);

      stageTime = new YoDouble(stageId + "_StageTime", registry);
      initialize = new YoBoolean(stageId + "_Initialize" + registry.getName(), registry);
      sequenceId = new YoInteger(stageId + "_PlanningSequenceId", registry);

      SideDependentList<ConvexPolygon2D> contactPointsInSoleFrame;
      if (contactPointParameters == null)
         contactPointsInSoleFrame = PlannerTools.createDefaultFootPolygons();
      else
         contactPointsInSoleFrame = createFootPolygonsFromContactPoints(contactPointParameters);

      plannerMap.put(FootstepPlannerType.PLANAR_REGION_BIPEDAL, createPlanarRegionBipedalPlanner(contactPointsInSoleFrame));
      plannerMap.put(FootstepPlannerType.PLAN_THEN_SNAP, new PlanThenSnapPlanner(new TurnWalkTurnPlanner(), contactPointsInSoleFrame));
      plannerMap.put(FootstepPlannerType.A_STAR, createAStarPlanner(contactPointsInSoleFrame));
      plannerMap.put(FootstepPlannerType.SIMPLE_BODY_PATH, new BodyPathBasedFootstepPlanner(footstepPlanningParameters, contactPointsInSoleFrame, registry));
      // FIXME this should use a real graphics list registry
      plannerMap.put(FootstepPlannerType.VIS_GRAPH_WITH_A_STAR,
                     new VisibilityGraphWithAStarPlanner(footstepPlanningParameters, contactPointsInSoleFrame, null, registry));

      donePlanningSteps.set(true);
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
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

   private FootstepPlanner getPlanner()
   {
      return plannerMap.get(activePlannerEnum.getValue());
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

   public void setPlanSequenceId(int sequenceId)
   {
      this.sequenceId.set(sequenceId);
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
      return pathPlan.getAndSet(null);
   }

   public FootstepPlan getPlan()
   {
      return stepPlan.getAndSet(null);
   }

   public int getPlanSequenceId()
   {
      return sequenceId.getIntegerValue();
   }

   @Override
   public FootstepPlanningResult planPath()
   {
      donePlanningPath.set(false);

      FootstepPlanningResult result = getPlanner().planPath();
      pathPlanResult.set(result);

      donePlanningPath.set(true);

      return result;
   }

   @Override
   public FootstepPlanningResult plan()
   {
      donePlanningSteps.set(false);

      FootstepPlanningResult result = getPlanner().plan();
      stepPlanResult.set(result);

      donePlanningPath.set(true);

      return result;
   }

   @Override
   public void run()
   {
      if (Thread.interrupted())
         return;

      update();
      stageTime.add(dt);
   }

   public void addCompletionCallback(PlannerCompletionCallback completionCallback)
   {
      completionCallbackList.add(completionCallback);
   }

   public void requestInitialize()
   {
      initialize.set(true);
   }

   public void update()
   {
      if (initialize.getBooleanValue())
      {
         if (!initialize()) // Return until the initialization succeeds
            return;
         initialize.set(false);
      }

      updateInternal();
   }

   private void updateInternal()
   {
      stageTime.add(dt);
      if (stageTime.getDoubleValue() > 20.0)
      {
         if (debug)
            PrintTools.info("Hard timeout at " + stageTime.getDoubleValue());
         donePlanningSteps.set(true);
         donePlanningPath.set(true);
         return;
      }

      sendMessageToUI("Starting To Plan: " + planId.getValue() + ", using type " + activePlannerEnum.getValue().toString() + " on stage " + stageId);

      FootstepPlanningResult status = getPlanner().planPath();
      pathPlanResult.set(status);
      if (status.validForExecution())
         pathPlan.set(getPlanner().getPathPlan());

      for (PlannerCompletionCallback completionCallback : completionCallbackList)
         completionCallback.pathPlanningIsComplete(status, this);

      if (status.validForExecution())
         status = getPlanner().plan();

      stepPlanResult.set(status);
      if (status.validForExecution())
         stepPlan.set(getPlanner().getPlan());

      for (PlannerCompletionCallback completionCallback : completionCallbackList)
         completionCallback.stepPlanningIsComplete(status, this);

      sendMessageToUI("Result: " + planId.getValue() + ", " + status.toString() + " on stage " + stageId);
   }

   public boolean initialize()
   {
      donePlanningSteps.set(false);
      donePlanningPath.set(false);

      stageTime.set(0.0);

      return true;
   }

   public void setTextToSpeechPublisher(IHMCRealtimeROS2Publisher<TextToSpeechPacket> textToSpeechPublisher)
   {
      this.textToSpeechPublisher = textToSpeechPublisher;
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
