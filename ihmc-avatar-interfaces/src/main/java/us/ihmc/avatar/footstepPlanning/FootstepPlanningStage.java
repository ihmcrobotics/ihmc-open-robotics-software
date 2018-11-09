package us.ihmc.avatar.footstepPlanning;

import com.google.common.util.concurrent.AtomicDouble;
import controller_msgs.msg.dds.TextToSpeechPacket;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.concurrent.ConcurrentCopier;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.listeners.HeuristicSearchAndActionPolicyDefinitions;
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
import us.ihmc.yoVariables.providers.EnumProvider;
import us.ihmc.yoVariables.providers.IntegerProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class FootstepPlanningStage implements FootstepPlanner
{
   private static final boolean debug = false;

   private final YoVariableRegistry registry;

   private FootstepPlanningResult pathPlanResult = null;
   private FootstepPlanningResult stepPlanResult = null;

   private final AtomicReference<BodyPathPlan> pathPlan = new AtomicReference<>();
   private final AtomicReference<FootstepPlan> stepPlan = new AtomicReference<>();

   private final EnumMap<FootstepPlannerType, FootstepPlanner> plannerMap = new EnumMap<>(FootstepPlannerType.class);
   private final EnumProvider<FootstepPlannerType> activePlannerEnum;
   private final IntegerProvider planId;

   private final AtomicDouble timeout = new AtomicDouble();
   private final AtomicDouble horizonLength = new AtomicDouble();
   private final AtomicReference<PlanarRegionsList> planarRegionsList = new AtomicReference<>();
   private final AtomicReference<FootstepPlannerGoal> goal = new AtomicReference<>();
   private final AtomicReference<FramePose3D> stanceFootPose = new AtomicReference<>();
   private final AtomicReference<RobotSide> stanceFootSide = new AtomicReference<>();

   private Runnable stageRunnable;

   private final YoDouble stageTime;
   private final YoBoolean initialize;

   private final YoInteger sequenceId;

   private final int stageId;
   private final long tickDurationMs;

   private final List<PlannerCompletionCallback> completionCallbackList = new ArrayList<>();

   private final FootstepPlannerParameters footstepPlanningParameters;
   private IHMCRealtimeROS2Publisher<TextToSpeechPacket> textToSpeechPublisher;

   private final PlannerGoalRecommendationHolder plannerGoalRecommendationHolder;

   public FootstepPlanningStage(int stageId, RobotContactPointParameters<RobotSide> contactPointParameters, FootstepPlannerParameters footstepPlannerParameters,
                                EnumProvider<FootstepPlannerType> activePlanner, IntegerProvider planId, YoGraphicsListRegistry graphicsListRegistry,
                                long tickDurationMs)
   {
      this.stageId = stageId;
      this.footstepPlanningParameters = footstepPlannerParameters;
      this.planId = planId;
      this.tickDurationMs = tickDurationMs;
      this.activePlannerEnum = activePlanner;

      plannerGoalRecommendationHolder = new PlannerGoalRecommendationHolder(stageId);

      registry = new YoVariableRegistry(stageId + getClass().getSimpleName());

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
      plannerMap.put(FootstepPlannerType.VIS_GRAPH_WITH_A_STAR,
                     new VisibilityGraphWithAStarPlanner(stageId + "", footstepPlanningParameters, contactPointsInSoleFrame, graphicsListRegistry, registry));

      initialize.set(true);
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   private AStarFootstepPlanner createAStarPlanner(SideDependentList<ConvexPolygon2D> footPolygons)
   {
      HeuristicSearchAndActionPolicyDefinitions heuristicPolicies = new HeuristicSearchAndActionPolicyDefinitions();
      heuristicPolicies.setGoalRecommendationListener(plannerGoalRecommendationHolder);
      heuristicPolicies.setAutomaticallyRotate(true);
      heuristicPolicies.setParameters(footstepPlanningParameters);

      FootstepNodeExpansion expansion = new ParameterBasedNodeExpansion(footstepPlanningParameters);
      AStarFootstepPlanner planner = AStarFootstepPlanner
            .createPlanner(footstepPlanningParameters, null, footPolygons, expansion, heuristicPolicies, registry);
      return planner;
   }

   private DepthFirstFootstepPlanner createPlanarRegionBipedalPlanner(SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame)
   {
      FootstepNodeSnapAndWiggler snapper = new FootstepNodeSnapAndWiggler(footPolygonsInSoleFrame, footstepPlanningParameters);
      SnapAndWiggleBasedNodeChecker nodeChecker = new SnapAndWiggleBasedNodeChecker(footPolygonsInSoleFrame, footstepPlanningParameters);
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
      this.stanceFootPose.set(stanceFootPose);
      this.stanceFootSide.set(side);
   }

   public void setGoal(FootstepPlannerGoal goal)
   {
      this.goal.set(goal);
   }

   public void setGoalUnsafe(FootstepPlannerGoal goal)
   {
      setGoal(goal);
      getPlanner().setGoal(goal);
   }

   public void setTimeout(double timeout)
   {
      this.timeout.set(timeout);
   }

   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList.set(planarRegionsList.copy());
   }

   public void setPlanningHorizonLength(double planningHorizonLength)
   {
      this.horizonLength.set(planningHorizonLength);
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
      return getPlanner().planPath();
   }

   @Override
   public FootstepPlanningResult plan()
   {
      return getPlanner().plan();
   }


   public int getStageId()
   {
      return stageId;
   }

   public Runnable createStageRunnable()
   {
      if (stageRunnable != null)
      {
         if (debug)
            PrintTools.error(this, "stageRunnable is not null.");
         return null;
      }

      stageRunnable = new Runnable()
      {
         @Override
         public void run()
         {
            if (Thread.interrupted())
               return;

            update();
         }
      };

      return stageRunnable;
   }

   public void destroyStageRunnable()
   {
      stageRunnable = null;
   }

   public void addCompletionCallback(PlannerCompletionCallback completionCallback)
   {
      completionCallbackList.add(completionCallback);
   }

   public void setPlannerGoalRecommendationHandler(PlannerGoalRecommendationHandler plannerGoalRecommendationHandler)
   {
      this.plannerGoalRecommendationHolder.setPlannerGoalRecommendationHandler(plannerGoalRecommendationHandler);
   }

   public void requestInitialize()
   {
      initialize.set(true);
      getPlanner().requestInitialize();
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
      stageTime.add(Conversions.millisecondsToSeconds(tickDurationMs));
      if (stageTime.getDoubleValue() > 20.0)
      {
         if (debug)
            PrintTools.info("Hard timeout at " + stageTime.getDoubleValue());
         return;
      }

      sendMessageToUI(
            "Starting To Plan: " + planId.getValue() + " sequence: " + sequenceId.getValue() + ", using type " + activePlannerEnum.getValue().toString()
                  + " on stage " + stageId);

      if (debug)
         PrintTools.info("Stage " + stageId + " planning path.");

      pathPlanResult = planPath();
      if (pathPlanResult.validForExecution())
         pathPlan.set(getPlanner().getPathPlan());

      for (PlannerCompletionCallback completionCallback : completionCallbackList)
         completionCallback.pathPlanningIsComplete(pathPlanResult, this);

      if (debug)
         PrintTools.info("Stage " + stageId + " planning steps.");

      stepPlanResult = pathPlanResult;
      if (pathPlanResult.validForExecution())
         stepPlanResult = plan();

      if (stepPlanResult.validForExecution())
         stepPlan.set(getPlanner().getPlan());

      for (PlannerCompletionCallback completionCallback : completionCallbackList)
         completionCallback.stepPlanningIsComplete(stepPlanResult, this);
   }

   public boolean initialize()
   {
      stageTime.set(0.0);

      pathPlanResult = null;
      stepPlanResult = null;

      getPlanner().setInitialStanceFoot(stanceFootPose.get(), stanceFootSide.get());
      getPlanner().setGoal(goal.get());
      getPlanner().setTimeout(timeout.get());
      getPlanner().setPlanarRegions(planarRegionsList.get());
      getPlanner().setPlanningHorizonLength(horizonLength.get());

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
