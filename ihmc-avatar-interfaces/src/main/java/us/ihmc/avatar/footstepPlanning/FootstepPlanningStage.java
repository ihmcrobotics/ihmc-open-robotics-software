package us.ihmc.avatar.footstepPlanning;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.EnumMap;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import com.google.common.util.concurrent.AtomicDouble;

import controller_msgs.msg.dds.TextToSpeechPacket;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.graphSearch.collision.FootstepNodeBodyCollisionDetector;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.SimplePlanarRegionFootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.MultiStagePlannerListener;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.StagePlannerListener;
import us.ihmc.footstepPlanning.graphSearch.heuristics.DistanceAndYawBasedHeuristics;
import us.ihmc.footstepPlanning.graphSearch.listeners.HeuristicSearchAndActionPolicyDefinitions;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.*;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.FootstepNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.ParameterBasedNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.planners.AStarFootstepPlanner;
import us.ihmc.footstepPlanning.graphSearch.planners.BodyPathBasedAStarPlanner;
import us.ihmc.footstepPlanning.graphSearch.stepCost.FootstepCost;
import us.ihmc.footstepPlanning.graphSearch.stepCost.FootstepCostBuilder;
import us.ihmc.footstepPlanning.simplePlanners.PlanThenSnapPlanner;
import us.ihmc.footstepPlanning.simplePlanners.TurnWalkTurnPlanner;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.bodyPathPlanner.BodyPathPlanner;
import us.ihmc.pathPlanning.statistics.PlannerStatistics;
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

public class FootstepPlanningStage implements FootstepPlanner
{
   private static final boolean debug = false;

   private final YoVariableRegistry registry;

   private FootstepPlanningResult stepPlanResult = null;

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
                                BodyPathPlanner bodyPathPlanner, EnumProvider<FootstepPlannerType> activePlanner, MultiStagePlannerListener multiStageListener,
                                IntegerProvider planId, long tickDurationMs)

   {
      this.stageId = stageId;
      this.footstepPlanningParameters = footstepPlannerParameters;
      this.planId = planId;
      this.tickDurationMs = tickDurationMs;
      this.activePlannerEnum = activePlanner;

      String prefix = stageId + "_Step_";

      plannerGoalRecommendationHolder = new PlannerGoalRecommendationHolder(stageId);

      registry = new YoVariableRegistry(prefix + getClass().getSimpleName());

      stageTime = new YoDouble(prefix + "StageTime", registry);
      initialize = new YoBoolean(prefix + "Initialize", registry);
      sequenceId = new YoInteger(prefix + "PlanningSequenceId", registry);

      SideDependentList<ConvexPolygon2D> contactPointsInSoleFrame;
      if (contactPointParameters == null)
         contactPointsInSoleFrame = PlannerTools.createDefaultFootPolygons();
      else
         contactPointsInSoleFrame = createFootPolygonsFromContactPoints(contactPointParameters);

      plannerMap.put(FootstepPlannerType.PLAN_THEN_SNAP, new PlanThenSnapPlanner(new TurnWalkTurnPlanner(footstepPlannerParameters), contactPointsInSoleFrame));
      plannerMap.put(FootstepPlannerType.A_STAR, createAStarPlanner(contactPointsInSoleFrame, multiStageListener));
      plannerMap.put(FootstepPlannerType.SIMPLE_BODY_PATH,
                     new BodyPathBasedAStarPlanner("simple_",
                                                   bodyPathPlanner,
                                                   footstepPlannerParameters,
                                                   contactPointsInSoleFrame,
                                                   footstepPlannerParameters.getCostParameters().getBodyPathBasedHeuristicsWeight(),
                                                   registry));
      plannerMap.put(FootstepPlannerType.VIS_GRAPH_WITH_A_STAR,
                     createBodyPathBasedAStarPlanner(footstepPlannerParameters, bodyPathPlanner, multiStageListener, contactPointsInSoleFrame));

      initialize.set(true);
   }

   private BodyPathBasedAStarPlanner createBodyPathBasedAStarPlanner(FootstepPlannerParameters footstepPlannerParameters, BodyPathPlanner bodyPathPlanner,
                                                                     MultiStagePlannerListener multiStageListener,
                                                                     SideDependentList<ConvexPolygon2D> contactPointsInSoleFrame)
   {
      StagePlannerListener plannerListener = new StagePlannerListener(null, multiStageListener.getBroadcastDt());
      multiStageListener.addStagePlannerListener(plannerListener);
      return new BodyPathBasedAStarPlanner("visGraph_",
                                                                          bodyPathPlanner,
                                                                          footstepPlannerParameters,
                                                                          contactPointsInSoleFrame,
                                                                          footstepPlannerParameters.getCostParameters().getAStarHeuristicsWeight(),
                                                                          registry,
                                                                          plannerListener);
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   private AStarFootstepPlanner createAStarPlanner(SideDependentList<ConvexPolygon2D> footPolygons, MultiStagePlannerListener multiStageListener)
   {
      HeuristicSearchAndActionPolicyDefinitions heuristicPolicies = new HeuristicSearchAndActionPolicyDefinitions();
      heuristicPolicies.setGoalRecommendationListener(plannerGoalRecommendationHolder);
      heuristicPolicies.setAutomaticallyRotate(true);
      heuristicPolicies.setParameters(footstepPlanningParameters);

      FootstepNodeExpansion expansion = new ParameterBasedNodeExpansion(footstepPlanningParameters);

      FootstepNodeBodyCollisionDetector collisionDetector = new FootstepNodeBodyCollisionDetector(footstepPlanningParameters);
      SimplePlanarRegionFootstepNodeSnapper snapper = new SimplePlanarRegionFootstepNodeSnapper(footPolygons);
      FootstepNodeSnapAndWiggler postProcessingSnapper = new FootstepNodeSnapAndWiggler(footPolygons, footstepPlanningParameters, collisionDetector);

      SnapBasedNodeChecker snapBasedNodeChecker = new SnapBasedNodeChecker(footstepPlanningParameters, footPolygons, snapper);
      BodyCollisionNodeChecker bodyCollisionNodeChecker = new BodyCollisionNodeChecker(collisionDetector, footstepPlanningParameters, snapper);
      PlanarRegionBaseOfCliffAvoider cliffAvoider = new PlanarRegionBaseOfCliffAvoider(footstepPlanningParameters, snapper, footPolygons);

      DistanceAndYawBasedHeuristics heuristics = new DistanceAndYawBasedHeuristics(footstepPlanningParameters.getCostParameters().getAStarHeuristicsWeight(),
                                                                                   footstepPlanningParameters);

      StagePlannerListener plannerListener = new StagePlannerListener(snapper, multiStageListener.getBroadcastDt());
      FootstepNodeChecker nodeChecker = new FootstepNodeCheckerOfCheckers(Arrays.asList(snapBasedNodeChecker, bodyCollisionNodeChecker, cliffAvoider));
      nodeChecker.addPlannerListener(plannerListener);
      multiStageListener.addStagePlannerListener(plannerListener);

      FootstepCostBuilder costBuilder = new FootstepCostBuilder();
      costBuilder.setFootstepPlannerParameters(footstepPlanningParameters);
      costBuilder.setSnapper(snapper);
      costBuilder.setIncludeHeightCost(true);
      costBuilder.setIncludePitchAndRollCost(true);
      costBuilder.setIncludeBoundingBoxCost(true);
      costBuilder.setCollisionDetector(collisionDetector);

      FootstepCost footstepCost = costBuilder.buildCost();

      AStarFootstepPlanner planner = new AStarFootstepPlanner(footstepPlanningParameters, nodeChecker, heuristics, expansion, footstepCost,
                                                              postProcessingSnapper, plannerListener, footPolygons, registry);

      heuristicPolicies.setCollisionNodeChecker(bodyCollisionNodeChecker);
      heuristicPolicies.setNodeSnapper(snapper);
      heuristicPolicies.build();

      heuristicPolicies.getPlannerListeners().forEach(nodeChecker::addPlannerListener);
      heuristicPolicies.getStartAndGoalListeners().forEach(planner::addStartAndGoalListener);

      return planner;
   }

   private FootstepPlanner getPlanner()
   {
      return plannerMap.get(activePlannerEnum.getValue());
   }

   public int getPlanSequenceId()
   {
      return sequenceId.getIntegerValue();
   }

   public int getStageId()
   {
      return stageId;
   }

   @Override
   public void setInitialStanceFoot(FramePose3D stanceFootPose, RobotSide side)
   {
      this.stanceFootPose.set(stanceFootPose);
      this.stanceFootSide.set(side);
   }

   @Override
   public void setGoal(FootstepPlannerGoal goal)
   {
      this.goal.set(goal);
   }

   public void setGoalUnsafe(FootstepPlannerGoal goal)
   {
      setGoal(goal);
      getPlanner().setGoal(goal);
   }

   @Override
   public void setTimeout(double timeout)
   {
      this.timeout.set(timeout);
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList.set(planarRegionsList == null ? null : planarRegionsList.copy());
   }

   @Override
   public void setPlanningHorizonLength(double planningHorizonLength)
   {
      this.horizonLength.set(planningHorizonLength);
   }

   public void setPlanSequenceId(int sequenceId)
   {
      this.sequenceId.set(sequenceId);
   }

   @Override
   public double getPlanningDuration()
   {
      return getPlanner().getPlanningDuration();
   }

   @Override
   public PlannerStatistics<?> getPlannerStatistics()
   {
      return getPlanner().getPlannerStatistics();
   }

   @Override
   public FootstepPlan getPlan()
   {
      return stepPlan.getAndSet(null);
   }



   @Override
   public FootstepPlanningResult plan()
   {
      return getPlanner().plan();
   }


   public Runnable createStageRunnable()
   {
      if (stageRunnable != null)
      {
         if (debug)
            LogTools.error("StageRunnable is not null.");
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

   @Override
   public void requestInitialize()
   {
      initialize.set(true);
      getPlanner().requestInitialize();
   }


   public boolean initialize()
   {
      stageTime.set(0.0);

      stepPlanResult = null;

      getPlanner().setInitialStanceFoot(stanceFootPose.get(), stanceFootSide.get());
      getPlanner().setGoal(goal.get());
      getPlanner().setTimeout(timeout.get());
      getPlanner().setPlanarRegions(planarRegionsList.get());
      getPlanner().setPlanningHorizonLength(horizonLength.get());

      return true;
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
            LogTools.error("Hard timeout at " + stageTime.getDoubleValue());
         return;
      }

      sendMessageToUI(
            "Starting To Plan: " + planId.getValue() + " sequence: " + sequenceId.getValue() + ", using type " + activePlannerEnum.getValue().toString()
                  + " on stage " + stageId);

      if (debug)
         LogTools.error("Stage " + stageId + " planning steps.");

      stepPlanResult = plan();

      if (stepPlanResult.validForExecution())
         stepPlan.set(getPlanner().getPlan());

      for (PlannerCompletionCallback completionCallback : completionCallbackList)
         completionCallback.stepPlanningIsComplete(stepPlanResult, this);
   }


   @Override
   public void cancelPlanning()
   {
      getPlanner().cancelPlanning();
      stepPlanResult = null;
      stepPlan.set(null);
      for (PlannerCompletionCallback completionCallback : completionCallbackList)
      {
         completionCallback.stepPlanningIsComplete(stepPlanResult, this);
      }
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
