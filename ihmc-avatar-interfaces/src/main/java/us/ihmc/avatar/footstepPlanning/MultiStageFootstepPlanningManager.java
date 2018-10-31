package us.ihmc.avatar.footstepPlanning;

import controller_msgs.msg.dds.*;
import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.concurrent.ConcurrentCopier;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.YoFootstepPlannerParameters;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.pathPlanning.statistics.ListOfStatistics;
import us.ihmc.pathPlanning.statistics.PlannerStatistics;
import us.ihmc.pathPlanning.statistics.VisibilityGraphStatistics;
import us.ihmc.pathPlanning.visibilityGraphs.tools.BodyPathPlan;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.graphics.YoGraphicPlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.lists.PairList;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.*;

import java.util.*;
import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicReference;

public class MultiStageFootstepPlanningManager implements PlannerCompletionCallback
{
   private static final boolean debug = false;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoEnum<FootstepPlannerType> activePlanner = new YoEnum<>("activePlanner", registry, FootstepPlannerType.class);

   private final AtomicReference<FootstepPlanningRequestPacket> latestRequestReference = new AtomicReference<>(null);
   private final AtomicReference<FootstepPlannerParametersPacket> latestParametersReference = new AtomicReference<>(null);

   private final AtomicReference<BodyPathPlan> bodyPathPlan = new AtomicReference<>(null);
   private final AtomicReference<FootstepPlan> footstepPlan = new AtomicReference<>(null);

   private Optional<PlanarRegionsList> planarRegionsList = Optional.empty();

   private final YoBoolean isDone = new YoBoolean("isDone", registry);
   private final YoBoolean requestedPlanarRegions = new YoBoolean("RequestedPlanarRegions", registry);
   private final YoDouble plannerTime = new YoDouble("PlannerTime", registry);
   private final YoDouble timeout = new YoDouble("PlannerTimeout", registry);
   private final YoInteger planId = new YoInteger("planId", registry);
   private final YoInteger globalSequenceIndex = new YoInteger("globalSequenceIndex", registry);

   private final YoGraphicPlanarRegionsList yoGraphicPlanarRegionsList;

   private final List<FootstepPlannerObjective> planningObjectivePool = new ArrayList<>();
   private final PlannerGoalRecommendationHandler goalRecommendationHandler;

   private final ConcurrentCopier<List<FootstepPlanningStage>> planningStagePool = new ConcurrentCopier<>(ArrayList::new);
   private final ConcurrentCopier<List<FootstepPlanningStage>> allPlanningStages = new ConcurrentCopier<>(ArrayList::new);
   private final ConcurrentCopier<HashMap<FootstepPlanningStage, FootstepPlannerObjective>> pathPlanningStagesInProgress = new ConcurrentCopier<>(HashMap::new);
   private final ConcurrentCopier<HashMap<FootstepPlanningStage, FootstepPlannerObjective>> stepPlanningStagesInProgress = new ConcurrentCopier<>(HashMap::new);

   private final ConcurrentCopier<HashMap<FootstepPlanningStage, ScheduledFuture<?>>> planningTasks = new ConcurrentCopier<>(HashMap::new);

   private final YoBoolean isDonePlanningPath = new YoBoolean("isDonePlanningPath", registry);
   private final YoBoolean isDonePlanningSteps = new YoBoolean("isDonePlanningSteps", registry);

   private final ConcurrentList<FootstepPlanningResult> completedPathResults = new ConcurrentList<>();
   private final ConcurrentList<FootstepPlanningResult> completedStepResults = new ConcurrentList<>();

   private final ConcurrentCopier<PairList<Integer, BodyPathPlan>> completedPathPlans = new ConcurrentCopier<>(PairList::new);
   private final ConcurrentCopier<PairList<Integer, FootstepPlan>> completedStepPlans = new ConcurrentCopier<>(PairList::new);

   private long tickDurationMs;

   private final YoFootstepPlannerParameters footstepPlanningParameters;
   private IHMCRealtimeROS2Publisher<TextToSpeechPacket> textToSpeechPublisher;

   private final RobotContactPointParameters<RobotSide> contactPointParameters;

   private final StatusMessageOutputManager statusOutputManager;
   private final ScheduledExecutorService executorService;
   private final YoBoolean initialize = new YoBoolean("initialize" + registry.getName(), registry);

   public MultiStageFootstepPlanningManager(RobotContactPointParameters<RobotSide> contactPointParameters, FootstepPlannerParameters footstepPlannerParameters,
                                            StatusMessageOutputManager statusOutputManager, YoVariableRegistry parentRegistry,
                                            YoGraphicsListRegistry graphicsListRegistry, long tickDurationMs)
   {
      this.contactPointParameters = contactPointParameters;
      this.statusOutputManager = statusOutputManager;
      this.tickDurationMs = tickDurationMs;
      this.yoGraphicPlanarRegionsList = new YoGraphicPlanarRegionsList("FootstepPlannerPlanarRegions", 200, 30, registry);
      goalRecommendationHandler = new PlannerGoalRecommendationHandler(allPlanningStages, stepPlanningStagesInProgress);

      ThreadFactory threadFactory = ThreadTools.getNamedThreadFactory(getClass().getSimpleName());
      executorService = Executors.newScheduledThreadPool(5, threadFactory);

      footstepPlanningParameters = new YoFootstepPlannerParameters(registry, footstepPlannerParameters);

      activePlanner.set(FootstepPlannerType.PLANAR_REGION_BIPEDAL);

      graphicsListRegistry.registerYoGraphic("footstepPlanning", yoGraphicPlanarRegionsList);
      isDone.set(false);
      planId.set(FootstepPlanningRequestPacket.NO_PLAN_ID);

      List<FootstepPlanningStage> allPlanningStages = this.allPlanningStages.getCopyForWriting();
      List<FootstepPlanningStage> planningStagePool = this.planningStagePool.getCopyForWriting();
      for (int i = 0; i < 5; i++)
      {
         FootstepPlanningStage planningStage = new FootstepPlanningStage(i, contactPointParameters, footstepPlannerParameters, activePlanner, planId,
                                                                         graphicsListRegistry, tickDurationMs);
         planningStage.addCompletionCallback(this);
         planningStage.setPlannerGoalRecommendationHandler(goalRecommendationHandler);
         registry.addChild(planningStage.getYoVariableRegistry());
         allPlanningStages.add(planningStage);
         planningStagePool.add(planningStage);
      }
      this.allPlanningStages.commit();
      this.planningStagePool.commit();

      isDonePlanningPath.set(false);
      isDonePlanningSteps.set(false);

      isDonePlanningPath.addVariableChangedListener(v -> {
         if (!isDonePlanningPath.getBooleanValue())
            statusOutputManager.reportStatusMessage(FootstepPlanningMessageReporter.packStatus(FootstepPlannerStatus.PLANNING_PATH));
         else
            statusOutputManager.reportStatusMessage(FootstepPlanningMessageReporter.packStatus(FootstepPlannerStatus.PLANNING_STEPS));
      });
      isDonePlanningSteps.addVariableChangedListener(v -> {
         if (isDonePlanningSteps.getBooleanValue())
            statusOutputManager.reportStatusMessage(FootstepPlanningMessageReporter.packStatus(FootstepPlannerStatus.IDLE));
      });

      parentRegistry.addChild(registry);
      initialize.set(true);
   }

   private FootstepPlanningStage createNewFootstepPlanningStage()
   {
      FootstepPlanningStage stage = new FootstepPlanningStage(allPlanningStages.getCopyForReading().size(), contactPointParameters, footstepPlanningParameters, activePlanner,
                                                              planId, null, tickDurationMs);
      stage.addCompletionCallback(this);
      stage.setPlannerGoalRecommendationHandler(goalRecommendationHandler);
      return stage;
   }

   private FootstepPlanningStage spawnNextAvailablePlanner()
   {
      List<FootstepPlanningStage> existingPlanningStagePool = planningStagePool.getCopyForReading();

      FootstepPlanningStage stageToReturn;

      if (existingPlanningStagePool.isEmpty())
      { // create a new stage
         stageToReturn = createNewFootstepPlanningStage();

         List<FootstepPlanningStage> existingPlanningStages = allPlanningStages.getCopyForReading();
         List<FootstepPlanningStage> updatedPlanningStages = allPlanningStages.getCopyForWriting();
         updatedPlanningStages.addAll(existingPlanningStages);
         updatedPlanningStages.add(stageToReturn);

         allPlanningStages.commit();
      }
      else
      { // get one fro the pool
         List<FootstepPlanningStage> updatedPlanningStagePool = planningStagePool.getCopyForWriting();

         updatedPlanningStagePool.addAll(existingPlanningStagePool);
         stageToReturn = updatedPlanningStagePool.remove(0);

         planningStagePool.commit();
      }


      return stageToReturn;
   }

   private FootstepPlanningStage cleanupPlanningStage(FootstepPlanningStage planningStage)
   {
      stepPlanningStagesInProgress.remove(planningStage);
      planningStage.requestInitialize();
      planningTasks.remove(planningStage).cancel(true);

      return planningStage;
   }

   private void cleanupAllPlanningStages()
   {
      globalSequenceIndex.set(0);
      pathPlanningStagesInProgress.clear();

      for (FootstepPlanningStage stepPlanningStageInProgress : stepPlanningStagesInProgress.keySet())
      {
         planningStagePool.add(cleanupPlanningStage(stepPlanningStageInProgress));
      }

      completedPathPlans.getCopyForWriting();
      completedPathPlans.commit();
      completedStepPlans.getCopyForWriting();
      completedStepPlans.commit();

      completedPathResults.getCopyForWriting();
      completedPathResults.commit();
      completedStepResults.getCopyForWriting();
      completedStepResults.commit();

      planningObjectivePool.clear();
   }

   private void assignGoalsToAvailablePlanners()
   {
      if (planningObjectivePool.isEmpty())
         return;

      if (planningStagePool.isEmpty())
         return;

      while (!planningObjectivePool.isEmpty() && !planningStagePool.isEmpty())
      {
         FootstepPlanningStage planner = spawnNextAvailablePlanner();
         FootstepPlannerObjective plannerGoal = planningObjectivePool.remove(0);

         globalSequenceIndex.increment();
         planner.setFootstepPlannerObjective(plannerGoal);
         planner.setPlanSequenceId(globalSequenceIndex.getIntegerValue());

         PlanarRegionsList planarRegionsList;
         if (this.planarRegionsList.isPresent())
         {
            planarRegionsList = this.planarRegionsList.get();
            yoGraphicPlanarRegionsList.submitPlanarRegionsListToRender(planarRegionsList);
            yoGraphicPlanarRegionsList.processPlanarRegionsListQueue();
         }
         else
         {
            planarRegionsList = null;
            yoGraphicPlanarRegionsList.clear();
         }

         planner.setPlanarRegions(planarRegionsList);

         Runnable runnable = planner.createStageRunnable();
         ScheduledFuture<?> plannerTask = executorService.schedule(runnable, 0, TimeUnit.MILLISECONDS);
         planningTasks.put(planner, plannerTask);

         pathPlanningStagesInProgress.put(planner, plannerGoal);
         stepPlanningStagesInProgress.put(planner, plannerGoal);
      }

      return;
   }

   private void updatePlanningObjectives()
   {
      while (goalRecommendationHandler.hasNewFootstepPlannerObjectives())
         planningObjectivePool.add(goalRecommendationHandler.pollNextFootstepPlannerObjective());
   }

   @Override
   public void pathPlanningIsComplete(FootstepPlanningResult pathPlanningResult, FootstepPlanningStage stageFinished)
   {
      List<FootstepPlanningResult> currentPathPlanResults = completedPathResults.getCopyForReading();
      List<FootstepPlanningResult> allPathPlanResults = completedPathResults.getCopyForWriting();
      if (currentPathPlanResults != null)
         allPathPlanResults.addAll(currentPathPlanResults);
      allPathPlanResults.add(pathPlanningResult);
      completedPathResults.commit();

      PairList<Integer, BodyPathPlan> currentPathPlans = completedPathPlans.getCopyForReading();
      PairList<Integer, BodyPathPlan> allCompletedPathPlans = completedPathPlans.getCopyForWriting();

      BodyPathPlan bodyPathPlan = stageFinished.getPathPlan();

      if (currentPathPlans != null)
         allCompletedPathPlans.addAll(currentPathPlans);
      if (pathPlanningResult.validForExecution() && bodyPathPlan != null)
      {
         allCompletedPathPlans.add(new ImmutablePair<>(stageFinished.getPlanSequenceId(), bodyPathPlan));
      }
      completedPathPlans.commit();

      HashMap<FootstepPlanningStage, FootstepPlannerObjective> currentPlanningStagesInProgress = pathPlanningStagesInProgress.getCopyForReading();
      HashMap<FootstepPlanningStage, FootstepPlannerObjective> updatedPlanningStagesInProgress = pathPlanningStagesInProgress.getCopyForWriting();
      updatedPlanningStagesInProgress.putAll(currentPlanningStagesInProgress);
      updatedPlanningStagesInProgress.remove(stageFinished);

      pathPlanningStagesInProgress.commit();
   }

   @Override
   public void stepPlanningIsComplete(FootstepPlanningResult stepPlanningResult, FootstepPlanningStage stageFinished)
   {
      List<FootstepPlanningResult> currentStepPlanResults = completedStepResults.getCopyForReading();
      List<FootstepPlanningResult> allStepPlanResults = completedStepResults.getCopyForWriting();
      if (currentStepPlanResults != null)
         allStepPlanResults.addAll(currentStepPlanResults);
      allStepPlanResults.add(stepPlanningResult);
      completedStepResults.commit();

      PairList<Integer, FootstepPlan> currentStepPlans = completedStepPlans.getCopyForReading();
      PairList<Integer, FootstepPlan> allCompletedStepPlans = completedStepPlans.getCopyForWriting();

      if (currentStepPlans != null)
         allCompletedStepPlans.addAll(currentStepPlans);
      if (stepPlanningResult.validForExecution())
      {
         allCompletedStepPlans.add(new ImmutablePair<>(stageFinished.getPlanSequenceId(), stageFinished.getPlan()));
      }
      completedStepPlans.commit();

      HashMap<FootstepPlanningStage, FootstepPlannerObjective> currentPlanningStagesInProgress = stepPlanningStagesInProgress.getCopyForReading();
      HashMap<FootstepPlanningStage, FootstepPlannerObjective> updatedPlanningStagesInProgress = stepPlanningStagesInProgress.getCopyForWriting();
      updatedPlanningStagesInProgress.putAll(currentPlanningStagesInProgress);
      updatedPlanningStagesInProgress.remove(stageFinished);
      stepPlanningStagesInProgress.commit();

      List<FootstepPlanningStage> currentStagePool = planningStagePool.getCopyForReading();
      List<FootstepPlanningStage> updatedStagePool = planningStagePool.getCopyForWriting();
      updatedStagePool.addAll(currentStagePool);
      updatedStagePool.add(stageFinished);
      planningStagePool.commit();

      stageFinished.destroyStageRunnable();
      planningTasks.remove(stageFinished).cancel(true);
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

   private boolean initialize()
   {
      isDone.set(false);
      requestedPlanarRegions.set(false);
      plannerTime.set(0.0);

      for (FootstepPlanningStage stage : allPlanningStages)
         stage.setTextToSpeechPublisher(textToSpeechPublisher);

      FootstepPlanningRequestPacket request = latestRequestReference.getAndSet(null);
      if (request == null)
         return false;

      planId.set(request.getPlannerRequestId());
      FootstepPlannerType requestedPlannerType = FootstepPlannerType.fromByte(request.getRequestedFootstepPlannerType());

      FootstepPlannerParametersPacket parameters = latestParametersReference.getAndSet(null);
      if (parameters != null)
         footstepPlanningParameters.set(parameters);

      if (debug)
      {
         PrintTools.info("Starting to plan. Plan id: " + request.getPlannerRequestId() + ". Timeout: " + request.getTimeout());
      }

      if (requestedPlannerType != null)
      {
         activePlanner.set(requestedPlannerType);
      }

      PlanarRegionsListMessage planarRegionsListMessage = request.getPlanarRegionsListMessage();
      if (planarRegionsListMessage == null)
      {
         this.planarRegionsList = Optional.empty();
      }
      else
      {
         PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage);
         this.planarRegionsList = Optional.of(planarRegionsList);
      }

      FootstepPlannerObjective plannerGoal = new FootstepPlannerObjective();

      FramePose3D initialStancePose = new FramePose3D(ReferenceFrame.getWorldFrame());
      initialStancePose.setPosition(new Point3D(request.getStanceFootPositionInWorld()));
      initialStancePose.setOrientation(new Quaternion(request.getStanceFootOrientationInWorld()));
      plannerGoal.setInitialStanceFootPose(initialStancePose);

      FramePose3D goalPose = new FramePose3D(ReferenceFrame.getWorldFrame());
      goalPose.setPosition(new Point3D(request.getGoalPositionInWorld()));
      goalPose.setOrientation(new Quaternion(request.getGoalOrientationInWorld()));

      plannerGoal.setInitialStanceFootSide(RobotSide.fromByte(request.getInitialStanceRobotSide()));

      FootstepPlannerGoal goal = new FootstepPlannerGoal();
      goal.setFootstepPlannerGoalType(FootstepPlannerGoalType.POSE_BETWEEN_FEET);
      goal.setGoalPoseBetweenFeet(goalPose);
      plannerGoal.setGoal(goal);

      double horizonLength = request.getHorizonLength();
      if (horizonLength > 0 && Double.isFinite(horizonLength))
         plannerGoal.setHorizonLength(horizonLength);

      double timeout = request.getTimeout();
      if (timeout > 0.0 && Double.isFinite(timeout))
      {
         plannerGoal.setTimeout(timeout);
         this.timeout.set(timeout);

         if (debug)
         {
            PrintTools.info("Setting timeout to " + timeout);
         }
      }
      else
      {
         plannerGoal.setTimeout(Double.POSITIVE_INFINITY);
      }

      planningObjectivePool.add(plannerGoal);

      assignGoalsToAvailablePlanners();

      return true;
   }

   private void updateInternal()
   {
      plannerTime.add(Conversions.millisecondsToSeconds(tickDurationMs));
      if (plannerTime.getDoubleValue() > 1000.0)
      {
         if (debug)
            PrintTools.info("Hard timeout at " + plannerTime.getDoubleValue());
         reportPlannerFailed(FootstepPlanningResult.TIMED_OUT_BEFORE_SOLUTION);
         return;
      }

      updatePlanningObjectives();

      // check if there are any more goals, and assign them to the available planners
      assignGoalsToAvailablePlanners();

      // check the status of the path planners
      FootstepPlanningResult pathStatus = getWorstResult(completedPathResults.getCopyForReading());
      if (!pathStatus.validForExecution())
      {
         reportPlannerFailed(pathStatus);
         return;
      }

      // path planner hasn't failed, so update the path planner status and send out new plan if finished
      boolean noMorePathsToPlan = pathPlanningStagesInProgress.isEmpty() && planningObjectivePool.isEmpty();
      boolean pathPlanningStatusChanged = isDonePlanningPath.getBooleanValue() != noMorePathsToPlan;
      if (noMorePathsToPlan && pathPlanningStatusChanged) // path planning just finished
      {
         sendMessageToUI("Result of path planning: " + planId.getIntegerValue() + ", " + pathStatus.toString());

         concatenateBodyPathPlans();
         statusOutputManager.reportStatusMessage(packPathResult(bodyPathPlan.get(), pathStatus));
      }
      if (pathPlanningStatusChanged) // path planning either just started or just finished, so this flag needs updating
         isDonePlanningPath.set(noMorePathsToPlan);

      // check the status of the step planners
      FootstepPlanningResult stepStatus = getWorstResult(completedStepResults.getCopyForReading());
      if (!stepStatus.validForExecution())
      {
         reportPlannerFailed(stepStatus);
         return;
      }

      // step planner hasn't failed, so update the step planner status and send out the new plan if finished
      boolean noMoreStepsToPlan = stepPlanningStagesInProgress.isEmpty() && planningObjectivePool.isEmpty();
      boolean stepPlanningStatusChanged = isDonePlanningSteps.getBooleanValue() != noMoreStepsToPlan;
      if (noMorePathsToPlan && stepPlanningStatusChanged) // step planning just finished.
      {
         sendMessageToUI("Result of step planning: " + planId.getIntegerValue() + ", " + stepStatus.toString());
         concatenateFootstepPlans();
         statusOutputManager
               .reportStatusMessage(packStepResult(footstepPlan.getAndSet(null), bodyPathPlan.getAndSet(null), stepStatus, plannerTime.getDoubleValue()));
      }
      if (stepPlanningStatusChanged) // step planning just started or just finished, so this flag needs updating.
         isDonePlanningSteps.set(noMoreStepsToPlan);

      boolean isDone = stepPlanningStagesInProgress.isEmpty();
      isDone &= pathPlanningStagesInProgress.isEmpty();
      isDone &= planningObjectivePool.isEmpty();
      isDone &= !goalRecommendationHandler.hasNewFootstepPlannerObjectives();
      if (isDone)
         this.isDone.set(true);
      else
         this.isDone.set(false);
   }

   private static FootstepPlanningResult getWorstResult(List<FootstepPlanningResult> results)
   {
      FootstepPlanningResult worstResult = FootstepPlanningResult.OPTIMAL_SOLUTION;
      if (results == null) // hasn't been assigned yet.
         return worstResult;

      for (FootstepPlanningResult result : results)
         worstResult = FootstepPlanningResult.getWorstResult(worstResult, result);

      return worstResult;
   }

   private void reportPlannerFailed(FootstepPlanningResult result)
   {
      statusOutputManager.reportStatusMessage(packStepResult(null, null, result, plannerTime.getDoubleValue()));
      statusOutputManager.reportStatusMessage(FootstepPlanningMessageReporter.packStatus(FootstepPlannerStatus.IDLE));
      isDonePlanningSteps.set(true);
      isDonePlanningPath.set(true);
      isDone.set(true);
   }

   private void concatenateBodyPathPlans()
   {
      PairList<Integer, BodyPathPlan> completedPathPlans = this.completedPathPlans.getCopyForReading();

      if (completedPathPlans.isEmpty())
      {
         bodyPathPlan.set(null);
         return;
      }

      completedPathPlans.sort(Comparator.comparingInt(ImmutablePair<Integer, BodyPathPlan>::getLeft));

      BodyPathPlan totalBodyPathPlan = new BodyPathPlan();
      totalBodyPathPlan.setStartPose(completedPathPlans.get(0).getRight().getStartPose());
      totalBodyPathPlan.setGoalPose(completedPathPlans.get(completedPathPlans.size() - 1).getRight().getStartPose());
      for (ImmutablePair<Integer, BodyPathPlan> bodyPathPlan : completedPathPlans)
      {
         for (int i = 0; i < bodyPathPlan.getRight().getNumberOfWaypoints(); i++)
            totalBodyPathPlan.addWaypoint(bodyPathPlan.getRight().getWaypoint(i));
      }

      bodyPathPlan.set(totalBodyPathPlan);
   }

   private void concatenateFootstepPlans()
   {
      PairList<Integer, FootstepPlan> completedStepPlans = this.completedStepPlans.getCopyForReading();

      if (completedStepPlans.isEmpty())
      {
         footstepPlan.set(null);
         return;
      }

      completedStepPlans.sort(Comparator.comparingInt(ImmutablePair<Integer, FootstepPlan>::getLeft));

      FootstepPlan totalFootstepPlan = new FootstepPlan();
      for (ImmutablePair<Integer, FootstepPlan> footstepPlanPairs : completedStepPlans)
      {
         FootstepPlan footstepPlan = footstepPlanPairs.getRight();

         if (footstepPlan.hasLowLevelPlanGoal())
            totalFootstepPlan.setLowLevelPlanGoal(footstepPlan.getLowLevelPlanGoal());

         for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)
            totalFootstepPlan.addFootstep(footstepPlan.getFootstep(i));
      }

      footstepPlan.set(totalFootstepPlan);
   }

   private void sendMessageToUI(String message)
   {
      textToSpeechPublisher.publish(MessageTools.createTextToSpeechPacket(message));
   }

   protected boolean isDone()
   {
      return isDone.getBooleanValue();
   }

   private BodyPathPlanMessage packPathResult(BodyPathPlan bodyPathPlan, FootstepPlanningResult status)
   {
      return FootstepPlanningMessageReporter.packPathResult(bodyPathPlan, status, planarRegionsList, planId.getIntegerValue());
   }

   private FootstepPlanningToolboxOutputStatus packStepResult(FootstepPlan footstepPlan, BodyPathPlan bodyPathPlan, FootstepPlanningResult status,
                                                              double timeTaken)
   {
      return FootstepPlanningMessageReporter.packStepResult(footstepPlan, bodyPathPlan, status, timeTaken, planarRegionsList, planId.getIntegerValue());
   }

   public void processRequest(FootstepPlanningRequestPacket request)
   {
      latestRequestReference.set(request);
   }

   public void processPlannerParameters(FootstepPlannerParametersPacket parameters)
   {
      latestParametersReference.set(parameters);
   }

   public void processPlanningStatisticsRequest()
   {
      /*
      FootstepPlanner planner = pathPlanningStagesInProgress.get(0);
      sendPlannerStatistics(planner.getPlannerStatistics());
      */
   }

   public void setTextToSpeechPublisher(IHMCRealtimeROS2Publisher<TextToSpeechPacket> textToSpeechPublisher)
   {
      this.textToSpeechPublisher = textToSpeechPublisher;
   }

   private void sendPlannerStatistics(PlannerStatistics plannerStatistics)
   {
      switch (plannerStatistics.getStatisticsType())
      {
      case LIST:
         sendListOfStatistics((ListOfStatistics) plannerStatistics);
         break;
      case VISIBILITY_GRAPH:
         statusOutputManager.reportStatusMessage(VisibilityGraphMessagesConverter.convertToBodyPathPlanStatisticsMessage(planId.getIntegerValue(),
                                                                                                                         (VisibilityGraphStatistics) plannerStatistics));
         break;
      }
   }

   private void sendListOfStatistics(ListOfStatistics listOfStatistics)
   {
      while (listOfStatistics.getNumberOfStatistics() > 0)
         sendPlannerStatistics(listOfStatistics.pollStatistics());
   }

   public void wakeUp()
   {
      if (debug)
         PrintTools.debug(this, "Waking up");

      initialize.set(true);
   }

   public void sleep()
   {
      if (debug)
         PrintTools.debug(this, "Going to sleep");

      cleanupAllPlanningStages();

      for (FootstepPlanningStage planningTask : planningTasks.keySet())
         planningTasks.remove(planningTask).cancel(true);
   }

   public void destroy()
   {
      sleep();

      if (debug)
         PrintTools.debug(this, "Destroyed");
   }

   private class ConcurrentList<T> extends ConcurrentCopier<List<T>>
   {
      public ConcurrentList()
      {
         super(ArrayList<T>::new);
      }

      public boolean isEmpty()
      {
         this.getCopyForReading().isEmpty();
      }

      public void add(T objectToAdd)
      {
         List<T> existingList = this.getCopyForReading();
         List<T> updatedList = this.getCopyForWriting();
         updatedList.addAll(existingList);
         updatedList.add(objectToAdd);

         this.commit();
      }

      public T remove(int indexToRemove)
      {
         List<T> existingList = this.getCopyForReading();
         List<T> updatedList = this.getCopyForWriting();
         updatedList.addAll(existingList);
         T objectToReturn = updatedList.remove(indexToRemove);

         this.commit();

         return objectToReturn;
      }
   }

   private class ConcurrentMap<K, V> extends ConcurrentCopier<Map<K, V>>
   {
      public ConcurrentMap()
      {
         super(HashMap<K, V>::new);
      }

      public boolean isEmpty()
      {
         return this.getCopyForReading().isEmpty();
      }

      public void put(K key, V value)
      {
         Map<K, V> existingMap = this.getCopyForReading();
         Map<K, V> updatedMap = this.getCopyForWriting();
         updatedMap.putAll(existingMap);
         updatedMap.put(key, value);

         this.commit();
      }

      public V remove(K key)
      {
         Map<K, V> existingMap = this.getCopyForReading();
         Map<K, V> updatedMap = this.getCopyForWriting();
         updatedMap.putAll(existingMap);
         updatedMap.remove(key, value);

         this.commit();
      }
   }

}
