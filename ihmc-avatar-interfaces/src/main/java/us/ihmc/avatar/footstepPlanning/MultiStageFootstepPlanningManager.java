package us.ihmc.avatar.footstepPlanning;

import controller_msgs.msg.dds.*;
import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ihmc.affinity.CPUTopology;
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
import us.ihmc.pathPlanning.statistics.StatisticsType;
import us.ihmc.pathPlanning.statistics.VisibilityGraphStatistics;
import us.ihmc.pathPlanning.visibilityGraphs.tools.BodyPathPlan;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.graphics.YoGraphicPlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.lists.PairList;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

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

   private final ConcurrentList<FootstepPlanningStage> planningStagePool = new ConcurrentList<>();
   private final ConcurrentList<FootstepPlanningStage> allPlanningStages = new ConcurrentList<>();
   private final ConcurrentMap<FootstepPlanningStage, FootstepPlannerObjective> pathPlanningStagesInProgress = new ConcurrentMap<>();
   private final ConcurrentMap<FootstepPlanningStage, FootstepPlannerObjective> stepPlanningStagesInProgress = new ConcurrentMap<>();

   private final ConcurrentMap<FootstepPlanningStage, ScheduledFuture<?>> planningTasks = new ConcurrentMap<>();

   private final YoBoolean isDonePlanningPath = new YoBoolean("isDonePlanningPath", registry);
   private final YoBoolean isDonePlanningSteps = new YoBoolean("isDonePlanningSteps", registry);

   private final ConcurrentList<FootstepPlanningResult> completedPathResults = new ConcurrentList<>();
   private final ConcurrentList<FootstepPlanningResult> completedStepResults = new ConcurrentList<>();

   private final ConcurrentPairList<Integer, PlannerStatistics<?>> completedPlanStatistics = new ConcurrentPairList<>();

   private final ConcurrentPairList<Integer, BodyPathPlan> completedPathPlans = new ConcurrentPairList<>();
   private final ConcurrentPairList<Integer, FootstepPlan> completedStepPlans = new ConcurrentPairList<>();

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

      CPUTopology topology = new CPUTopology();
      int numberOfCores = topology.getNumberOfCores();

      ThreadFactory threadFactory = ThreadTools.getNamedThreadFactory(getClass().getSimpleName());
      executorService = Executors.newScheduledThreadPool(numberOfCores, threadFactory);

      footstepPlanningParameters = new YoFootstepPlannerParameters(registry, footstepPlannerParameters);

      activePlanner.set(FootstepPlannerType.PLANAR_REGION_BIPEDAL);

      graphicsListRegistry.registerYoGraphic("footstepPlanning", yoGraphicPlanarRegionsList);
      isDone.set(false);
      planId.set(FootstepPlanningRequestPacket.NO_PLAN_ID);

      for (int i = 0; i < numberOfCores; i++)
      {
         FootstepPlanningStage planningStage = new FootstepPlanningStage(i, contactPointParameters, footstepPlannerParameters, activePlanner, planId,
                                                                         graphicsListRegistry, tickDurationMs);
         planningStage.addCompletionCallback(this);
         planningStage.setPlannerGoalRecommendationHandler(goalRecommendationHandler);
         registry.addChild(planningStage.getYoVariableRegistry());
         allPlanningStages.add(planningStage);
         planningStagePool.add(planningStage);
      }

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
      FootstepPlanningStage stage = new FootstepPlanningStage(allPlanningStages.getCopyForReading().size(), contactPointParameters, footstepPlanningParameters,
                                                              activePlanner, planId, null, tickDurationMs);
      stage.addCompletionCallback(this);
      stage.setPlannerGoalRecommendationHandler(goalRecommendationHandler);
      return stage;
   }

   private FootstepPlanningStage spawnNextAvailablePlanner()
   {
      FootstepPlanningStage stageToReturn;

      if (planningStagePool.isEmpty())
      { // create a new stage
         stageToReturn = createNewFootstepPlanningStage();
         allPlanningStages.add(stageToReturn);
      }
      else
      { // get one from the pool
         stageToReturn = planningStagePool.remove(0);
      }

      return stageToReturn;
   }

   private FootstepPlanningStage cleanupPlanningStage(FootstepPlanningStage planningStage)
   {
      stepPlanningStagesInProgress.remove(planningStage);
      planningStage.requestInitialize();
      planningStage.destroyStageRunnable();
      planningTasks.remove(planningStage).cancel(true);
      planningStagePool.add(planningStage);

      return planningStage;
   }

   private void cleanupAllPlanningStages()
   {
      globalSequenceIndex.set(0);
      pathPlanningStagesInProgress.clear();

      for (FootstepPlanningStage stepPlanningStageInProgress : stepPlanningStagesInProgress.iterator())
      {
         planningStagePool.add(cleanupPlanningStage(stepPlanningStageInProgress));
      }

      completedPathPlans.clear();
      completedStepPlans.clear();

      completedPlanStatistics.clear();

      completedPathResults.clear();
      completedStepResults.clear();

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
         planner.requestInitialize();

         Runnable runnable = planner.createStageRunnable();
         ScheduledFuture<?> plannerTask = executorService.schedule(runnable, 0, TimeUnit.MILLISECONDS);
         planningTasks.put(planner, plannerTask);

         pathPlanningStagesInProgress.put(planner, plannerGoal);
         stepPlanningStagesInProgress.put(planner, plannerGoal);

         if (debug)
            PrintTools.info("Just started up planning objective " + globalSequenceIndex.getIntegerValue() + " on stage " + planner.getStageId());
      }
   }

   private void updatePlanningObjectives()
   {
      while (goalRecommendationHandler.hasNewFootstepPlannerObjectives())
         planningObjectivePool.add(goalRecommendationHandler.pollNextFootstepPlannerObjective());
   }

   @Override
   public void pathPlanningIsComplete(FootstepPlanningResult pathPlanningResult, FootstepPlanningStage stageFinished)
   {
      completedPathResults.add(pathPlanningResult);

      BodyPathPlan bodyPathPlan = stageFinished.getPathPlan();

      if (pathPlanningResult.validForExecution() && bodyPathPlan != null)
         completedPathPlans.add(stageFinished.getPlanSequenceId(), bodyPathPlan);

      FootstepPlannerObjective objective = pathPlanningStagesInProgress.remove(stageFinished);

      if (debug)
         PrintTools.info("Stage " + stageFinished.getStageId() + " just finished planning its path.");
   }

   @Override
   public void stepPlanningIsComplete(FootstepPlanningResult stepPlanningResult, FootstepPlanningStage stageFinished)
   {
      completedStepResults.add(stepPlanningResult);

      if (stepPlanningResult.validForExecution())
         completedStepPlans.add(stageFinished.getPlanSequenceId(), stageFinished.getPlan());

      completedPlanStatistics.add(stageFinished.getPlanSequenceId(), stageFinished.getPlannerStatistics());

      planningStagePool.add(cleanupPlanningStage(stageFinished));

      if (debug)
         PrintTools.info("Stage " + stageFinished.getStageId() + " just finished planning its steps.");
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

      for (FootstepPlanningStage stage : planningTasks.iterator())
      {
         ScheduledFuture<?> task = planningTasks.getCopyForReading().get(stage);

         if (task.isDone())
         {
            try
            {
               task.get();
            }
            catch (ExecutionException exception)
            {
               exception.getCause().printStackTrace();
               PrintTools.info(exception.getMessage());
               exception.printStackTrace();
            }
            catch (InterruptedException ex)
            {
               ex.getCause().printStackTrace();
            }

            throw new RuntimeException("Something killed stage " + stage.getStageId() + " before it could complete.");
         }
      }
   }

   private boolean initialize()
   {
      isDone.set(false);
      requestedPlanarRegions.set(false);
      plannerTime.set(0.0);

      for (FootstepPlanningStage stage : allPlanningStages.iterable())
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
      this.isDone.set(isDone);
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
      if (completedPathPlans.isEmpty())
      {
         bodyPathPlan.set(null);
         return;
      }

      PairList<Integer, BodyPathPlan> completedPathPlans = this.completedPathPlans.getCopyForReading();
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
      if (completedStepPlans.isEmpty())
      {
         footstepPlan.set(null);
         return;
      }

      PairList<Integer, FootstepPlan> completedStepPlans = this.completedStepPlans.getCopyForReading();
      completedStepPlans.sort(Comparator.comparingInt(ImmutablePair<Integer, FootstepPlan>::getLeft));

      int firstPlan = Integer.MAX_VALUE;
      FootstepPlan totalFootstepPlan = new FootstepPlan();
      for (ImmutablePair<Integer, FootstepPlan> footstepPlanPairs : completedStepPlans)
      {
         FootstepPlan footstepPlan = footstepPlanPairs.getRight();
         int number = footstepPlanPairs.getLeft();

         if (footstepPlan.hasLowLevelPlanGoal() && number < firstPlan)
         {
            totalFootstepPlan.setLowLevelPlanGoal(footstepPlan.getLowLevelPlanGoal());
            firstPlan = number;
         }

         totalFootstepPlan.addFootstepPlan(footstepPlan);
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
      EnumMap<StatisticsType, PlannerStatistics<?>> mapToPopulate = new EnumMap<>(StatisticsType.class);
      for (ImmutablePair<Integer, PlannerStatistics<?>> pair : completedPlanStatistics.iterable())
         concatenateStatistics(mapToPopulate, pair.getLeft(), pair.getRight());

      ListOfStatistics statistics = convertToListOfStatistics(mapToPopulate);
      sendPlannerStatistics(statistics);
   }

   private void concatenateStatistics(EnumMap<StatisticsType, PlannerStatistics<?>> mapToPopulate, int segmentId, PlannerStatistics<?> plannerStatistics)
   {
      switch (plannerStatistics.getStatisticsType())
      {
      case LIST:
         concatenateListOfStatistics(mapToPopulate, segmentId, (ListOfStatistics) plannerStatistics);
         break;
      case VISIBILITY_GRAPH:
         VisibilityGraphStatistics incomingStatistics = (VisibilityGraphStatistics) plannerStatistics;
         VisibilityGraphStatistics statistics;
         if (mapToPopulate.containsKey(StatisticsType.VISIBILITY_GRAPH))
            statistics = (VisibilityGraphStatistics) mapToPopulate.get(StatisticsType.VISIBILITY_GRAPH);
         else
         {
            statistics = new VisibilityGraphStatistics();
            mapToPopulate.put(StatisticsType.VISIBILITY_GRAPH, statistics);
         }

         if (segmentId == 1)
            statistics.setStartMapId(incomingStatistics.getStartMapId());
         if (segmentId == globalSequenceIndex.getIntegerValue())
            statistics.setGoalMapId(incomingStatistics.getGoalMapId());

         statistics.getStartVisibilityMap().addConnections(incomingStatistics.getStartVisibilityMap().getConnections());
         statistics.getGoalVisibilityMap().addConnections(incomingStatistics.getGoalVisibilityMap().getConnections());
         statistics.getInterRegionsVisibilityMap().addConnections(incomingStatistics.getInterRegionsVisibilityMap().getConnections());
         for (int i = 0; i < incomingStatistics.getNumberOfNavigableRegions(); i++)
            statistics.addNavigableRegion(incomingStatistics.getNavigableRegion(i));

         break;
      }
   }

   private void concatenateListOfStatistics(EnumMap<StatisticsType, PlannerStatistics<?>> mapToPopulate, int segmentId, ListOfStatistics listOfStatistics)
   {
      while (listOfStatistics.getNumberOfStatistics() > 0)
         concatenateStatistics(mapToPopulate, segmentId, listOfStatistics.pollStatistics());
   }

   private ListOfStatistics convertToListOfStatistics(EnumMap<StatisticsType, PlannerStatistics<?>> statisticsMap)
   {
      if (statisticsMap.containsKey(StatisticsType.LIST))
         throw new IllegalArgumentException("Statistics haven't been unpacked properly");

      ListOfStatistics statistics = new ListOfStatistics();
      for (StatisticsType type : statisticsMap.keySet())
         statistics.addStatistics(statisticsMap.get(type));

      return statistics;
   }

   private void sendPlannerStatistics(PlannerStatistics<?> plannerStatistics)
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

   public void setTextToSpeechPublisher(IHMCRealtimeROS2Publisher<TextToSpeechPacket> textToSpeechPublisher)
   {
      this.textToSpeechPublisher = textToSpeechPublisher;
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

      for (FootstepPlanningStage planningTask : planningTasks.iterator())
         planningTasks.remove(planningTask).cancel(true);
   }

   public void destroy()
   {
      sleep();

      if (debug)
         PrintTools.debug(this, "Destroyed");
   }

   class ConcurrentList<T> extends ConcurrentCopier<List<T>>
   {
      public ConcurrentList()
      {
         super(ArrayList::new);
      }

      public boolean isEmpty()
      {
         List<T> readCopy = this.getCopyForReading();
         if (readCopy == null)
            return true;
         else
            return readCopy.isEmpty();
      }

      public void clear()
      {
         this.getCopyForWriting();
         this.commit();
      }

      public void add(T objectToAdd)
      {
         List<T> existingList = this.getCopyForReading();
         List<T> updatedList = this.getCopyForWriting();
         updatedList.clear();
         if (existingList != null)
            updatedList.addAll(existingList);
         updatedList.add(objectToAdd);

         this.commit();
      }

      public T remove(int indexToRemove)
      {
         List<T> existingList = this.getCopyForReading();
         List<T> updatedList = this.getCopyForWriting();
         updatedList.clear();
         if (existingList != null)
            updatedList.addAll(existingList);
         T objectToReturn = updatedList.remove(indexToRemove);

         this.commit();

         return objectToReturn;
      }

      public Iterable<T> iterable()
      {
         return this.getCopyForReading();
      }
   }

   class ConcurrentPairList<L, R> extends ConcurrentCopier<PairList<L, R>>
   {
      public ConcurrentPairList()
      {
         super(PairList::new);
      }

      public boolean isEmpty()
      {
         PairList<L, R> readCopy = this.getCopyForReading();
         if (readCopy == null)
            return true;
         else
            return readCopy.isEmpty();
      }

      public void clear()
      {
         this.getCopyForWriting();
         this.commit();
      }

      public void add(L leftObjectToAdd, R rightObjectToAdd)
      {
         PairList<L, R> existingList = this.getCopyForReading();
         PairList<L, R> updatedList = this.getCopyForWriting();
         updatedList.clear();
         if (existingList != null)
            updatedList.addAll(existingList);
         updatedList.add(new ImmutablePair<>(leftObjectToAdd, rightObjectToAdd));

         this.commit();
      }

      public ImmutablePair<L, R> remove(int indexToRemove)
      {
         PairList<L, R> existingList = this.getCopyForReading();
         PairList<L, R> updatedList = this.getCopyForWriting();
         updatedList.clear();
         if (existingList != null)
            updatedList.addAll(existingList);
         ImmutablePair<L, R> objectToReturn = updatedList.remove(indexToRemove);

         this.commit();

         return objectToReturn;
      }

      public Iterable<ImmutablePair<L, R>> iterable()
      {
         return this.getCopyForReading();
      }
   }

   class ConcurrentMap<K, V> extends ConcurrentCopier<HashMap<K, V>>
   {
      public ConcurrentMap()
      {
         super(HashMap::new);
      }

      public boolean isEmpty()
      {
         return this.getCopyForReading().isEmpty();
      }

      public Iterable<K> iterator()
      {
         return this.getCopyForReading().keySet();
      }

      public void clear()
      {
         this.getCopyForWriting();
         this.commit();
      }

      public void put(K key, V value)
      {
         Map<K, V> existingMap = this.getCopyForReading();
         Map<K, V> updatedMap = this.getCopyForWriting();
         updatedMap.clear();
         if (existingMap != null)
            updatedMap.putAll(existingMap);
         updatedMap.put(key, value);

         this.commit();
      }

      public V remove(K key)
      {
         Map<K, V> existingMap = this.getCopyForReading();
         Map<K, V> updatedMap = this.getCopyForWriting();
         updatedMap.clear();
         if (existingMap != null)
            updatedMap.putAll(existingMap);
         V objectToReturn = updatedMap.remove(key);

         this.commit();

         return objectToReturn;
      }
   }
}
