package us.ihmc.avatar.footstepPlanning;

import controller_msgs.msg.dds.*;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.YoFootstepPlannerParameters;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessageConverter;
import us.ihmc.pathPlanning.statistics.ListOfStatistics;
import us.ihmc.pathPlanning.statistics.PlannerStatistics;
import us.ihmc.pathPlanning.statistics.VisibilityGraphStatistics;
import us.ihmc.pathPlanning.visibilityGraphs.tools.BodyPathPlan;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.graphics.YoGraphicPlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

public class MultiStageFootstepPlanningManager
{
   private static final boolean debug = false;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry graphicsListRegistry;

   private final YoEnum<FootstepPlannerType> activePlanner = new YoEnum<>("activePlanner", registry, FootstepPlannerType.class);

   private final AtomicReference<FootstepPlanningRequestPacket> latestRequestReference = new AtomicReference<>(null);
   private final AtomicReference<FootstepPlannerParametersPacket> latestParametersReference = new AtomicReference<>(null);
   private Optional<PlanarRegionsList> planarRegionsList = Optional.empty();

   private final YoBoolean isDone = new YoBoolean("isDone", registry);
   private final YoBoolean requestedPlanarRegions = new YoBoolean("RequestedPlanarRegions", registry);
   private final YoDouble toolboxTime = new YoDouble("ToolboxTime", registry);
   private final YoDouble timeout = new YoDouble("ToolboxTimeout", registry);
   private final YoInteger planId = new YoInteger("planId", registry);

   private final YoGraphicPlanarRegionsList yoGraphicPlanarRegionsList;

   private final List<FootstepPlanningStage> allPlanningStages = new ArrayList<>();
   private final List<FootstepPlanningStage> availablePlanningStages = new ArrayList<>();
   private final List<FootstepPlanningStage> planningStagesInProgress = new ArrayList<>();

   private final List<ScheduledFuture<?>> scheduledStages = new ArrayList<>();

   private final List<FootstepPlanningResult> completedPathResults = new ArrayList<>();
   private final List<FootstepPlanningResult> completedStepResults = new ArrayList<>();

   private final List<BodyPathPlan> completedPathPlans = new ArrayList<>();
   private final List<FootstepPlan> completedFootstepPlans = new ArrayList<>();

   private double dt;

   private final YoFootstepPlannerParameters footstepPlanningParameters;
   private IHMCRealtimeROS2Publisher<TextToSpeechPacket> textToSpeechPublisher;

   private final RobotContactPointParameters<RobotSide> contactPointParameters;

   private final StatusMessageOutputManager statusOutputManager;
   private final ScheduledExecutorService executorService;
   private final YoBoolean initialize = new YoBoolean("initialize" + registry.getName(), registry);

   public MultiStageFootstepPlanningManager(RobotContactPointParameters<RobotSide> contactPointParameters,
                                            FootstepPlannerParameters footstepPlannerParameters, StatusMessageOutputManager statusOutputManager,
                                            ScheduledExecutorService executorService, YoVariableRegistry parentRegistry,
                                            YoGraphicsListRegistry graphicsListRegistry, double dt)
   {
      this.contactPointParameters = contactPointParameters;
      this.statusOutputManager = statusOutputManager;
      this.dt = dt;
      this.executorService = executorService;
      this.graphicsListRegistry = graphicsListRegistry;
      this.yoGraphicPlanarRegionsList = new YoGraphicPlanarRegionsList("FootstepPlannerToolboxPlanarRegions", 200, 30, registry);

      footstepPlanningParameters = new YoFootstepPlannerParameters(registry, footstepPlannerParameters);

      activePlanner.set(FootstepPlannerType.PLANAR_REGION_BIPEDAL);

      graphicsListRegistry.registerYoGraphic("footstepPlanningToolbox", yoGraphicPlanarRegionsList);
      isDone.set(true);
      planId.set(FootstepPlanningRequestPacket.NO_PLAN_ID);

      parentRegistry.addChild(registry);
      requestInitialize();
   }

   private FootstepPlanningStage createFootstepPlanningStage()
   {
      return new FootstepPlanningStage(contactPointParameters, footstepPlanningParameters, activePlanner, registry, graphicsListRegistry, dt);
   }

   private FootstepPlannerStatusMessage packStatus(FootstepPlannerStatus status)
   {
      FootstepPlannerStatusMessage message = new FootstepPlannerStatusMessage();
      message.setFootstepPlannerStatus(status.toByte());

      return message;
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

   private void createPlanningStage()
   {
      if (!planningStagesInProgress.isEmpty())
      {
         if (debug)
            PrintTools.error(this, "toolboxRunnable is not null.");
         return;
      }

      FootstepPlanningStage stage = createFootstepPlanningStage();
      planningStagesInProgress.add(stage);
      allPlanningStages.add(stage);
   }

   private void updateStageStatus()
   {
      for (FootstepPlanningStage stage : planningStagesInProgress)
      {
         if (stage.donePlanningSteps())
         {
            completedPathPlans.add(stage.getPathPlan());
            completedFootstepPlans.add(stage.getPlan());

            availablePlanningStages.add(stage);
            planningStagesInProgress.remove(stage);
         }
      }
   }

   /**
    * Initializes once and run the {@link #updateInternal()} of this toolbox controller only if the
    * initialization succeeded.
    *
    * @see #hasBeenInitialized() The initialization state of this toolbox controller.
    */
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

   /**
    * Internal initialization method for preparing this toolbox controller before running
    * {@link #updateInternal()}.
    *
    * @return {@code true} if the initialization succeeded, {@code false} otherwise.
    */
   private boolean initialize()
   {
      isDone.set(false);
      requestedPlanarRegions.set(false);
      toolboxTime.set(0.0);

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

      FramePose3D initialStancePose = new FramePose3D(ReferenceFrame.getWorldFrame());
      initialStancePose.setPosition(new Point3D(request.getStanceFootPositionInWorld()));
      initialStancePose.setOrientation(new Quaternion(request.getStanceFootOrientationInWorld()));

      FramePose3D goalPose = new FramePose3D(ReferenceFrame.getWorldFrame());
      goalPose.setPosition(new Point3D(request.getGoalPositionInWorld()));
      goalPose.setOrientation(new Quaternion(request.getGoalOrientationInWorld()));

      FootstepPlanner planner = planningStagesInProgress.get(0);
      planner.setInitialStanceFoot(initialStancePose, RobotSide.fromByte(request.getInitialStanceRobotSide()));

      FootstepPlannerGoal goal = new FootstepPlannerGoal();
      goal.setFootstepPlannerGoalType(FootstepPlannerGoalType.POSE_BETWEEN_FEET);
      goal.setGoalPoseBetweenFeet(goalPose);
      planner.setGoal(goal);

      double horizonLength = request.getHorizonLength();
      if (horizonLength > 0 && Double.isFinite(horizonLength))
         planner.setPlanningHorizonLength(horizonLength);

      double timeout = request.getTimeout();
      if (timeout > 0.0 && Double.isFinite(timeout))
      {
         planner.setTimeout(timeout);
         this.timeout.set(timeout);

         if (debug)
         {
            PrintTools.info("Setting timeout to " + timeout);
         }
      }
      else
      {
         planner.setTimeout(Double.POSITIVE_INFINITY);
      }

      return true;
   }

   /**
    * Internal update method that should perform the computation for this toolbox controller. It is
    * called only if {@link #initialize()} has succeeded.
    */
   private void updateInternal()
   {
      toolboxTime.add(dt);
      if (toolboxTime.getDoubleValue() > 20.0)
      {
         if (debug)
            PrintTools.info("Hard timeout at " + toolboxTime.getDoubleValue());
         statusOutputManager.reportStatusMessage(packStepResult(null, null, FootstepPlanningResult.TIMED_OUT_BEFORE_SOLUTION, -1.0));
         isDone.set(true);
         return;
      }

      FootstepPlanner planner = planningStagesInProgress.get(0);

      if (planarRegionsList.isPresent())
      {
         planner.setPlanarRegions(planarRegionsList.get());
         yoGraphicPlanarRegionsList.submitPlanarRegionsListToRender(planarRegionsList.get());
         yoGraphicPlanarRegionsList.processPlanarRegionsListQueue();
      }
      else
      {
         planner.setPlanarRegions(null);
         yoGraphicPlanarRegionsList.clear();
      }

      sendMessageToUI("Starting To Plan: " + planId.getIntegerValue() + ", " + activePlanner.getEnumValue().toString());

      statusOutputManager.reportStatusMessage(packStatus(FootstepPlannerStatus.PLANNING_PATH));

      FootstepPlanningResult status = planner.planPath();

      BodyPathPlan bodyPathPlan = null;
      if (status.validForExecution())
      {
         bodyPathPlan = planner.getPathPlan();
         statusOutputManager.reportStatusMessage(packStatus(FootstepPlannerStatus.PLANNING_STEPS));
         statusOutputManager.reportStatusMessage(packPathResult(bodyPathPlan, status));

         status = planner.plan();
      }

      FootstepPlan footstepPlan = planner.getPlan();

      sendMessageToUI("Result: " + planId.getIntegerValue() + ", " + status.toString());

      statusOutputManager.reportStatusMessage(packStepResult(footstepPlan, bodyPathPlan, status, planner.getPlanningDuration()));
      statusOutputManager.reportStatusMessage(packStatus(FootstepPlannerStatus.IDLE));

      isDone.set(true);
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
      if (debug)
      {
         PrintTools.info("Finished planning path. Result: " + status);
      }

      BodyPathPlanMessage result = new BodyPathPlanMessage();
      if (bodyPathPlan != null)
      {
         for (int i = 0; i < bodyPathPlan.getNumberOfWaypoints(); i++)
            result.getBodyPath().add().set(bodyPathPlan.getWaypoint(i));

         result.getPathPlannerStartPose().set(bodyPathPlan.getStartPose());
         result.getPathPlannerGoalPose().set(bodyPathPlan.getGoalPose());
      }

      planarRegionsList.ifPresent(regions -> result.getPlanarRegionsList().set(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(regions)));
      result.setPlanId(planId.getIntegerValue());
      result.setFootstepPlanningResult(status.toByte());
      return result;
   }

   private FootstepPlanningToolboxOutputStatus packStepResult(FootstepPlan footstepPlan, BodyPathPlan bodyPathPlan, FootstepPlanningResult status,
                                                              double timeTaken)
   {
      if (debug)
      {
         PrintTools.info("Finished planning. Result: " + status);
      }

      FootstepPlanningToolboxOutputStatus result = new FootstepPlanningToolboxOutputStatus();
      if (footstepPlan == null)
      {
         result.getFootstepDataList().set(new FootstepDataListMessage());
      }
      else
      {
         result.getFootstepDataList().set(FootstepDataMessageConverter.createFootstepDataListFromPlan(footstepPlan, 0.0, 0.0, ExecutionMode.OVERRIDE));

         if (footstepPlan.hasLowLevelPlanGoal())
         {
            result.getLowLevelPlannerGoal().set(footstepPlan.getLowLevelPlanGoal());
         }
      }

      if (bodyPathPlan != null)
      {
         for (int i = 0; i < bodyPathPlan.getNumberOfWaypoints(); i++)
            result.getBodyPath().add().set(bodyPathPlan.getWaypoint(i));
      }

      planarRegionsList.ifPresent(regions -> result.getPlanarRegionsList().set(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(regions)));
      result.setPlanId(planId.getIntegerValue());
      result.setFootstepPlanningResult(status.toByte());
      result.setTimeTaken(timeTaken);
      return result;
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
      FootstepPlanner planner = planningStagesInProgress.get(0);
      sendPlannerStatistics(planner.getPlannerStatistics());
   }

   private void sendPlannerStatistics(PlannerStatistics plannerStatistics)
   {
      switch (plannerStatistics.getStatisticsType())
      {
      case LIST:
         sendListOfStatistics((ListOfStatistics) plannerStatistics);
         break;
      case VISIBILITY_GRAPH:
         reportMessage(VisibilityGraphMessagesConverter
                             .convertToBodyPathPlanStatisticsMessage(planId.getIntegerValue(), (VisibilityGraphStatistics) plannerStatistics));
         break;
      }
   }

   private void sendListOfStatistics(ListOfStatistics listOfStatistics)
   {
      while (listOfStatistics.getNumberOfStatistics() > 0)
         sendPlannerStatistics(listOfStatistics.pollStatistics());
   }

   public void setTextToSpeechPublisher(IHMCRealtimeROS2Publisher<TextToSpeechPacket> publisher)
   {
      textToSpeechPublisher = publisher;
   }

   /**
    * Publishes the given status message. It used for sending the result computed by this toolbox
    * controller.
    *
    * @param statusMessage the message to publish.
    */
   private <S extends Settable<S>> void reportMessage(S statusMessage)
   {
      statusOutputManager.reportStatusMessage(statusMessage);
   }

   public void reinitialize()
   {
      requestInitialize();
   }

   public void wakeUp()
   {
      if (!scheduledStages.isEmpty())
      {
         if (debug)
            PrintTools.error(this, "This planner is already running.");
         return;
      }

      if (debug)
         PrintTools.debug(this, "Waking up");

      createPlanningStage();
      //      scheduledStages
      //            .add(executorService.scheduleAtFixedRate(planningStagesInProgress.get(0), 0, (long) dt, TimeUnit.SECONDS));
      reinitialize();
   }

   public void sleep()
   {
      if (debug)
         PrintTools.debug(this, "Going to sleep");

      planningStagesInProgress.clear();

      if (scheduledStages.isEmpty())
      {
         if (debug)
            PrintTools.error(this, "There are no stages running.");
      }
      else
      {
         while (!scheduledStages.isEmpty())
            scheduledStages.remove(0).cancel(true);
      }
   }

   public void destroy()
   {
      sleep();

      if (debug)
         PrintTools.debug(this, "Destroyed");
   }
}
