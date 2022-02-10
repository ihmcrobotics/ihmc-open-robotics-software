package us.ihmc.behaviors.heightMapNavigation;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.HeightMapMessage;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.BehaviorDefinition;
import us.ihmc.behaviors.BehaviorInterface;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.behaviors.tools.RemoteHumanoidRobotInterface;
import us.ihmc.behaviors.tools.behaviorTree.BehaviorTreeNodeStatus;
import us.ihmc.behaviors.tools.behaviorTree.ResettingNode;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.UUID;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.behaviors.lookAndStep.LookAndStepBehaviorAPI.ROS2_REGIONS_FOR_FOOTSTEP_PLANNING;

public class HeightMapNavigationBehavior extends ResettingNode implements BehaviorInterface
{
   public static final BehaviorDefinition DEFINITION = new BehaviorDefinition("Height Map Navigation",
                                                                              HeightMapNavigationBehavior::new,
                                                                              HeightMapNavigationBehaviorAPI.API);

   private enum State
   {
      WAITING_TO_START,
      PLAN_BODY_PATH,
      FOLLOW_PATH
   }

   private static final double replanDelay = 0.5;
   private static final BehaviorTreeNodeStatus defaultStatus = BehaviorTreeNodeStatus.SUCCESS;
   private final BehaviorHelper helper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final AtomicReference<PlanarRegionsListMessage> planarRegions = new AtomicReference<>();
   private final AtomicReference<HeightMapMessage> heightMapMessage = new AtomicReference<>();
   private final AtomicReference<Pose3D> goalPose;
   private final AtomicReference<State> currentState = new AtomicReference<>();
   private final AtomicBoolean requestStart = new AtomicBoolean();
   private final AtomicBoolean requestStop = new AtomicBoolean();

   private final FootstepPlanningModule planningModule;
   private final FootstepPlannerLogger logger;

   private final Stopwatch stopwatch = new Stopwatch();
   private final AtomicBoolean firstStep = new AtomicBoolean();
   private final AtomicBoolean firstTickInStepState = new AtomicBoolean();
   private RobotSide lastStepSide = RobotSide.LEFT;
   private final Pose3D lastStepPose = new Pose3D();
   private long previousStepMessageId = 0;

   private final FootstepPlannerRequest request = new FootstepPlannerRequest();
   private final FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
   private final RemoteHumanoidRobotInterface remoteHumanoidInterface;
   private final ResettableExceptionHandlingExecutorService executor = MissingThreadTools.newSingleThreadExecutor("RegionsRelay", true);

   public HeightMapNavigationBehavior(BehaviorHelper helper)
   {
      this.helper = helper;
      syncedRobot = helper.newSyncedRobot();
      LogTools.info("Constructing");

      helper.subscribeViaCallback(ROS2_REGIONS_FOR_FOOTSTEP_PLANNING, planarRegions::set);
      ROS2Tools.createCallbackSubscription(helper.getROS2Node(), ROS2Tools.HEIGHT_MAP_OUTPUT, s -> heightMapMessage.set(s.takeNextData()));

      DRCRobotModel robotModel = helper.getRobotModel();
      SideDependentList<ConvexPolygon2D> footPolygons = helper.createFootPolygons();
      VisibilityGraphsParametersBasics visibilityGraphsParameters = robotModel.getVisibilityGraphsParameters();
      FootstepPlannerParametersBasics footstepPlannerParameters = robotModel.getFootstepPlannerParameters("FootLookAndStep");
      SwingPlannerParametersBasics swingPlannerParameters = robotModel.getSwingPlannerParameters("FootLookAndStep");
      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();

      planningModule = new FootstepPlanningModule("Atlas", visibilityGraphsParameters, footstepPlannerParameters, swingPlannerParameters, walkingControllerParameters, footPolygons, null);
      goalPose = helper.getMessager().createInput(HeightMapNavigationBehaviorAPI.GoalPose);
      helper.getMessager().registerTopicListener(HeightMapNavigationBehaviorAPI.RequestStart, s -> requestStart.set(true));
      helper.getMessager().registerTopicListener(HeightMapNavigationBehaviorAPI.RequestStop, s -> requestStop.set(true));
      planningModule.addCustomTerminationCondition((plannerTime, iterations, bestFinalStep, bestSecondToLastStep, bestPathSize) -> bestPathSize > 0);
      remoteHumanoidInterface = helper.getOrCreateRobotInterface();
      logger = new FootstepPlannerLogger(planningModule);

      helper.subscribeViaCallback(ROS2Tools.MAPSENSE_REGIONS, newValue ->
      {
         planarRegions.set(newValue);
         executor.submit(() -> helper.publish(HeightMapNavigationBehaviorAPI.PlanarRegionsForUI, PlanarRegionMessageConverter.convertToPlanarRegionsList(newValue)));
      });
   }

   @Override
   public BehaviorTreeNodeStatus tickInternal()
   {
      if (currentState.get() == State.WAITING_TO_START)
      {
         if (!requestStart.getAndSet(false))
         {
            return defaultStatus;
         }

         if (goalPose.get() == null)
         {
            LogTools.error("Need to set goal before starting.");
            return defaultStatus;
         }
         if (heightMapMessage.get() == null)
         {
            LogTools.error("Height map not received.");
            return defaultStatus;
         }

         requestStop.set(false);
         firstStep.set(true);
         firstTickInStepState.set(true);
         previousStepMessageId = 0;

         currentState.set(State.PLAN_BODY_PATH);
      }

      if (currentState.get() == State.PLAN_BODY_PATH)
      {
         // Plan body path
         setStartFootPosesToCurrent();

         request.setHeightMapMessage(heightMapMessage.get());
         request.setGoalFootPoses(0.2, goalPose.get());
         request.setPlanBodyPath(true);
         request.setPerformAStarSearch(false);

         LogTools.info("Starting plan");
         FootstepPlannerOutput output = planningModule.handleRequest(request);
         LogTools.info("Finished plan");
         logger.logSession();

         if (output.getBodyPathPlanningResult() != BodyPathPlanningResult.FOUND_SOLUTION)
         {
            LogTools.error("Body path failed to find solution");
            currentState.set(State.WAITING_TO_START);
            return defaultStatus;
         }

         request.getBodyPathWaypoints().clear();
         for (int i = 0; i < output.getBodyPath().size(); i++)
         {
            request.getBodyPathWaypoints().add(new Pose3D(output.getBodyPath().get(i)));
         }
      }
      else
      {
         // Execute steps
         if (requestStop.getAndSet(false))
         {
            LogTools.error("Stop requested");
            currentState.set(State.WAITING_TO_START);
            return defaultStatus;
         }
         if (planarRegions.get() == null)
         {
            LogTools.error("No planar regions");
            currentState.set(State.WAITING_TO_START);
            return defaultStatus;
         }

         boolean planAStep = firstStep.getAndSet(false) || stopwatch.lapElapsed() > replanDelay;
         if (!planAStep)
         {
            return defaultStatus;
         }

         request.setPlanBodyPath(false);
         request.setPerformAStarSearch(true);
         request.setHeightMapMessage(null);
         request.setPlanarRegionsList(PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegions.get()));

         RobotSide stepSide = lastStepSide.getOppositeSide();
         request.setRequestedInitialStanceSide(stepSide);

         if (firstStep.get())
         {
            setStartFootPosesToCurrent();
         }
         else
         {
            setStartFootPosesBasedOnLastCommandedStep();
         }

         LogTools.info("Planning step");
         FootstepPlannerOutput output = planningModule.handleRequest(request);
         FootstepPlanningResult result = output.getFootstepPlanningResult();
         LogTools.info(" \t " + result);
         stopwatch.lap();

         if (result != FootstepPlanningResult.FOUND_SOLUTION)
         {
            return defaultStatus;
         }

         ExecutionMode executionMode = firstStep.get() ? ExecutionMode.OVERRIDE : ExecutionMode.QUEUE;
         PlannedFootstep footstep = output.getFootstepPlan().getFootstep(0);
         footstepDataListMessage.getFootstepDataList().clear();
         FootstepDataMessage footstepDataMessage = footstepDataListMessage.getFootstepDataList().add();
         footstepDataMessage.getLocation().set(footstep.getFootstepPose().getPosition());
         footstepDataMessage.getOrientation().set(footstep.getFootstepPose().getOrientation());

         footstepDataListMessage.getQueueingProperties().setExecutionMode(executionMode.toByte());
         long messageId = UUID.randomUUID().getLeastSignificantBits();
         footstepDataListMessage.getQueueingProperties().setMessageId(messageId);
         footstepDataListMessage.getQueueingProperties().setPreviousMessageId(previousStepMessageId);
         remoteHumanoidInterface.requestWalk(footstepDataListMessage);

         firstStep.set(false);
         lastStepSide = stepSide;
         lastStepPose.set(footstep.getFootstepPose());
         previousStepMessageId = messageId;
         stopwatch.lap();
      }

      return defaultStatus;
   }

   private void setStartFootPosesToCurrent()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         FramePose3D foot = new FramePose3D(syncedRobot.getReferenceFrames().getSoleFrame(robotSide));
         foot.changeFrame(ReferenceFrame.getWorldFrame());
         request.setStartFootPose(robotSide, foot);
      }
   }

   private void setStartFootPosesBasedOnLastCommandedStep()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         if (robotSide == lastStepSide)
         {
            request.setStartFootPose(robotSide, lastStepPose);
         }
         else
         {
            FramePose3D foot = new FramePose3D(syncedRobot.getReferenceFrames().getSoleFrame(robotSide));
            foot.changeFrame(ReferenceFrame.getWorldFrame());
            request.setStartFootPose(robotSide, foot);
         }
      }
   }

   @Override
   public void reset()
   {
      currentState.set(State.WAITING_TO_START);
      goalPose.set(null);
   }

   @Override
   public String getName()
   {
      return DEFINITION.getName();
   }
}
