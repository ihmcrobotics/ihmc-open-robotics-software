package us.ihmc.humanoidBehaviors.demo;

import controller_msgs.msg.dds.*;
import std_msgs.msg.dds.Empty;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.humanoidBehaviors.BehaviorModule;
import us.ihmc.humanoidBehaviors.BehaviorRegistry;
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehavior;
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorAPI;
import us.ihmc.humanoidBehaviors.stairs.TraverseStairsBehavior;
import us.ihmc.humanoidBehaviors.stairs.TraverseStairsBehaviorAPI;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorControlModeEnum;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.CurrentBehaviorStatus;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorType;
import us.ihmc.log.LogTools;
import us.ihmc.messager.kryo.KryoMessager;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class BuildingExplorationBehaviorCoordinator
{
   private static final boolean DEBUG = true;
   private static final int UPDATE_RATE_MILLIS = 50;
   private static final double xyProximityToDoorToStopWalking = 1.6;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final ROS2Node ros2Node;
   private final YoEnum<BuildingExplorationStateName> requestedState = new YoEnum<>("requestedState", "", registry, BuildingExplorationStateName.class, true);
   private final StateMachine<BuildingExplorationStateName, State> stateMachine;
   private final KryoMessager kryoMessager;

   private final AtomicBoolean isRunning = new AtomicBoolean();
   private final AtomicBoolean stopRequested = new AtomicBoolean();

   private final AtomicReference<DoorLocationPacket> doorLocationPacket = new AtomicReference<>();
   private final AtomicReference<RobotConfigurationData> robotConfigurationData = new AtomicReference<>();
   private final Point3D bombPosition = new Point3D();
   private final IHMCROS2Publisher<AbortWalkingMessage> abortWalkingPublisher;

   private final ScheduledExecutorService executorService;
   private ScheduledFuture<?> stateMachineTask = null;

   private Consumer<BuildingExplorationStateName> stateChangedCallback = state -> {};
   private Runnable doorDetectedCallback = () -> {};

   private final TeleopState teleopState;
   private final LookAndStepState lookAndStepState;
   private final WalkThroughDoorState walkThroughDoorState;
   private final TraverseStairsState traverseStairsState;

   public BuildingExplorationBehaviorCoordinator(String robotName, DomainFactory.PubSubImplementation pubSubImplementation)
   {
      ros2Node = ROS2Tools.createROS2Node(pubSubImplementation, getClass().getSimpleName());
      executorService = Executors.newSingleThreadScheduledExecutor();

      BehaviorRegistry behaviorRegistry = BehaviorRegistry.of(LookAndStepBehavior.DEFINITION, TraverseStairsBehavior.DEFINITION);
      kryoMessager = KryoMessager.createClient(behaviorRegistry.getMessagerAPI(),
                                           "127.0.0.1",
                                           NetworkPorts.BEHAVIOUR_MODULE_PORT.getPort(),
                                           getClass().getSimpleName(),
                                           UPDATE_RATE_MILLIS);

      ThreadTools.startAsDaemon(() -> ExceptionTools.handle(kryoMessager::startMessager, DefaultExceptionHandler.RUNTIME_EXCEPTION), "KryoConnect");

      teleopState = new TeleopState();
      lookAndStepState = new LookAndStepState(robotName, ros2Node, kryoMessager, bombPosition);
      walkThroughDoorState = new WalkThroughDoorState(robotName, ros2Node);
      traverseStairsState = new TraverseStairsState(ros2Node, kryoMessager, bombPosition, robotConfigurationData::get);

      ROS2Topic<?> objectDetectionTopic = ROS2Tools.OBJECT_DETECTOR_TOOLBOX.withRobot(robotName).withOutput();
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, DoorLocationPacket.class, objectDetectionTopic, s -> doorLocationPacket.set(s.takeNextData()));

      ROS2Topic<?> controllerOutputTopic = ROS2Tools.getControllerOutputTopic(robotName);
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                    RobotConfigurationData.class,
                                                    controllerOutputTopic,
                                                    s -> robotConfigurationData.set(s.takeNextData()));
      abortWalkingPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, AbortWalkingMessage.class, ROS2Tools.getControllerInputTopic(robotName));

      try
      {
         stateMachine = buildStateMachine();
      }
      catch (Exception e)
      {
         throw new RuntimeException(e);
      }

      stateMachine.addStateChangedListener((from, to) -> stateChangedCallback.accept(to));
   }

   public void setBombPosition(Point3DReadOnly bombPosition)
   {
      this.bombPosition.set(bombPosition);
   }

   public void requestState(BuildingExplorationStateName requestedState)
   {
      this.requestedState.set(requestedState);
   }

   public void setStateChangedCallback(Consumer<BuildingExplorationStateName> stateChangedCallback)
   {
      this.stateChangedCallback = stateChangedCallback;
   }

   public void setDebrisDetectedCallback(Runnable debrisDetectedCallback)
   {
      lookAndStepState.debrisDetectedCallback = debrisDetectedCallback;
   }

   public void setDoorDetectedCallback(Runnable doorDetectedCallback)
   {
      this.doorDetectedCallback = doorDetectedCallback;
   }

   public void setStairsDetectedCallback(Runnable stairsDetectedCallback)
   {
      lookAndStepState.stairsDetectedCallback = stairsDetectedCallback;
   }

   public void ignoreDebris()
   {
      lookAndStepState.ignoreDebris();
      requestState(BuildingExplorationStateName.LOOK_AND_STEP);
   }

   public void proceedWithDoorBehavior()
   {
      walkThroughDoorState.proceedWithDoorBehavior();
   }

   public void start()
   {
      if (!isRunning.getAndSet(true))
      {
         LogTools.debug("Starting behavior coordinator");

         doorLocationPacket.set(null);
         robotConfigurationData.set(null);

         stateMachineTask = executorService.scheduleAtFixedRate(this::update, 0, UPDATE_RATE_MILLIS, TimeUnit.MILLISECONDS);
      }
      else
      {
         LogTools.debug("Start called but module is already running");
      }
   }

   public void stop()
   {
      if (isRunning.get())
      {
         LogTools.debug("Stop requested. Shutting down on next update");
         stopRequested.set(true);
      }
   }

   private StateMachine<BuildingExplorationStateName, State> buildStateMachine()
   {
      StateMachineFactory<BuildingExplorationStateName, State> factory = new StateMachineFactory<>(BuildingExplorationStateName.class);

      factory.setNamePrefix("buildingExploration");
      factory.setRegistry(registry);
      factory.buildClock(() -> Conversions.nanosecondsToSeconds(System.nanoTime()));

      factory.addState(BuildingExplorationStateName.TELEOP, teleopState);
      factory.addState(BuildingExplorationStateName.LOOK_AND_STEP, lookAndStepState);
      factory.addState(BuildingExplorationStateName.WALK_THROUGH_DOOR, walkThroughDoorState);
      factory.addState(BuildingExplorationStateName.TRAVERSE_STAIRS, traverseStairsState);

      factory.addDoneTransition(BuildingExplorationStateName.LOOK_AND_STEP, BuildingExplorationStateName.TELEOP);
      factory.addDoneTransition(BuildingExplorationStateName.WALK_THROUGH_DOOR, BuildingExplorationStateName.TELEOP);
      factory.addDoneTransition(BuildingExplorationStateName.TRAVERSE_STAIRS, BuildingExplorationStateName.TELEOP);

      factory.addRequestedTransition(BuildingExplorationStateName.TELEOP, BuildingExplorationStateName.LOOK_AND_STEP, requestedState);
      factory.addRequestedTransition(BuildingExplorationStateName.TELEOP, BuildingExplorationStateName.WALK_THROUGH_DOOR, requestedState);
      factory.addRequestedTransition(BuildingExplorationStateName.TELEOP, BuildingExplorationStateName.TRAVERSE_STAIRS, requestedState);

      factory.addRequestedTransition(BuildingExplorationStateName.LOOK_AND_STEP, BuildingExplorationStateName.TELEOP, requestedState);
      factory.addRequestedTransition(BuildingExplorationStateName.WALK_THROUGH_DOOR, BuildingExplorationStateName.TELEOP, requestedState);
      factory.addRequestedTransition(BuildingExplorationStateName.TRAVERSE_STAIRS, BuildingExplorationStateName.TELEOP, requestedState);

      factory.addTransition(BuildingExplorationStateName.LOOK_AND_STEP, BuildingExplorationStateName.WALK_THROUGH_DOOR, this::transitionToWalkThroughDoor);
      factory.addTransition(BuildingExplorationStateName.LOOK_AND_STEP, BuildingExplorationStateName.TELEOP, t -> lookAndStepState.getDebrisDetected());
      factory.addTransition(BuildingExplorationStateName.LOOK_AND_STEP, BuildingExplorationStateName.TRAVERSE_STAIRS, t -> lookAndStepState.getStairsDetected());

      return factory.build(BuildingExplorationStateName.TELEOP);
   }

   private void update()
   {
      if (stopRequested.getAndSet(false))
      {
         requestState(BuildingExplorationStateName.TELEOP);
         stateMachine.doActionAndTransition();
         abortWalkingPublisher.publish(new AbortWalkingMessage());

         new Thread(() ->
                    {
                       if (stateMachineTask != null)
                       {
                          stateMachineTask.cancel(true);
                          stateMachineTask = null;
                       }

                       isRunning.set(false);
                    }).start();
      }
      else
      {
         stateMachine.doActionAndTransition();
      }
   }

   private static class TeleopState implements State
   {
      @Override
      public void onEntry()
      {
         // do nothing, this is an idle state where the operator sends commands from the VR UI
         if (DEBUG)
         {
            LogTools.info("Entering " + getClass().getSimpleName());
         }
      }

      @Override
      public void doAction(double timeInState)
      {
         // do nothing
      }

      @Override
      public void onExit(double timeInState)
      {
         if (DEBUG)
         {
            LogTools.info("Exiting " + getClass().getSimpleName());
         }
      }
   }

   private boolean transitionToWalkThroughDoor(double timeInState)
   {
      DoorLocationPacket doorLocationPacket = this.doorLocationPacket.get();
      RobotConfigurationData robotConfigurationData = this.robotConfigurationData.get();

      if (doorLocationPacket == null || robotConfigurationData == null)
      {
         return false;
      }

      Point3D doorPosition = doorLocationPacket.getDoorTransformToWorld().getPosition();
      Point3D robotRootJointPosition = new Point3D(robotConfigurationData.getRootTranslation());

      double xyDistanceToDoor = doorPosition.distanceXY(robotRootJointPosition);
      boolean doorDetected = xyDistanceToDoor <= xyProximityToDoorToStopWalking;
      if (doorDetected)
      {
         doorDetectedCallback.run();
      }

      return doorDetected;
   }

   private static class LookAndStepState implements State
   {
      private static final double horizonFromDebrisToStop = 0.8;
      private static final double horizonForStairs = 1.0;
      private static final double heightIncreaseForStairs = 0.55;

      private static final double debrisCheckBodyBoxWidth = 0.3;
      private static final double debrisCheckBodyBoxDepth = 0.8;
      private static final double debrisCheckBodyBoxHeight = 1.5;
      private static final double debrisCheckBodyBoxBaseZ = 0.5;
      private static final int numberOfStepsToIgnoreDebrisAfterClearing = 4;

      private final KryoMessager messager;
      private final IHMCROS2Publisher<Pose3D> goalPublisher;
      private final IHMCROS2Publisher<Empty> resetPublisher;

      private final Point3DReadOnly bombPosition;

      private final FootstepPlannerParametersBasics footstepPlannerParameters;

      private final AtomicReference<List<Pose3D>> bodyPath;
      private final AtomicReference<PlanarRegionsListMessage> planarRegions = new AtomicReference<>();
      private final AtomicReference<RobotConfigurationData> robotConfigurationData = new AtomicReference<>();

      private final AtomicBoolean debrisDetected = new AtomicBoolean();
      private final AtomicBoolean stairsDetected = new AtomicBoolean();

      private final AtomicInteger stepCounter = new AtomicInteger();
      private int numberOfStepsToIgnoreDebris = 0;

      private Runnable debrisDetectedCallback = () -> {};
      private Runnable stairsDetectedCallback = () -> {};

      private Notification bodyPathPlanningStateReached = new Notification();
      private LookAndStepBehavior.State currentState = LookAndStepBehavior.State.RESET;

      public LookAndStepState(String robotName, ROS2Node ros2Node, KryoMessager messager, Point3DReadOnly bombPosition)
      {
         this.messager = messager;
         this.bombPosition = bombPosition;

         goalPublisher = IHMCROS2Publisher.newPose3DPublisher(ros2Node, ROS2Tools.BEHAVIOR_MODULE.withInput().withType(Pose3D.class));
         resetPublisher = ROS2Tools.createPublisher(ros2Node, ROS2Tools.BEHAVIOR_MODULE.withInput().withType(Empty.class));

         this.footstepPlannerParameters = new DefaultFootstepPlannerParameters();
         this.footstepPlannerParameters.setBodyBoxDepth(debrisCheckBodyBoxWidth);
         this.footstepPlannerParameters.setBodyBoxWidth(debrisCheckBodyBoxDepth);
         this.footstepPlannerParameters.setBodyBoxHeight(debrisCheckBodyBoxHeight);
         this.footstepPlannerParameters.setBodyBoxBaseZ(debrisCheckBodyBoxBaseZ);

         bodyPath = messager.createInput(LookAndStepBehaviorAPI.BodyPathPlanForUI);
         ROS2Tools.createCallbackSubscription(ros2Node, ROS2Tools.LIDAR_REA_REGIONS, s -> planarRegions.set(s.takeNextData()));
         ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                       RobotConfigurationData.class,
                                                       ROS2Tools.getControllerOutputTopic(robotName),
                                                       s -> robotConfigurationData.set(s.takeNextData()));
         ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, FootstepStatusMessage.class, ROS2Tools.getControllerOutputTopic(robotName), s ->
         {
            if (s.takeNextData().getFootstepStatus() == FootstepStatusMessage.FOOTSTEP_STATUS_COMPLETED)
            {
               stepCounter.incrementAndGet();
            }
         });
         messager.registerTopicListener(LookAndStepBehaviorAPI.CurrentState, state ->
         {
            currentState = LookAndStepBehavior.State.valueOf(state);
            if (currentState.equals(LookAndStepBehavior.State.BODY_PATH_PLANNING))
            {
               bodyPathPlanningStateReached.set();
            }
         });
      }

      @Override
      public void onEntry()
      {
         resetPublisher.publish(new Empty());

         String behaviorName = LookAndStepBehavior.DEFINITION.getName();
         messager.submitMessage(BehaviorModule.API.BehaviorSelection, behaviorName);
         ThreadTools.sleep(100);

         if (!currentState.equals(LookAndStepBehavior.State.BODY_PATH_PLANNING))
         {
            LogTools.info("Waiting for BODY_PATH_PLANNING state...");
            bodyPathPlanningStateReached.poll(); // clear it to wait for another one
            bodyPathPlanningStateReached.blockingPoll();
         }
         LogTools.info("Look and step is in BODY_PATH_PLANNING state. Proceeding...");

         messager.submitMessage(LookAndStepBehaviorAPI.OperatorReviewEnabled, false);
         ThreadTools.sleep(100);

         goalPublisher.publish(new Pose3D(bombPosition, new Quaternion()));
         ThreadTools.sleep(100);

         planarRegions.set(null);
         bodyPath.set(null);

         debrisDetected.set(false);
         stairsDetected.set(false);
         stepCounter.set(0);

         if (DEBUG)
         {
            LogTools.info("Entering " + getClass().getSimpleName());
         }
      }

      @Override
      public void doAction(double timeInState)
      {
         if (!debrisDetected.get() && (stepCounter.get() > numberOfStepsToIgnoreDebris))
         {
            checkForDebris();
         }
         if (!stairsDetected.get())
         {
            checkForStairs();
         }
      }

      private void ignoreDebris()
      {
         numberOfStepsToIgnoreDebris = numberOfStepsToIgnoreDebrisAfterClearing;
      }

      private void checkForDebris()
      {
         List<Pose3D> bodyPath = this.bodyPath.get();
         if (bodyPath == null)
            return;

         PlanarRegionsListMessage planarRegionsMessage = this.planarRegions.get();
         if (planarRegionsMessage == null)
            return;

         RobotConfigurationData robotConfigurationData = this.robotConfigurationData.get();
         if (robotConfigurationData == null)
            return;

         PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsMessage);
         Pose3D rootPose = new Pose3D(new Point3D(robotConfigurationData.getRootTranslation()), robotConfigurationData.getRootOrientation());

         boolean debrisDetected = PlannerTools.doesPathContainBodyCollisions(rootPose, bodyPath, planarRegionsList, footstepPlannerParameters, horizonFromDebrisToStop);
         if (debrisDetected)
         {
            LogTools.debug("Debris detected");
            this.debrisDetected.set(true);
            debrisDetectedCallback.run();
         }
      }

      private void checkForStairs()
      {
         List<Pose3D> bodyPath = this.bodyPath.get();
         if (bodyPath == null)
            return;

         RobotConfigurationData robotConfigurationData = this.robotConfigurationData.get();
         if (robotConfigurationData == null)
            return;

         Pose3D rootPose = new Pose3D(new Point3D(robotConfigurationData.getRootTranslation()), robotConfigurationData.getRootOrientation());
         Pose3D currentPoseAlongBodyPath = new Pose3D();
         Pose3D extrapolatedPoseAlongBodyPath = new Pose3D();

         PlannerTools.extrapolatePose(rootPose, bodyPath, 0.0, currentPoseAlongBodyPath);
         PlannerTools.extrapolatePose(rootPose, bodyPath, horizonForStairs, extrapolatedPoseAlongBodyPath);

         boolean stairsDetected = Math.abs(currentPoseAlongBodyPath.getZ() - extrapolatedPoseAlongBodyPath.getZ()) >= heightIncreaseForStairs;
         if (stairsDetected)
         {
            LogTools.debug("Stairs detected");
            this.stairsDetected.set(true);
            stairsDetectedCallback.run();
         }
      }

      boolean getDebrisDetected()
      {
         return debrisDetected.get();
      }

      boolean getStairsDetected()
      {
         return stairsDetected.get();
      }

      @Override
      public void onExit(double timeInState)
      {
         if (DEBUG)
         {
            LogTools.info("Exiting " + getClass().getSimpleName());
         }

         numberOfStepsToIgnoreDebris = 0;
         resetPublisher.publish(new Empty());
      }
   }

   private static class WalkThroughDoorState implements State
   {
      final IHMCROS2Publisher<BehaviorControlModePacket> behaviorModePublisher;
      final IHMCROS2Publisher<HumanoidBehaviorTypePacket> behaviorTypePublisher;
      final AtomicBoolean isDone = new AtomicBoolean();
      final AtomicBoolean receivedOperatorConfirmation = new AtomicBoolean();
      final AtomicBoolean hasStartedBehavior = new AtomicBoolean();

      public WalkThroughDoorState(String robotName, ROS2Node ros2Node)
      {
         ROS2Topic<?> inputTopic = ROS2Tools.BEHAVIOR_MODULE.withRobot(robotName).withInput();
         ROS2Topic<?> outputTopic = ROS2Tools.BEHAVIOR_MODULE.withRobot(robotName).withOutput();

         behaviorModePublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, BehaviorControlModePacket.class, inputTopic);
         behaviorTypePublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, HumanoidBehaviorTypePacket.class, inputTopic);

         ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, BehaviorStatusPacket.class, outputTopic, s ->
         {
            BehaviorStatusPacket behaviorStatusPacket = s.takeNextData();
            CurrentBehaviorStatus behaviorStatus = CurrentBehaviorStatus.fromByte(behaviorStatusPacket.getCurrentBehaviorStatus());
            if (behaviorStatus == CurrentBehaviorStatus.NO_BEHAVIOR_RUNNING)
            {
               isDone.set(true);
            }
         });
      }

      @Override
      public void onEntry()
      {
         receivedOperatorConfirmation.set(false);
         hasStartedBehavior.set(false);
         isDone.set(false);

         if (DEBUG)
         {
            LogTools.info("Entering " + getClass().getSimpleName());
         }
      }

      private void startBehavior()
      {
         HumanoidBehaviorTypePacket humanoidBehaviorTypePacket = new HumanoidBehaviorTypePacket();
         humanoidBehaviorTypePacket.setHumanoidBehaviorType(HumanoidBehaviorType.WALK_THROUGH_DOOR.toByte());
         behaviorTypePublisher.publish(humanoidBehaviorTypePacket);
      }

      @Override
      public void doAction(double timeInState)
      {
         if (receivedOperatorConfirmation.get() && !hasStartedBehavior.get())
         {
            startBehavior();
            hasStartedBehavior.set(true);

            if (DEBUG)
            {
               LogTools.info("Sent packet to start " + HumanoidBehaviorType.WALK_THROUGH_DOOR);
            }
         }
      }

      @Override
      public void onExit(double timeInState)
      {
         BehaviorControlModePacket behaviorControlModePacket = new BehaviorControlModePacket();
         behaviorControlModePacket.setBehaviorControlModeEnumRequest(BehaviorControlModeEnum.STOP.toByte());
         behaviorModePublisher.publish(behaviorControlModePacket);

         if (DEBUG)
         {
            LogTools.info("Exiting " + getClass().getSimpleName());
         }
      }

      @Override
      public boolean isDone(double timeInState)
      {
         return isDone.get();
      }

      public void proceedWithDoorBehavior()
      {
         receivedOperatorConfirmation.set(true);
      }
   }

   private static class TraverseStairsState implements State
   {
      private final KryoMessager messager;
      private final IHMCROS2Publisher<Pose3D> goalPublisher;

      private final IHMCROS2Publisher<Empty> startPublisher;
      private final IHMCROS2Publisher<Empty> stopPublisher;

      private final AtomicBoolean isDone = new AtomicBoolean();
      private final Point3DReadOnly bombPosition;
      private final Supplier<RobotConfigurationData> robotConfigurationDataSupplier;

      public TraverseStairsState(ROS2Node ros2Node, KryoMessager messager, Point3DReadOnly bombPosition, Supplier<RobotConfigurationData> robotConfigurationDataSupplier)
      {
         this.messager = messager;

         this.goalPublisher = IHMCROS2Publisher.newPose3DPublisher(ros2Node, TraverseStairsBehaviorAPI.GOAL_INPUT);
         this.startPublisher = ROS2Tools.createPublisher(ros2Node, TraverseStairsBehaviorAPI.START);
         this.stopPublisher = ROS2Tools.createPublisher(ros2Node, TraverseStairsBehaviorAPI.STOP);

         this.bombPosition = bombPosition;
         this.robotConfigurationDataSupplier = robotConfigurationDataSupplier;

         ROS2Tools.createCallbackSubscription(ros2Node, TraverseStairsBehaviorAPI.COMPLETED, s -> isDone.set(true));
      }

      @Override
      public void onEntry()
      {
         if (DEBUG)
         {
            LogTools.info("Entering " + getClass().getSimpleName());
         }

         isDone.set(false);

         String behaviorName = TraverseStairsBehavior.DEFINITION.getName();
         messager.submitMessage(BehaviorModule.API.BehaviorSelection, behaviorName);
         ThreadTools.sleep(100);

         Quaternion goalOrientation = new Quaternion();

         if (robotConfigurationDataSupplier.get() != null)
         {
            Vector3D rootTranslation = robotConfigurationDataSupplier.get().getRootTranslation();
            double dx = bombPosition.getX() - rootTranslation.getX();
            double dy = bombPosition.getY() - rootTranslation.getY();
            double yaw = Math.atan2(dy, dx);

            goalOrientation.setYawPitchRoll(yaw, 0.0, 0.0);
         }

         goalPublisher.publish(new Pose3D(bombPosition, goalOrientation));
         ThreadTools.sleep(100);

         startPublisher.publish(new Empty());
      }

      @Override
      public void doAction(double timeInState)
      {
      }

      @Override
      public void onExit(double timeInState)
      {
         if (DEBUG)
         {
            LogTools.info("Exiting " + getClass().getSimpleName());
         }

         messager.submitMessage(BehaviorModule.API.BehaviorSelection, "null");
      }

      @Override
      public boolean isDone(double timeInState)
      {
         return isDone.get();
      }
   }

   public KryoMessager getKryoMessager()
   {
      return kryoMessager;
   }

   public static void main(String[] args)
   {
      // Start behavior coordinator
      BuildingExplorationBehaviorCoordinator behaviorCoordinator = new BuildingExplorationBehaviorCoordinator("Atlas", DomainFactory.PubSubImplementation.FAST_RTPS);
      behaviorCoordinator.setBombPosition(new Point3D(14.2, 0.0, 0.86));
      behaviorCoordinator.requestState(BuildingExplorationStateName.LOOK_AND_STEP);
      behaviorCoordinator.start();
   }
}
