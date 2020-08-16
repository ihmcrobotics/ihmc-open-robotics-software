package us.ihmc.atlas.behaviors.coordinator;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.BehaviorControlModePacket;
import controller_msgs.msg.dds.BehaviorStatusPacket;
import controller_msgs.msg.dds.DoorLocationPacket;
import controller_msgs.msg.dds.HumanoidBehaviorTypePacket;
import controller_msgs.msg.dds.RobotConfigurationData;
import std_msgs.msg.dds.Empty;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidBehaviors.BehaviorModule;
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehavior;
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorAPI;
import us.ihmc.humanoidBehaviors.ui.BehaviorUIRegistry;
import us.ihmc.humanoidBehaviors.ui.behaviors.LookAndStepBehaviorUI;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorControlModeEnum;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.CurrentBehaviorStatus;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorType;
import us.ihmc.log.LogTools;
import us.ihmc.messager.kryo.KryoMessager;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoEnum;

public class BuildingExplorationBehaviorCoordinator
{
   private static final boolean DEBUG = true;
   private static final int UPDATE_RATE_MILLIS = 50;
   private static final double xyProximityToDoorToStopWalking = 1.6;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final Ros2Node ros2Node;
   private final YoEnum<BuildingExplorationStateName> requestedState = new YoEnum<>("requestedState", "", registry, BuildingExplorationStateName.class, true);
   private final StateMachine<BuildingExplorationStateName, State> stateMachine;

   private final AtomicBoolean isRunning = new AtomicBoolean();
   private final AtomicReference<DoorLocationPacket> doorLocationPacket = new AtomicReference<>();
   private final AtomicReference<RobotConfigurationData> robotConfigurationData = new AtomicReference<>();

   private final ScheduledExecutorService executorService;
   private ScheduledFuture<?> stateMachineTask = null;

   private final TeleopState teleopState;
   private final LookAndStepState lookAndStepState;
   private final WalkThroughDoorState walkThroughDoorState;
   private final TraverseStairsState traverseStairsState;

   public BuildingExplorationBehaviorCoordinator(String robotName, DomainFactory.PubSubImplementation pubSubImplementation)
   {
      ros2Node = ROS2Tools.createRos2Node(pubSubImplementation, getClass().getSimpleName());
      executorService = Executors.newSingleThreadScheduledExecutor();

      teleopState = new TeleopState();
      lookAndStepState = new LookAndStepState(ros2Node);
      walkThroughDoorState = new WalkThroughDoorState(robotName, ros2Node);
      traverseStairsState = new TraverseStairsState();

      ROS2Topic<?> objectDetectionTopic = ROS2Tools.OBJECT_DETECTOR_TOOLBOX.withRobot(robotName).withOutput();
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, DoorLocationPacket.class, objectDetectionTopic, s -> doorLocationPacket.set(s.takeNextData()));

      ROS2Topic<?> controllerOutputTopic = ROS2Tools.getControllerOutputTopic(robotName);
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, RobotConfigurationData.class, controllerOutputTopic, s -> robotConfigurationData.set(s.takeNextData()));

      try
      {
         stateMachine = buildStateMachine();
      }
      catch (Exception e)
      {
         throw new RuntimeException(e);
      }
   }

   public void setBombPose(Pose3D bombPose)
   {
      lookAndStepState.bombPose.set(bombPose);
   }

   public void requestState(BuildingExplorationStateName requestedState)
   {
      this.requestedState.set(requestedState);
   }

   public void start()
   {
      if (!isRunning.get())
      {
         doorLocationPacket.set(null);
         robotConfigurationData.set(null);

         isRunning.set(true);
         stateMachine.resetToInitialState();
         stateMachineTask = executorService.scheduleAtFixedRate(this::update, 0, UPDATE_RATE_MILLIS, TimeUnit.MILLISECONDS);
      }
   }

   public void stop()
   {
      if (isRunning.get())
      {
         isRunning.set(false);

         if (stateMachineTask != null)
         {
            stateMachineTask.cancel(true);
            stateMachineTask = null;
         }
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

      return factory.build(BuildingExplorationStateName.TELEOP);
   }

   private void update()
   {
      stateMachine.doActionAndTransition();
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
      return xyDistanceToDoor <= xyProximityToDoorToStopWalking;
   }

   private static class LookAndStepState implements State
   {
      private final KryoMessager messager;
      private final IHMCROS2Publisher<Pose3D> goalPublisher;
      private final IHMCROS2Publisher<Empty> resetPublisher;
      private final Pose3D bombPose = new Pose3D();

      public LookAndStepState(Ros2Node ros2Node)
      {
         BehaviorUIRegistry behaviorRegistry = BehaviorUIRegistry.of(LookAndStepBehaviorUI.DEFINITION);
         goalPublisher = IHMCROS2Publisher.newPose3DPublisher(ros2Node, ROS2Tools.BEHAVIOR_MODULE.withInput().withType(Pose3D.class));
         resetPublisher = ROS2Tools.createPublisher(ros2Node, ROS2Tools.BEHAVIOR_MODULE.withInput().withType(Empty.class));

         messager = KryoMessager.createClient(behaviorRegistry.getMessagerAPI(),
                                              "127.0.0.1",
                                              NetworkPorts.BEHAVIOUR_MODULE_PORT.getPort(),
                                              getClass().getSimpleName(),
                                              UPDATE_RATE_MILLIS);
         try
         {
            messager.startMessager();
         }
         catch (Exception e)
         {
            throw new RuntimeException(e);
         }
      }

      @Override
      public void onEntry()
      {
         resetPublisher.publish(new Empty());

         String behaviorName = LookAndStepBehavior.DEFINITION.getName();
         messager.submitMessage(BehaviorModule.API.BehaviorSelection, behaviorName);
         ThreadTools.sleep(100);

         messager.submitMessage(LookAndStepBehaviorAPI.OperatorReviewEnabled, false);
         ThreadTools.sleep(100);

         goalPublisher.publish(bombPose);
         ThreadTools.sleep(100);

         if (DEBUG)
         {
            LogTools.info("Entering " + getClass().getSimpleName());
         }
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

         resetPublisher.publish(new Empty());
      }

      void setBombPose(Pose3DReadOnly bombPose)
      {
         this.bombPose.set(bombPose);
      }
   }

   private static class WalkThroughDoorState implements State
   {
      final IHMCROS2Publisher<BehaviorControlModePacket> behaviorModePublisher;
      final IHMCROS2Publisher<HumanoidBehaviorTypePacket> behaviorTypePublisher;
      final AtomicBoolean isDone = new AtomicBoolean();

      public WalkThroughDoorState(String robotName, Ros2Node ros2Node)
      {
         ROS2Topic<?> inputTopic = ROS2Tools.BEHAVIOR_MODULE.withRobot(robotName).withInput();
         ROS2Topic<?> outputTopic = ROS2Tools.BEHAVIOR_MODULE.withRobot(robotName).withOutput();

         behaviorModePublisher = ROS2Tools.createPublisher(ros2Node, BehaviorControlModePacket.class, inputTopic);
         behaviorTypePublisher = ROS2Tools.createPublisher(ros2Node, HumanoidBehaviorTypePacket.class, inputTopic);

         ROS2Tools.createCallbackSubscription(ros2Node, BehaviorStatusPacket.class, outputTopic, s ->
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
         HumanoidBehaviorTypePacket humanoidBehaviorTypePacket = new HumanoidBehaviorTypePacket();
         humanoidBehaviorTypePacket.setHumanoidBehaviorType(HumanoidBehaviorType.WALK_THROUGH_DOOR.toByte());
         behaviorTypePublisher.publish(humanoidBehaviorTypePacket);
         isDone.set(false);

         if (DEBUG)
         {
            LogTools.info("Entering " + getClass().getSimpleName());
         }
      }

      @Override
      public void doAction(double timeInState)
      {
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
   }

   private static class TraverseStairsState implements State
   {
      @Override
      public void onEntry()
      {
         // TODO fill in once state is implemented
         if (DEBUG)
         {
            LogTools.info("Entering " + getClass().getSimpleName());
         }
      }

      @Override
      public void doAction(double timeInState)
      {
      }

      @Override
      public void onExit(double timeInState)
      {
         // TODO fill in once state is implemented
         if (DEBUG)
         {
            LogTools.info("Exiting " + getClass().getSimpleName());
         }
      }

      @Override
      public boolean isDone(double timeInState)
      {
         // TODO listen for callback from stairs behavior
         return true;
      }
   }

   public static void main(String[] args)
   {
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS);

      // Start behavior coordinator
      BuildingExplorationBehaviorCoordinator behaviorCoordinator = new BuildingExplorationBehaviorCoordinator(robotModel.getSimpleRobotName(), DomainFactory.PubSubImplementation.FAST_RTPS);
      behaviorCoordinator.setBombPose(new Pose3D(14.5, 0.0, 0.0, 0.0, 0.0, 0.0));
      behaviorCoordinator.requestState(BuildingExplorationStateName.LOOK_AND_STEP);
      behaviorCoordinator.start();
   }
}
