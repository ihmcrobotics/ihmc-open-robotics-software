package us.ihmc.atlas.behaviors.coordinator;

import controller_msgs.msg.dds.*;
import std_msgs.msg.dds.Empty;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorControlModeEnum;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.CurrentBehaviorStatus;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorType;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class BuildingExplorationBehaviorCoordinator
{
   private static final int UPDATE_RATE_MILLIS = 50;
   private static final double xyProximityToDoorToStopWalking = 2.0;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final Ros2Node ros2Node;
   private final YoEnum<BuildingExplorationStateName> requestedState = YoEnum.create("requestedState", "", BuildingExplorationStateName.class, registry, true);
   private final StateMachine<BuildingExplorationStateName, State> stateMachine;

   private final AtomicBoolean isRunning = new AtomicBoolean();
   private final AtomicReference<DoorLocationPacket> doorLocationPacket = new AtomicReference<>();
   private final AtomicReference<RobotConfigurationData> robotConfigurationData = new AtomicReference<>();

   private final ScheduledExecutorService executorService;
   private ScheduledFuture<?> stateMachineTask = null;

   public BuildingExplorationBehaviorCoordinator(String robotName, DomainFactory.PubSubImplementation pubSubImplementation)
   {
      ros2Node = ROS2Tools.createRos2Node(pubSubImplementation, getClass().getSimpleName());
      executorService = Executors.newSingleThreadScheduledExecutor();

      ROS2Topic<?> objectDetectionTopic = ROS2Tools.OBJECT_DETECTOR_TOOLBOX.withRobot(robotName).withOutput();
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, DoorLocationPacket.class, objectDetectionTopic, s -> doorLocationPacket.set(s.takeNextData()));

      ROS2Topic<?> controllerOutputTopic = ROS2Tools.getControllerOutputTopic(robotName);
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, RobotConfigurationData.class, controllerOutputTopic, s -> robotConfigurationData.set(s.takeNextData()));

      try
      {
         stateMachine = buildStateMachine(robotName);
      }
      catch (Exception e)
      {
         throw new RuntimeException(e);
      }
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

   private StateMachine<BuildingExplorationStateName, State> buildStateMachine(String robotName)
   {
      StateMachineFactory<BuildingExplorationStateName, State> factory = new StateMachineFactory<>(BuildingExplorationStateName.class);

      factory.setNamePrefix("buildingExploration");
      factory.setRegistry(registry);
      factory.buildClock(() -> Conversions.nanosecondsToSeconds(System.nanoTime()));

      factory.addState(BuildingExplorationStateName.TELEOP, new TeleopState());
      factory.addState(BuildingExplorationStateName.LOOK_AND_STEP, new LookAndStepState(ros2Node));
      factory.addState(BuildingExplorationStateName.WALK_THROUGH_DOOR, new WalkThroughDoorState(robotName, ros2Node));
      factory.addState(BuildingExplorationStateName.TRAVERSE_STAIRS, new TraverseStairsState());

      factory.addDoneTransition(BuildingExplorationStateName.LOOK_AND_STEP, BuildingExplorationStateName.TELEOP);
      factory.addDoneTransition(BuildingExplorationStateName.WALK_THROUGH_DOOR, BuildingExplorationStateName.TELEOP);
      factory.addDoneTransition(BuildingExplorationStateName.TRAVERSE_STAIRS, BuildingExplorationStateName.TELEOP);

      factory.addRequestedTransition(BuildingExplorationStateName.TELEOP, BuildingExplorationStateName.LOOK_AND_STEP, requestedState);
      factory.addRequestedTransition(BuildingExplorationStateName.TELEOP, BuildingExplorationStateName.WALK_THROUGH_DOOR, requestedState);
      factory.addRequestedTransition(BuildingExplorationStateName.TELEOP, BuildingExplorationStateName.TRAVERSE_STAIRS, requestedState);

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
      }

      @Override
      public void doAction(double timeInState)
      {
         // do nothing
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
      private final IHMCROS2Publisher<Pose3D> goalPublisher;
      private final IHMCROS2Publisher<Empty> resetPublisher;

      private final Pose3D bombPose = new Pose3D();

      public LookAndStepState(Ros2Node ros2Node)
      {
         ROS2Topic<Pose3D> goalTopic = ROS2Tools.BEHAVIOR_MODULE.withInput().withType(Pose3D.class);
         ROS2Topic<Empty> resetTopic = ROS2Tools.BEHAVIOR_MODULE.withInput().withType(Empty.class);

         goalPublisher = ROS2Tools.createPublisher(ros2Node, goalTopic);
         resetPublisher = ROS2Tools.createPublisher(ros2Node, resetTopic);
      }

      @Override
      public void onEntry()
      {
         goalPublisher.publish(bombPose);
      }

      @Override
      public void doAction(double timeInState)
      {

      }

      @Override
      public void onExit(double timeInState)
      {

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
         ROS2Topic<?> outputTopic = ROS2Tools.BEHAVIOR_MODULE.withRobot(robotName).withInput();

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
      }

      @Override
      public void doAction(double timeInState)
      {
      }

      @Override
      public void onExit(double timeInState)
      {
         // TODO fill in once state is implemented
      }

      @Override
      public boolean isDone(double timeInState)
      {
         // TODO listen for callback from stairs behavior
         return false;
      }
   }
}
