package us.ihmc.atlas.behaviors.coordinator;

import controller_msgs.msg.dds.BehaviorControlModePacket;
import controller_msgs.msg.dds.BehaviorControlModeResponsePacket;
import controller_msgs.msg.dds.BehaviorStatusPacket;
import controller_msgs.msg.dds.HumanoidBehaviorTypePacket;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehavior;
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorAPI;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorControlModeEnum;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.CurrentBehaviorStatus;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorType;
import us.ihmc.messager.kryo.KryoMessager;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.robotics.time.ThreadTimer;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

public class BuildingExplorationBehaviorCoordinator
{
   static final int UPDATE_RATE_MILLIS = 50;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final Ros2Node ros2Node;
   private final YoEnum<BuildingExplorationStateName> requestedState = YoEnum.create("requestedState", "", BuildingExplorationStateName.class, registry, true);
   private final StateMachine<BuildingExplorationStateName, State> stateMachine;

   private final AtomicBoolean isRunning = new AtomicBoolean();
   private final ScheduledExecutorService executorService;
   private ScheduledFuture<?> stateMachineTask = null;

   public BuildingExplorationBehaviorCoordinator(String robotName, DomainFactory.PubSubImplementation pubSubImplementation)
   {
      ros2Node = ROS2Tools.createRos2Node(pubSubImplementation, getClass().getSimpleName());
      executorService = Executors.newSingleThreadScheduledExecutor();

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

   private StateMachine<BuildingExplorationStateName, State> buildStateMachine(String robotName) throws Exception
   {
      StateMachineFactory<BuildingExplorationStateName, State> factory = new StateMachineFactory<>(BuildingExplorationStateName.class);

      factory.setNamePrefix("buildingExploration");
      factory.setRegistry(registry);
      factory.buildClock(() -> Conversions.nanosecondsToSeconds(System.nanoTime()));

      factory.addState(BuildingExplorationStateName.TELEOP, new TeleopState());
      factory.addState(BuildingExplorationStateName.LOOK_AND_STEP, new LookAndStepState());
      factory.addState(BuildingExplorationStateName.WALK_THROUGH_DOOR, new WalkThroughDoorState(robotName));
      factory.addState(BuildingExplorationStateName.TRAVERSE_STAIRS, new TraverseStairsState());

      factory.addDoneTransition(BuildingExplorationStateName.LOOK_AND_STEP, BuildingExplorationStateName.TELEOP);
      factory.addDoneTransition(BuildingExplorationStateName.WALK_THROUGH_DOOR, BuildingExplorationStateName.TELEOP);
      factory.addDoneTransition(BuildingExplorationStateName.TRAVERSE_STAIRS, BuildingExplorationStateName.TELEOP);

      factory.addRequestedTransition(BuildingExplorationStateName.TELEOP, BuildingExplorationStateName.LOOK_AND_STEP, requestedState);
      factory.addRequestedTransition(BuildingExplorationStateName.TELEOP, BuildingExplorationStateName.WALK_THROUGH_DOOR, requestedState);
      factory.addRequestedTransition(BuildingExplorationStateName.TELEOP, BuildingExplorationStateName.TRAVERSE_STAIRS, requestedState);

      return factory.build(BuildingExplorationStateName.TELEOP);
   }

   private void update()
   {
      // TODO perform any transition checks based on fiducials, etc.

      stateMachine.doActionAndTransition();
   }

   private class TeleopState implements State
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

   private class LookAndStepState implements State
   {
      private final Pose3D bombPose = new Pose3D();
      private final KryoMessager lookAndStepMessager = KryoMessager.createClient(LookAndStepBehavior.DEFINITION.getBehaviorAPI(),
                                                                                 "localhost",
                                                                                 NetworkPorts.BEHAVIOUR_MODULE_PORT.getPort(),
                                                                                 BuildingExplorationBehaviorCoordinator.this.getClass().getSimpleName(),
                                                                                 50);

      public LookAndStepState() throws Exception
      {
         lookAndStepMessager.startMessager();
      }

      @Override
      public void onEntry()
      {
         lookAndStepMessager.submitMessage(LookAndStepBehaviorAPI.GoalInput, bombPose);

         // TODO start behavior topic
//         ThreadTools.sleep(100);
//         lookAndStepMessager.submitMessage(LookAndStepBehaviorAPI.Start, bombPose);
      }

      @Override
      public void doAction(double timeInState)
      {

      }

      @Override
      public void onExit(double timeInState)
      {
         // TODO stop behavior topic
//         lookAndStepMessager.submitMessage(LookAndStepBehaviorAPI.Reset, bombPose);
      }

      void setBombPose(Pose3DReadOnly bombPose)
      {
         this.bombPose.set(bombPose);
      }
   }

   private class WalkThroughDoorState implements State
   {
      final IHMCROS2Publisher<BehaviorControlModePacket> behaviorModePublisher;
      final IHMCROS2Publisher<HumanoidBehaviorTypePacket> behaviorTypePublisher;
      final AtomicBoolean isDone = new AtomicBoolean();

      public WalkThroughDoorState(String robotName)
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

   private class TraverseStairsState implements State
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
