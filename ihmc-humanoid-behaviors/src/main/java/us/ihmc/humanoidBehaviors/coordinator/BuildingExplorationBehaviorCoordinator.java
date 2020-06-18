package us.ihmc.humanoidBehaviors.coordinator;

import controller_msgs.msg.dds.BehaviorControlModePacket;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorControlModeEnum;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
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

public class BuildingExplorationBehaviorCoordinator
{
   static final int UPDATE_RATE_MILLIS = 50;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final Ros2Node ros2Node;
   private final YoEnum<BuildingExplorationStateName> requestedState = YoEnum.create("requestedState", "", BuildingExplorationStateName.class, registry, true);
   private final StateMachine<BuildingExplorationStateName, BuildingExplorationState> stateMachine;

   private final AtomicBoolean isRunning = new AtomicBoolean();
   private final ScheduledExecutorService executorService;
   private ScheduledFuture<?> stateMachineTask = null;

   public BuildingExplorationBehaviorCoordinator(String robotName, DomainFactory.PubSubImplementation pubSubImplementation)
   {
      ros2Node = ROS2Tools.createRos2Node(pubSubImplementation, getClass().getSimpleName());
      stateMachine = buildStateMachine(robotName);
      executorService = Executors.newSingleThreadScheduledExecutor();
   }

   public void start()
   {
      if (!isRunning.get())
      {
         isRunning.set(true);
         stateMachine.resetToInitialState();
         stateMachineTask = executorService.scheduleAtFixedRate(stateMachine::doActionAndTransition, 0, UPDATE_RATE_MILLIS, TimeUnit.MILLISECONDS);
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

         // Trigger all non-teleop behaviors to stop
         BehaviorControlModePacket behaviorControlModePacket = new BehaviorControlModePacket();
         behaviorControlModePacket.setBehaviorControlModeEnumRequest(BehaviorControlModePacket.STOP);

         stateMachine.getState(BuildingExplorationStateName.LOOK_AND_STEP).outputPublisher.publish(behaviorControlModePacket);
         stateMachine.getState(BuildingExplorationStateName.WALK_THROUGH_DOOR).outputPublisher.publish(behaviorControlModePacket);
         stateMachine.getState(BuildingExplorationStateName.TRAVERSE_STAIRS).outputPublisher.publish(behaviorControlModePacket);
      }
   }

   /* adding package-private getter for tests */
   StateMachine<BuildingExplorationStateName, BuildingExplorationState> getStateMachine()
   {
      return stateMachine;
   }

   private StateMachine<BuildingExplorationStateName, BuildingExplorationState> buildStateMachine(String robotName)
   {
      StateMachineFactory<BuildingExplorationStateName, BuildingExplorationState> factory = new StateMachineFactory<>(BuildingExplorationStateName.class);
      factory.setNamePrefix("buildingExploration");
      factory.setRegistry(registry);
      factory.buildClock(() -> Conversions.nanosecondsToSeconds(System.nanoTime()));

      factory.addState(BuildingExplorationStateName.TELEOP, new BuildingExplorationState(robotName, BuildingExplorationStateName.TELEOP));
      factory.addState(BuildingExplorationStateName.LOOK_AND_STEP, new BuildingExplorationState(robotName, BuildingExplorationStateName.LOOK_AND_STEP));
      factory.addState(BuildingExplorationStateName.WALK_THROUGH_DOOR, new BuildingExplorationState(robotName, BuildingExplorationStateName.WALK_THROUGH_DOOR));
      factory.addState(BuildingExplorationStateName.TRAVERSE_STAIRS, new BuildingExplorationState(robotName, BuildingExplorationStateName.TRAVERSE_STAIRS));

      factory.addDoneTransition(BuildingExplorationStateName.LOOK_AND_STEP, BuildingExplorationStateName.TELEOP);
      factory.addDoneTransition(BuildingExplorationStateName.WALK_THROUGH_DOOR, BuildingExplorationStateName.TELEOP);
      factory.addDoneTransition(BuildingExplorationStateName.TRAVERSE_STAIRS, BuildingExplorationStateName.TELEOP);

      factory.addRequestedTransition(BuildingExplorationStateName.TELEOP, BuildingExplorationStateName.LOOK_AND_STEP, requestedState);
      factory.addRequestedTransition(BuildingExplorationStateName.TELEOP, BuildingExplorationStateName.WALK_THROUGH_DOOR, requestedState);
      factory.addRequestedTransition(BuildingExplorationStateName.TELEOP, BuildingExplorationStateName.TRAVERSE_STAIRS, requestedState);

      return factory.build(BuildingExplorationStateName.TELEOP);
   }

   public void requestState(BuildingExplorationStateName requestedState)
   {
      this.requestedState.set(requestedState);
   }

   class BuildingExplorationState implements State
   {
      private final IHMCROS2Publisher<BehaviorControlModePacket> outputPublisher;
      private final AtomicBoolean isDone = new AtomicBoolean();

      private BuildingExplorationState(String robotName, BuildingExplorationStateName buildingExplorationStateName)
      {
         ROS2Topic<?> inputTopic = BehaviorCoordinatorCommunicationTools.getCoordinatorInputTopic(robotName, buildingExplorationStateName);
         ROS2Topic<?> outputTopic = BehaviorCoordinatorCommunicationTools.getCoordinatorOutputTopic(robotName, buildingExplorationStateName);

         outputPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, BehaviorControlModePacket.class, outputTopic);
         ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, BehaviorControlModePacket.class, inputTopic, s ->
         {
            BehaviorControlModePacket inputPacket = s.takeNextData();
            BehaviorControlModeEnum controlMode = BehaviorControlModeEnum.fromByte(inputPacket.getBehaviorControlModeEnumRequest());
            if (controlMode == BehaviorControlModeEnum.STOP)
            {
               isDone.set(true);
            }
         });
      }

      @Override
      public void onEntry()
      {
         isDone.set(false);

         BehaviorControlModePacket behaviorControlModePacket = new BehaviorControlModePacket();
         behaviorControlModePacket.setBehaviorControlModeEnumRequest(BehaviorControlModePacket.RESUME);
         outputPublisher.publish(behaviorControlModePacket);
      }

      @Override
      public void doAction(double timeInState)
      {
         // do nothing by default
      }

      @Override
      public void onExit(double timeInState)
      {
         BehaviorControlModePacket behaviorControlModePacket = new BehaviorControlModePacket();
         behaviorControlModePacket.setBehaviorControlModeEnumRequest(BehaviorControlModePacket.STOP);
         outputPublisher.publish(behaviorControlModePacket);
      }

      @Override
      public boolean isDone(double timeInState)
      {
         return isDone.get();
      }
   }
}
