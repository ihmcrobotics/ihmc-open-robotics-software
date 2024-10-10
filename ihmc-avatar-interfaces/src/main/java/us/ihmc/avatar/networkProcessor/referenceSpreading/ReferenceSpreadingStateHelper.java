package us.ihmc.avatar.networkProcessor.referenceSpreading;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.HandHybridJointspaceTaskspaceTrajectoryMessage;
import controller_msgs.msg.dds.SpatialVectorMessage;
import us.ihmc.avatar.networkProcessor.referenceSpreading.ReferenceSpreadingToolboxController.HandTrajectoryMessagePublisher;
import us.ihmc.commons.Conversions;
import us.ihmc.log.LogTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

public class ReferenceSpreadingStateHelper
{
   public enum States
   {
      BEFORE,
      AFTER,
      WAITING
   }

   HandTrajectoryMessagePublisher trajectoryMessagePublisher;
   YoRegistry registry;

   ReferenceSpreadingTrajectory preImpactReference;
   HashMap<RobotSide, SpatialVectorMessage> handWrenches = new HashMap<>(RobotSide.values().length);

   public ReferenceSpreadingStateHelper(String filePath, FullHumanoidRobotModel fullRobotModel, HandTrajectoryMessagePublisher trajectoryMessagePublisher, YoRegistry registry)
   {
      this.trajectoryMessagePublisher = trajectoryMessagePublisher;
      this.registry = registry;
      preImpactReference = new ReferenceSpreadingTrajectory(filePath, fullRobotModel);

      for (RobotSide robotSide : RobotSide.values()) {
         handWrenches.put(robotSide, new SpatialVectorMessage());
      }
   }

   public StateMachine<States, State> setUpStateMachines(DoubleProvider time)
   {

      StateMachineFactory<States, State> factory = new StateMachineFactory<>(States.class);
      factory.setNamePrefix("stateMachine").setRegistry(registry).buildYoClock(time);

      factory.addState(States.BEFORE, new BeforeState());
      factory.addState(States.AFTER, new AfterState());
      factory.addState(States.WAITING, new WaitingState());

      StateTransitionCondition beforeToAfterTransitionCondition = t -> t>1;

      factory.addTransition(States.BEFORE, States.AFTER, beforeToAfterTransitionCondition);
      factory.addDoneTransition(States.AFTER, States.WAITING);


      return factory.build(States.BEFORE);
   }

   public void setTrajectoryMessagePublisher(HandTrajectoryMessagePublisher trajectoryMessagePublisher)
   {
      this.trajectoryMessagePublisher = trajectoryMessagePublisher;
   }

   public void updateHandWrenches(CapturabilityBasedStatus capturabilityBasedStatus)
   {
      handWrenches.get(RobotSide.LEFT).set(capturabilityBasedStatus.getLeftHandWrench());
      handWrenches.get(RobotSide.RIGHT).set(capturabilityBasedStatus.getRightHandWrench());
   }

   private class BeforeState implements State
   {

      public BeforeState()
      {
      }

      public void doAction(double timeInState)
      {
//         LogTools.info("BeforeState: " + timeInState);
      }

      public void onEntry()
      {
         LogTools.info("Entering BeforeState");
         for (RobotSide robotSide : RobotSide.values())
         {
            HandHybridJointspaceTaskspaceTrajectoryMessage handHybridTrajectoryMessage = preImpactReference.getHandHybridTrajectoryMessage(robotSide);
            trajectoryMessagePublisher.publish(handHybridTrajectoryMessage);
         }

         LogTools.info("Published all messages");
      }

      public void onExit(double timeInState)
      {
         LogTools.info("Exiting BeforeState");
      }
   }

   private class AfterState implements State
   {
      public AfterState()
      {
      }

      public void doAction(double timeInState)
      {
//         LogTools.info("AfterState: " + timeInState);
           LogTools.info("Hand wrenches: " + handWrenches);
      }

      public void onEntry()
      {
         LogTools.info("Entering AfterState");
      }

      public void onExit(double timeInState)
      {
         LogTools.info("Exiting AfterState");
      }
   }

   private class WaitingState implements State
   {
      public WaitingState()
      {
      }

      public void doAction(double timeInState)
      {
      }

      public void onEntry()
      {
         LogTools.info("Entering WaitingState");
      }

      public void onExit(double timeInState)
      {
         LogTools.info("Exiting WaitingState: " + timeInState);
      }
   }

   public interface RSTimeProvider
   {
      void initialize();

      void update(long currentTime);

      double getTime();

      static RSTimeProvider createTimeProvider()
      {
         return new RSTimeProvider()
         {
            private double time = 0.0;

            @Override
            public void initialize()
            {
               time = 0.0;
            }

            @Override
            public void update(long currentTime)
            {
               time = Conversions.nanosecondsToSeconds(currentTime);

            }

            @Override
            public double getTime()
            {
               return time;
            }
         };
      }
   }
}
