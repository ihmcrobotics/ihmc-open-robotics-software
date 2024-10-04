package us.ihmc.avatar.networkProcessor.referenceSpreading;

import com.esotericsoftware.minlog.Log;
import controller_msgs.msg.dds.HandTrajectoryMessage;
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

public class ReferenceSpreadingStateHelper
{
   public enum States
   {
      BEFORE,
      AFTER,
      DONE
   }

   HandTrajectoryMessagePublisher trajectoryMessagePublisher;
   YoRegistry registry;

   ReferenceSpreadingTrajectory preImpactReference;

   public ReferenceSpreadingStateHelper(String filePath, HandTrajectoryMessagePublisher trajectoryMessagePublisher, YoRegistry registry)
   {
      this.trajectoryMessagePublisher = trajectoryMessagePublisher;
      this.registry = registry;
      preImpactReference = new ReferenceSpreadingTrajectory(filePath);
   }

   public StateMachine<States, State> setUpStateMachines(DoubleProvider time)
   {
      StateMachineFactory<States, State> factory = new StateMachineFactory<>(States.class);
      factory.setNamePrefix("stateMachine").setRegistry(registry).buildYoClock(time);

      factory.addState(States.BEFORE, new BeforeState());
      factory.addState(States.AFTER, new AfterState());
      factory.addState(States.DONE, new DoneState());

      StateTransitionCondition beforeToAfterTransitionCondition = t -> t>1;

      factory.addTransition(States.BEFORE, States.AFTER, beforeToAfterTransitionCondition);
      factory.addDoneTransition(States.AFTER, States.DONE);

      return factory.build(States.BEFORE);
   }

   private class BeforeState implements State
   {
      public BeforeState()
      {
      }

      public void doAction(double timeInState)
      {
         LogTools.info("BeforeState: " + timeInState);
      }

      public void onEntry()
      {
         LogTools.info("Entering BeforeState");
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
         LogTools.info("AfterState: " + timeInState);
      }

      public void onEntry()
      {
         LogTools.info("Entering AfterState");
         HandTrajectoryMessage handTrajectoryMessage = preImpactReference.getHandTrajectoryMessage(RobotSide.LEFT);
         //         LogTools.info("HandTrajectoryMessage: " + handTrajectoryMessage);
         trajectoryMessagePublisher.publish(handTrajectoryMessage);
      }

      public void onExit(double timeInState)
      {
         LogTools.info("Exiting AfterState");
      }
   }

   private class DoneState implements State
   {
      public DoneState()
      {
      }

      public void doAction(double timeInState)
      {
      }

      public void onEntry()
      {
         LogTools.info("Entering DoneState");
      }

      public void onExit(double timeInState)
      {
         LogTools.info("Exiting DoneState: " + timeInState);
      }
   }

   public interface RSTimeProvider
   {
      void initialize();

      void update(long currentTime);

      double getTime();

      static RSTimeProvider createTimeProfider()
      {
         return new RSTimeProvider()
         {
            private double time = 0.0;
            private long initialTime = -1L;

            @Override
            public void initialize()
            {
               time = 0.0;
               initialTime = -1L;
            }

            @Override
            public void update(long currentTime)
            {
               if (initialTime == -1L)
               {
                  initialTime = currentTime;
               }
               time = Conversions.nanosecondsToSeconds(currentTime-initialTime);

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
