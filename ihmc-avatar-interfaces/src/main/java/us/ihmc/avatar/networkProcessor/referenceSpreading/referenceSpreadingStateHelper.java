package us.ihmc.avatar.networkProcessor.referenceSpreading;

import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxController;
import us.ihmc.commons.Conversions;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class referenceSpreadingStateHelper
{
   public enum States
   {
      BEFORE,
      AFTER,
      DONE
   }

   YoRegistry registry;

   public referenceSpreadingStateHelper(YoRegistry registry)
   {
      this.registry = registry;
   }

   public StateMachine<States, State> setUpStateMachines(DoubleProvider time)
   {
      StateMachineFactory<States, State> factory = new StateMachineFactory<>(States.class);
      factory.setNamePrefix("stateMachine").setRegistry(registry).buildYoClock(time);

      factory.addState(States.BEFORE, new BeforeState());
      factory.addState(States.AFTER, new AfterState());
      factory.addState(States.DONE, new DoneState());

      StateTransitionCondition beforeToAfterTransitionCondition = t -> {
         LogTools.info("Time: " + t);
         return t > 1.5;};

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
      }

      public void onEntry()
      {
      }

      public void onExit(double timeInState)
      {

      }
   }

   private class AfterState implements State
   {
      public AfterState()
      {
      }

      public void doAction(double timeInState)
      {
         LogTools.info("TimeInState: " + timeInState);
      }

      public void onEntry()
      {
      }

      public void onExit(double timeInState)
      {

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
      }

      public void onExit(double timeInState)
      {

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
