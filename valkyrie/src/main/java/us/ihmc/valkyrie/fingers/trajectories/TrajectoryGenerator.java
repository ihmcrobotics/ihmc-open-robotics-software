package us.ihmc.valkyrie.fingers.trajectories;

import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class TrajectoryGenerator implements RobotController, DesiredTrajectoryInterface
{
   private final YoVariableRegistry registry;

   private final StateMachine<TrajectoryGeneratorState, State> stateMachine;
   private final YoEnum<TrajectoryGeneratorState> requestedState;

   private final TrajectoryInterface trajectory;

   enum TrajectoryGeneratorState
   {
      WORKING, DONOTHING
   }

   public TrajectoryGenerator(String name, YoDouble yoTime, YoVariableRegistry parentRegistry, TrajectoryInterface trajectory)
   {
      registry = new YoVariableRegistry(name);

      requestedState = new YoEnum<>(name + "requestedState", registry, TrajectoryGeneratorState.class, true);
      requestedState.set(null);

      StateMachineFactory<TrajectoryGeneratorState, State> factory = new StateMachineFactory<>(TrajectoryGeneratorState.class);

      factory.setNamePrefix(name).setRegistry(registry).buildYoClock(yoTime);

      StateWorking stateWorking = new StateWorking();
      StateDonothing stateDonothing = new StateDonothing();

      factory.addState(TrajectoryGeneratorState.WORKING, stateWorking);
      factory.addState(TrajectoryGeneratorState.DONOTHING, stateDonothing);
      factory.addRequestedTransition(TrajectoryGeneratorState.WORKING, TrajectoryGeneratorState.DONOTHING, requestedState, true);
      factory.addRequestedTransition(TrajectoryGeneratorState.DONOTHING, TrajectoryGeneratorState.WORKING, requestedState, false);

      stateMachine = factory.build(TrajectoryGeneratorState.DONOTHING);

      parentRegistry.addChild(registry);

      this.trajectory = trajectory;
   }

   public void executeTrajectory(double trajectoryTime, double delayTime, double... goalConditions)
   {
      trajectory.setGoal(trajectoryTime, delayTime, goalConditions);
      requestedState.set(TrajectoryGeneratorState.WORKING);
      stateMachine.doTransitions();
   }

   private class StateWorking implements State
   {
      @Override
      public void onEntry()
      {

      }

      @Override
      public void doAction(double timeInState)
      {

      }

      @Override
      public boolean isDone(double timeInState)
      {
         boolean b = timeInState >= trajectory.getTrajectoryTime();
         if (b)
         {
            requestedState.set(TrajectoryGeneratorState.DONOTHING);
         }
         return b;
      }

      @Override
      public void onExit()
      {
         trajectory.initialize(trajectory.getGoalConditions());
      }
   }

   private class StateDonothing implements State
   {
      @Override
      public void onEntry()
      {

      }

      @Override
      public void doAction(double timeInState)
      {

      }

      @Override
      public void onExit()
      {
         PrintTools.info("StateDonothing onExit");
      }
   }

   @Override
   public void initialize()
   {

   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return registry.getName();
   }

   @Override
   public String getDescription()
   {
      return registry.getName() + this.getName();
   }

   @Override
   public void doControl()
   {
      stateMachine.doActionAndTransition();
   }

   @Override
   public double getDesiredQ()
   {
      return trajectory.getQ(getTime());
   }

   @Override
   public double getDesiredQd()
   {
      return trajectory.getQd(getTime());
   }

   private double getTime()
   {
      if (stateMachine.getCurrentStateKey() == TrajectoryGeneratorState.WORKING)
      {
         return stateMachine.getTimeInCurrentState();
      }
      else
      {
         return 0.0;
      }
   }
}