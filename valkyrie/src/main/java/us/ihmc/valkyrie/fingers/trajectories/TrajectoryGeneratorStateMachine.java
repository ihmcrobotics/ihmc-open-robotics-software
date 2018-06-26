package us.ihmc.valkyrie.fingers.trajectories;

import us.ihmc.robotics.math.trajectories.DoubleTrajectoryGenerator;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class TrajectoryGeneratorStateMachine implements RobotController, DoubleTrajectoryGenerator
{
   private final YoVariableRegistry registry;

   private final StateMachine<TrajectoryGeneratorState, State> stateMachine;
   private final YoEnum<TrajectoryGeneratorState> requestedState;

   private final GoalPositionTrajectory trajectory;

   enum TrajectoryGeneratorState
   {
      WORKING, DONOTHING
   }

   public TrajectoryGeneratorStateMachine(String name, YoDouble yoTime, YoVariableRegistry parentRegistry, GoalPositionTrajectory trajectory)
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
      factory.addRequestedTransition(TrajectoryGeneratorState.WORKING, TrajectoryGeneratorState.WORKING, requestedState, false);
      factory.addRequestedTransition(TrajectoryGeneratorState.WORKING, TrajectoryGeneratorState.DONOTHING, requestedState, true);
      factory.addRequestedTransition(TrajectoryGeneratorState.DONOTHING, TrajectoryGeneratorState.WORKING, requestedState, false);

      stateMachine = factory.build(TrajectoryGeneratorState.DONOTHING);

      parentRegistry.addChild(registry);

      this.trajectory = trajectory;
   }

   public void executeTrajectory(double trajectoryTime, double delayTime, double... goalConditions)
   {
      trajectory.setGoalPosition(trajectoryTime, delayTime, goalConditions[0]);
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
         if (trajectory.isDone())
            requestedState.set(TrajectoryGeneratorState.DONOTHING);
         return trajectory.isDone();
      }

      @Override
      public void onExit()
      {
         trajectory.initialize();
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
      if (stateMachine.getCurrentStateKey() == TrajectoryGeneratorState.WORKING)
         compute(stateMachine.getTimeInCurrentState());
      stateMachine.doActionAndTransition();
   }

   @Override
   public void compute(double time)
   {
      trajectory.compute(time);
   }

   @Override
   public boolean isDone()
   {
      return false;
   }

   @Override
   public double getValue()
   {
      return trajectory.getValue();
   }

   @Override
   public double getVelocity()
   {
      return trajectory.getVelocity();
   }

   @Override
   public double getAcceleration()
   {
      return 0;
   }
}