package us.ihmc.valkyrie.fingers;

import java.util.EnumMap;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleTrajectoryPoint1D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.robotics.trajectories.providers.SettableDoubleProvider;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class ProposedValkyrieFingerSetController<T extends Enum<T>> implements RobotController
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry;

   private final RobotSide robotSide;

   private Map<String, SettableDoubleProvider> fingerControlSpace;
   private Map<String, MultipleWaypointsTrajectoryGenerator> trajectoryGenerators;

   private final StateMachine<TrajectoryGeneratorMode, State> stateMachine;
   private final YoEnum<TrajectoryGeneratorMode> requestedState;

   enum TrajectoryGeneratorMode
   {
      JOINTSPACE
   }

   public ProposedValkyrieFingerSetController(RobotSide robotSide, YoDouble yoTime, EnumMap<T, DoubleProvider> fingerControlSpaceMap,
                                              YoVariableRegistry parentRegistry)
   {
      this.robotSide = robotSide;
      registry = new YoVariableRegistry(this.robotSide.getCamelCaseName() + name);

      trajectoryGenerators = new HashMap<String, MultipleWaypointsTrajectoryGenerator>();
      this.fingerControlSpace = new HashMap<String, SettableDoubleProvider>();
      Object[] array = fingerControlSpaceMap.keySet().toArray();
      for (int i = 0; i < array.length; i++)
      {
         String namePrefix = robotSide.getLowerCaseName() + array[i].toString();
         MultipleWaypointsTrajectoryGenerator trajectoryGenerator = new MultipleWaypointsTrajectoryGenerator(namePrefix + "trajectory", parentRegistry);
         trajectoryGenerator.clear();
         trajectoryGenerator.appendWaypoint(yoTime.getDoubleValue(), fingerControlSpaceMap.get(array[i]).getValue(), 0.0);
         trajectoryGenerator.initialize();
         trajectoryGenerators.put(namePrefix, trajectoryGenerator);

         SettableDoubleProvider controlSpace = new SettableDoubleProvider();

         this.fingerControlSpace.put(namePrefix, controlSpace);
      }

      requestedState = new YoEnum<>(name + "requestedState", registry, TrajectoryGeneratorMode.class, true);
      requestedState.set(null);
      StateMachineFactory<TrajectoryGeneratorMode, State> factory = new StateMachineFactory<>(TrajectoryGeneratorMode.class);

      factory.setNamePrefix(name).setRegistry(registry).buildYoClock(yoTime);

      JointSpaceState stateWorking = new JointSpaceState();

      factory.addState(TrajectoryGeneratorMode.JOINTSPACE, stateWorking);
      factory.addRequestedTransition(TrajectoryGeneratorMode.JOINTSPACE, TrajectoryGeneratorMode.JOINTSPACE, requestedState, false);

      stateMachine = factory.build(TrajectoryGeneratorMode.JOINTSPACE);

      parentRegistry.addChild(registry);
   }

   private class JointSpaceState implements State
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
         return false;
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
      return name;
   }

   @Override
   public String getDescription()
   {
      return registry.getName();
   }

   @Override
   public void doControl()
   {
      stateMachine.doActionAndTransition();
      Set<String> keySet = trajectoryGenerators.keySet();
      for (int i = 0; i < trajectoryGenerators.size(); i++)
      {
         Object key = keySet.toArray()[i];
         MultipleWaypointsTrajectoryGenerator multipleWaypointsTrajectoryGenerator = trajectoryGenerators.get(key);
         double value = multipleWaypointsTrajectoryGenerator.getValue();

         if (multipleWaypointsTrajectoryGenerator.getLastWaypointTime() > stateMachine.getTimeInCurrentState())
         {
            multipleWaypointsTrajectoryGenerator.compute(stateMachine.getTimeInCurrentState());
            value = multipleWaypointsTrajectoryGenerator.getValue();
         }
         else
         {
            SimpleTrajectoryPoint1D lastPoint = new SimpleTrajectoryPoint1D();
            multipleWaypointsTrajectoryGenerator.getLastWaypoint(lastPoint);
            value = lastPoint.getPosition();
         }

         fingerControlSpace.get(key).setValue(value);
      }
   }

   public void setDesired(String controlSpaceName, double time, double delayTime, double goal)
   {
      String controlSpaceNameWithRobotSide = robotSide.getLowerCaseName() + controlSpaceName;
      
      trajectoryGenerators.get(controlSpaceNameWithRobotSide).clear();
      trajectoryGenerators.get(controlSpaceNameWithRobotSide).appendWaypoint(delayTime, fingerControlSpace.get(controlSpaceNameWithRobotSide).getValue(), 0.0);
      trajectoryGenerators.get(controlSpaceNameWithRobotSide).appendWaypoint(delayTime + time, goal, 0.0);
      trajectoryGenerators.get(controlSpaceNameWithRobotSide).initialize();

      requestedState.set(TrajectoryGeneratorMode.JOINTSPACE);
   }

   public double getDesired(String controlSpaceNameWithRobotSide)
   {
      return fingerControlSpace.get(controlSpaceNameWithRobotSide).getValue();
   }
}
