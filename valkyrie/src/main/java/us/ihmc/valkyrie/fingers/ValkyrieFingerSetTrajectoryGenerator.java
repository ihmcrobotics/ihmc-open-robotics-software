package us.ihmc.valkyrie.fingers;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.OneDoFTrajectoryPoint;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.robotics.trajectories.providers.SettableDoubleProvider;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class ValkyrieFingerSetTrajectoryGenerator<T extends Enum<T>> implements RobotController
{
   private final String name;
   private final YoRegistry registry;

   private final RobotSide robotSide;

   private final EnumMap<T, SettableDoubleProvider> desiredQs;
   private final EnumMap<T, SettableDoubleProvider> desiredQds;
   private final EnumMap<T, MultipleWaypointsTrajectoryGenerator> trajectoryGenerators;

   private final List<T> controlledFingerJoints;

   private final EnumMap<T, TDoubleArrayList> wayPointPositions;
   private final EnumMap<T, TDoubleArrayList> wayPointTimes;

   private final StateMachine<TrajectoryGeneratorMode, State> stateMachine;
   private final YoEnum<TrajectoryGeneratorMode> requestedState;

   enum TrajectoryGeneratorMode
   {
      JOINTSPACE
   }

   public ValkyrieFingerSetTrajectoryGenerator(Class<T> enumType, RobotSide robotSide, YoDouble yoTime, EnumMap<T, YoDouble> fingerControlSpaceMap,
                                               YoRegistry parentRegistry)
   {
      this.robotSide = robotSide;
      this.name = robotSide.getLowerCaseName() + enumType.getSimpleName();

      this.registry = new YoRegistry(name);

      this.desiredQs = new EnumMap<>(enumType);
      this.desiredQds = new EnumMap<>(enumType);
      this.trajectoryGenerators = new EnumMap<>(enumType);
      this.controlledFingerJoints = new ArrayList<T>();

      this.wayPointPositions = new EnumMap<>(enumType);
      this.wayPointTimes = new EnumMap<>(enumType);

      T[] enumConstants = enumType.getEnumConstants();
      for (int i = 0; i < enumConstants.length; i++)
      {
         T key = enumConstants[i];
         double initialValue = fingerControlSpaceMap.get(key).getDoubleValue();
         desiredQs.put(key, new SettableDoubleProvider(initialValue));
         desiredQds.put(key, new SettableDoubleProvider(0.0));

         MultipleWaypointsTrajectoryGenerator trajectoryGenerator = new MultipleWaypointsTrajectoryGenerator(robotSide + key.name() + "_traj", parentRegistry);
         trajectoryGenerator.clear();
         trajectoryGenerator.appendWaypoint(yoTime.getDoubleValue(), initialValue, 0.0);
         trajectoryGenerator.initialize();
         trajectoryGenerators.put(key, trajectoryGenerator);

         controlledFingerJoints.add(key);

         wayPointPositions.put(key, new TDoubleArrayList());
         wayPointTimes.put(key, new TDoubleArrayList());
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

   private void setTrajectories()
   {
      for (int i = 0; i < controlledFingerJoints.size(); i++)
      {
         T key = controlledFingerJoints.get(i);
         trajectoryGenerators.get(key).clear();
         trajectoryGenerators.get(key).appendWaypoint(0.0, desiredQs.get(key).getValue(), 0.0);
         TDoubleArrayList wayPointPositions = this.wayPointPositions.get(key);
         TDoubleArrayList wayPointTimes = this.wayPointTimes.get(key);
         for (int j = 0; j < wayPointPositions.size(); j++)
         {
            trajectoryGenerators.get(key).appendWaypoint(wayPointTimes.get(j), wayPointPositions.get(j), 0.0);
         }
      }
   }

   /**
    *  setting up joint trajectory is in order of methods,
    *  <p>clearTrajectories()
    *  <p>setDelay() <- optional
    *  <p>appendWayPoint() or appendStopPoint()
    *  <p>executeTrajectories()
    * @author shadylady
    */
   private class JointSpaceState implements State
   {
      @Override
      public void onEntry()
      {
         setTrajectories();
         for (int i = 0; i < controlledFingerJoints.size(); i++)
         {
            T key = controlledFingerJoints.get(i);
            trajectoryGenerators.get(key).initialize();
         }
      }

      private final OneDoFTrajectoryPoint lastPoint = new OneDoFTrajectoryPoint();

      @Override
      public void doAction(double timeInState)
      {
         for (int i = 0; i < controlledFingerJoints.size(); i++)
         {
            T key = controlledFingerJoints.get(i);
            MultipleWaypointsTrajectoryGenerator multipleWaypointsTrajectoryGenerator = trajectoryGenerators.get(key);
            double desiredQ = multipleWaypointsTrajectoryGenerator.getValue();
            double desiredQd = multipleWaypointsTrajectoryGenerator.getVelocity();

            if (multipleWaypointsTrajectoryGenerator.getLastWaypointTime() > timeInState)
            {
               multipleWaypointsTrajectoryGenerator.compute(timeInState);
               desiredQ = multipleWaypointsTrajectoryGenerator.getValue();
               desiredQd = multipleWaypointsTrajectoryGenerator.getVelocity();
            }
            else
            {
               multipleWaypointsTrajectoryGenerator.getLastWaypoint(lastPoint);
               desiredQ = lastPoint.getPosition();
               desiredQd = lastPoint.getVelocity();
            }

            desiredQs.get(key).setValue(desiredQ);
            desiredQds.get(key).setValue(desiredQd);

         }
      }

      @Override
      public boolean isDone(double timeInState)
      {
         for (int i = 0; i < controlledFingerJoints.size(); i++)
         {
            T key = controlledFingerJoints.get(i);
            if (!trajectoryGenerators.get(key).isDone())
               return false;
         }
         return true;
      }

      @Override
      public void onExit()
      {

      }
   }

   @Override
   public void initialize()
   {
      stateMachine.resetToInitialState();
   }

   @Override
   public YoRegistry getYoRegistry()
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
   }

   public void executeTrajectories()
   {
      requestedState.set(TrajectoryGeneratorMode.JOINTSPACE);
   }

   public void clearTrajectories()
   {
      for (int i = 0; i < controlledFingerJoints.size(); i++)
      {
         T key = controlledFingerJoints.get(i);
         wayPointPositions.get(key).clear();
         wayPointTimes.get(key).clear();
      }
   }

   public void appendWayPoint(T controlSpace, double time, double goal)
   {
      wayPointPositions.get(controlSpace).add(goal);
      wayPointTimes.get(controlSpace).add(time);
   }

   public void appendStopPoint(T controlSpace, double current)
   {
      wayPointPositions.get(controlSpace).add(current);
      wayPointTimes.get(controlSpace).add(0.0);
   }

   public double getDesired(T controlSpace)
   {
      return desiredQs.get(controlSpace).getValue();
   }

   public double getDesiredVelocity(T controlSpace)
   {
      return desiredQds.get(controlSpace).getValue();
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }
}
