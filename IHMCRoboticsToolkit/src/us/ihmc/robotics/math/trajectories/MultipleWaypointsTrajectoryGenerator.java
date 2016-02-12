package us.ihmc.robotics.math.trajectories;

import java.util.ArrayList;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.trajectories.providers.SettableDoubleProvider;

/**
 * This class does a cubic interpolation between the provided waypoints.
 * 
 * Procedure for use:
 * 1. setWaypoints(double[] timeAtWaypoints, double[] positions, double[] velocities)
 *    clears the generator and appends the given waypoints
 * 2. setInitialCondition(double initialPosition, double initialVelocity)
 *    informs the generator of the initial position and velocity
 * 3. compute(double time)
 * 4. getValue(), getVelocity, and getAcceleration()
 */
public class MultipleWaypointsTrajectoryGenerator implements DoubleTrajectoryGenerator
{
   private final int maximumNumberOfWaypoints;

   private final YoVariableRegistry registry;

   private final DoubleYoVariable currentTrajectoryTime;

   private final IntegerYoVariable numberOfWaypoints;
   private final IntegerYoVariable currentWaypointIndex;

   private final ArrayList<DoubleYoVariable> timeAtWaypoints;
   private final ArrayList<DoubleYoVariable> positionAtWaypoints;
   private final ArrayList<DoubleYoVariable> velocityAtWaypoints;

   private final SettableDoubleProvider initialPositionProvider = new SettableDoubleProvider();
   private final SettableDoubleProvider initialVelocityProvider = new SettableDoubleProvider();
   private final SettableDoubleProvider finalPositionProvider = new SettableDoubleProvider();
   private final SettableDoubleProvider finalVelocityProvider = new SettableDoubleProvider();
   private final SettableDoubleProvider trajectoryTimeProvider = new SettableDoubleProvider();
   private final CubicPolynomialTrajectoryGenerator subTrajectory;

   public MultipleWaypointsTrajectoryGenerator(String namePrefix, int maximumNumberOfWaypoints, YoVariableRegistry parentRegistry)
   {
      this.maximumNumberOfWaypoints = maximumNumberOfWaypoints;

      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      parentRegistry.addChild(registry);

      numberOfWaypoints = new IntegerYoVariable(namePrefix + "NumberOfWaypoints", registry);
      numberOfWaypoints.set(0);

      currentTrajectoryTime = new DoubleYoVariable(namePrefix + "TrajectoryTime", registry);
      currentWaypointIndex = new IntegerYoVariable(namePrefix + "CurrentWaypointIndex", registry);

      subTrajectory = new CubicPolynomialTrajectoryGenerator(namePrefix + "SubTrajectory", initialPositionProvider, initialVelocityProvider,
            finalPositionProvider, finalVelocityProvider, trajectoryTimeProvider, registry);

      timeAtWaypoints = new ArrayList<>(maximumNumberOfWaypoints);
      positionAtWaypoints = new ArrayList<>(maximumNumberOfWaypoints);
      velocityAtWaypoints = new ArrayList<>(maximumNumberOfWaypoints);

      for (int i = 0; i < maximumNumberOfWaypoints; i++)
      {
         timeAtWaypoints.add(new DoubleYoVariable(namePrefix + "TimeAtWaypoint" + i, registry));
         positionAtWaypoints.add(new DoubleYoVariable(namePrefix + "PositionAtWaypoint" + i, registry));
         velocityAtWaypoints.add(new DoubleYoVariable(namePrefix + "VelocityAtWaypoint" + i, registry));
      }

      clear();
   }

   public void clear()
   {
      numberOfWaypoints.set(0);
      currentWaypointIndex.set(0);

      for (int i = 0; i < maximumNumberOfWaypoints; i++)
      {
         timeAtWaypoints.get(i).set(Double.NaN);
         positionAtWaypoints.get(i).set(Double.NaN);
         velocityAtWaypoints.get(i).set(Double.NaN);
      }
   }

   public void appendWaypoint(double timeAtWaypoint, double position, double velocity)
   {
      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + 1);

      appendWaypointUnsafe(timeAtWaypoint, position, velocity);
   }

   private void appendWaypointUnsafe(double timeAtWaypoint, double position, double velocity)
   {
      timeAtWaypoints.get(numberOfWaypoints.getIntegerValue()).set(timeAtWaypoint);
      positionAtWaypoints.get(numberOfWaypoints.getIntegerValue()).set(position);
      velocityAtWaypoints.get(numberOfWaypoints.getIntegerValue()).set(velocity);

      numberOfWaypoints.increment();
   }

   public void appendWaypoints(double[] timeAtWaypoints, double[] positions, double[] velocities)
   {
      if (timeAtWaypoints.length != positions.length || positions.length != velocities.length)
         throw new RuntimeException("Arguments are inconsistent.");

      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + timeAtWaypoints.length);

      for (int i = 0; i < timeAtWaypoints.length; i++)
         appendWaypointUnsafe(timeAtWaypoints[i], positions[i], velocities[i]);
   }

   public void appendWaypoint(Waypoint1DInterface waypoint1D)
   {
      appendWaypoint(waypoint1D.getTime(), waypoint1D.getPosition(), waypoint1D.getVelocity());
   }

   public void appendWaypoints(Waypoint1DInterface[] waypoints1D)
   {
      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + waypoints1D.length);

      for (int i = 0; i < waypoints1D.length; i++)
         appendWaypointUnsafe(waypoints1D[i].getTime(), waypoints1D[i].getPosition(), waypoints1D[i].getVelocity());
   }

   public void appendWaypoints(TrajectoryWaypoint1DDataInterface trajectoryWaypoint1DData)
   {
      appendWaypoints(trajectoryWaypoint1DData.getWaypoints());
   }

   private void checkNumberOfWaypoints(int length)
   {
      if (length > maximumNumberOfWaypoints)
         throw new RuntimeException("Cannot exceed the maximum number of waypoints. Number of waypoints provided: " + length);
   }

   @Override
   public void initialize()
   {
      if (numberOfWaypoints.getIntegerValue() == 0)
      {
         throw new RuntimeException("Trajectory has no waypoints.");
      }

      currentWaypointIndex.set(0);

      double timeAtFirstWaypoint = timeAtWaypoints.get(0).getDoubleValue();
      for (int i = 0; i < numberOfWaypoints.getIntegerValue(); i++)
      {
         timeAtWaypoints.get(i).sub(timeAtFirstWaypoint);
      }

      if (numberOfWaypoints.getIntegerValue() == 1)
      {
         finalPositionProvider.setValue(positionAtWaypoints.get(0).getDoubleValue());
         finalVelocityProvider.setValue(velocityAtWaypoints.get(0).getDoubleValue());
         trajectoryTimeProvider.setValue(0.0);
         subTrajectory.initialize();
      }
      else
         initializeSubTrajectory(0);
   }

   private void initializeSubTrajectory(int waypointIndex)
   {
      initialPositionProvider.setValue(positionAtWaypoints.get(waypointIndex).getDoubleValue());
      initialVelocityProvider.setValue(velocityAtWaypoints.get(waypointIndex).getDoubleValue());

      finalPositionProvider.setValue(positionAtWaypoints.get(waypointIndex + 1).getDoubleValue());
      finalVelocityProvider.setValue(velocityAtWaypoints.get(waypointIndex + 1).getDoubleValue());

      double subTrajectoryTime = timeAtWaypoints.get(waypointIndex + 1).getDoubleValue() - timeAtWaypoints.get(waypointIndex).getDoubleValue();
      trajectoryTimeProvider.setValue(subTrajectoryTime);

      subTrajectory.initialize();
   }

   @Override
   public void compute(double time)
   {
      currentTrajectoryTime.set(time);

      if (currentWaypointIndex.getIntegerValue() < numberOfWaypoints.getIntegerValue() - 2 && time >= timeAtWaypoints.get(currentWaypointIndex.getIntegerValue() + 1).getDoubleValue())
      {
         currentWaypointIndex.increment();
         initializeSubTrajectory(currentWaypointIndex.getIntegerValue());
      }

      double subTrajectoryTime = time - timeAtWaypoints.get(currentWaypointIndex.getIntegerValue()).getDoubleValue();
      subTrajectory.compute(subTrajectoryTime);
   }

   @Override
   public boolean isDone()
   {
      if (numberOfWaypoints.getIntegerValue() == 0)
         return true;

      boolean isLastWaypoint = currentWaypointIndex.getIntegerValue() >= numberOfWaypoints.getIntegerValue() - 2;
      if (!isLastWaypoint)
         return false;
      boolean subTrajectoryIsDone = subTrajectory.isDone();
      return subTrajectoryIsDone;
   }

   @Override
   public double getValue()
   {
      return subTrajectory.getValue();
   }

   @Override
   public double getVelocity()
   {
      return subTrajectory.getVelocity();
   }

   @Override
   public double getAcceleration()
   {
      return subTrajectory.getAcceleration();
   }
}
