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

   private final String namePrefix;

   private final YoVariableRegistry registry;

   private final DoubleYoVariable currentTrajectoryTime;

   private final IntegerYoVariable numberOfWaypoints;
   private final IntegerYoVariable currentWaypointIndex;

   private final ArrayList<YoWaypoint1D> waypoints;

   private final SettableDoubleProvider initialPositionProvider = new SettableDoubleProvider();
   private final SettableDoubleProvider initialVelocityProvider = new SettableDoubleProvider();
   private final SettableDoubleProvider finalPositionProvider = new SettableDoubleProvider();
   private final SettableDoubleProvider finalVelocityProvider = new SettableDoubleProvider();
   private final SettableDoubleProvider trajectoryTimeProvider = new SettableDoubleProvider();
   private final CubicPolynomialTrajectoryGenerator subTrajectory;

   public MultipleWaypointsTrajectoryGenerator(String namePrefix, int maximumNumberOfWaypoints, YoVariableRegistry parentRegistry)
   {
      this.namePrefix = namePrefix;
      this.maximumNumberOfWaypoints = maximumNumberOfWaypoints;

      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      parentRegistry.addChild(registry);

      numberOfWaypoints = new IntegerYoVariable(namePrefix + "NumberOfWaypoints", registry);
      numberOfWaypoints.set(0);

      currentTrajectoryTime = new DoubleYoVariable(namePrefix + "TrajectoryTime", registry);
      currentWaypointIndex = new IntegerYoVariable(namePrefix + "CurrentWaypointIndex", registry);

      subTrajectory = new CubicPolynomialTrajectoryGenerator(namePrefix + "SubTrajectory", initialPositionProvider, initialVelocityProvider,
            finalPositionProvider, finalVelocityProvider, trajectoryTimeProvider, registry);

      waypoints = new ArrayList<>(maximumNumberOfWaypoints);

      for (int i = 0; i < maximumNumberOfWaypoints; i++)
      {
         YoWaypoint1D waypoint = new YoWaypoint1D(namePrefix, "AtWaypoint" + i, registry);
         waypoints.add(waypoint);
      }

      clear();
   }

   public void clear()
   {
      numberOfWaypoints.set(0);
      currentWaypointIndex.set(0);

      for (int i = 0; i < maximumNumberOfWaypoints; i++)
      {
         waypoints.get(i).setToNaN();
      }
   }

   public void appendWaypoint(double timeAtWaypoint, double position, double velocity)
   {
      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + 1);

      appendWaypointUnsafe(timeAtWaypoint, position, velocity);
   }

   private void appendWaypointUnsafe(double timeAtWaypoint, double position, double velocity)
   {
      waypoints.get(numberOfWaypoints.getIntegerValue()).set(timeAtWaypoint, position, velocity);

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

      double timeAtFirstWaypoint = waypoints.get(0).getTime();
      for (int i = 0; i < numberOfWaypoints.getIntegerValue(); i++)
      {
         waypoints.get(i).subtractTimeOffset(timeAtFirstWaypoint);
      }

      if (numberOfWaypoints.getIntegerValue() == 1)
      {
         finalPositionProvider.setValue(waypoints.get(0).getPosition());
         finalVelocityProvider.setValue(waypoints.get(0).getVelocity());
         trajectoryTimeProvider.setValue(0.0);
         subTrajectory.initialize();
      }
      else
         initializeSubTrajectory(0);
   }

   private void initializeSubTrajectory(int waypointIndex)
   {
      initialPositionProvider.setValue(waypoints.get(waypointIndex).getPosition());
      initialVelocityProvider.setValue(waypoints.get(waypointIndex).getVelocity());

      finalPositionProvider.setValue(waypoints.get(waypointIndex + 1).getPosition());
      finalVelocityProvider.setValue(waypoints.get(waypointIndex + 1).getVelocity());

      double subTrajectoryTime = waypoints.get(waypointIndex + 1).getTime() - waypoints.get(waypointIndex).getTime();
      trajectoryTimeProvider.setValue(subTrajectoryTime);

      subTrajectory.initialize();
   }

   @Override
   public void compute(double time)
   {
      currentTrajectoryTime.set(time);

      if (currentWaypointIndex.getIntegerValue() < numberOfWaypoints.getIntegerValue() - 2 && time >= waypoints.get(currentWaypointIndex.getIntegerValue() + 1).getTime())
      {
         currentWaypointIndex.increment();
         initializeSubTrajectory(currentWaypointIndex.getIntegerValue());
      }

      double subTrajectoryTime = time - waypoints.get(currentWaypointIndex.getIntegerValue()).getTime();
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

   @Override
   public String toString()
   {
      if (numberOfWaypoints.getIntegerValue() == 0)
         return namePrefix + ": Has no waypoints.";
      else
         return namePrefix + ": number of waypoints = " + numberOfWaypoints.getIntegerValue() + ", current waypoint index = " + currentWaypointIndex.getIntegerValue()
         + "\nFirst waypoint: " + waypoints.get(0) + ", last waypoint: " + waypoints.get(numberOfWaypoints.getIntegerValue() - 1);
   }
}
