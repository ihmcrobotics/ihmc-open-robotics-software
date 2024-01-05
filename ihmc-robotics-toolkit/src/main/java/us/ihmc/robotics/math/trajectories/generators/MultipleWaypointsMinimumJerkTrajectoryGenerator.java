package us.ihmc.robotics.math.trajectories.generators;

import java.util.ArrayList;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.robotics.math.trajectories.interfaces.DoubleTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.yoVariables.YoPolynomial;
import us.ihmc.robotics.math.trajectories.trajectorypoints.YoOneDoFTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.OneDoFTrajectoryPointBasics;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.TrajectoryPointListBasics;
import us.ihmc.robotics.trajectories.providers.SettableDoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class MultipleWaypointsMinimumJerkTrajectoryGenerator implements DoubleTrajectoryGenerator
{
   public static final int defaultMaximumNumberOfWaypoints = 30;

   private final int maximumNumberOfWaypoints;

   private final String namePrefix;

   private final YoRegistry registry;

   private final YoDouble currentTrajectoryTime;

   private final YoInteger numberOfWaypoints;
   private final YoInteger currentWaypointIndex;

   private final ArrayList<YoOneDoFTrajectoryPoint> waypoints;

   private final SettableDoubleProvider initialPositionProvider = new SettableDoubleProvider();
   private final SettableDoubleProvider initialVelocityProvider = new SettableDoubleProvider();
   private final SettableDoubleProvider initialAccelerationProvider = new SettableDoubleProvider();
   private final SettableDoubleProvider finalPositionProvider = new SettableDoubleProvider();
   private final SettableDoubleProvider finalVelocityProvider = new SettableDoubleProvider();
   private final SettableDoubleProvider finalAccelerationProvider = new SettableDoubleProvider();
   private final SettableDoubleProvider trajectoryTimeProvider = new SettableDoubleProvider();
   private final YoPolynomial subTrajectory;

   public MultipleWaypointsMinimumJerkTrajectoryGenerator(String namePrefix, YoRegistry parentRegistry)
   {
      this(namePrefix, defaultMaximumNumberOfWaypoints, parentRegistry);
   }

   public MultipleWaypointsMinimumJerkTrajectoryGenerator(String namePrefix, int maximumNumberOfWaypoints, YoRegistry parentRegistry)
   {
      this.namePrefix = namePrefix;
      this.maximumNumberOfWaypoints = maximumNumberOfWaypoints;

      registry = new YoRegistry(namePrefix + getClass().getSimpleName());
      parentRegistry.addChild(registry);

      numberOfWaypoints = new YoInteger(namePrefix + "NumberOfWaypoints", registry);
      numberOfWaypoints.set(0);

      currentTrajectoryTime = new YoDouble(namePrefix + "TrajectoryTime", registry);
      currentWaypointIndex = new YoInteger(namePrefix + "CurrentWaypointIndex", registry);

      subTrajectory = new YoPolynomial(namePrefix + "SubTrajectory", 6, registry);

      waypoints = new ArrayList<>(maximumNumberOfWaypoints);

      for (int i = 0; i < maximumNumberOfWaypoints; i++)
      {
         YoOneDoFTrajectoryPoint waypoint = new YoOneDoFTrajectoryPoint(namePrefix, "AtWaypoint" + i, registry);
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

   public void appendWaypoint(double timeAtWaypoint, double position, double velocity, double acceleration)
   {
      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + 1);

      appendWaypointUnsafe(timeAtWaypoint, position, velocity, acceleration);
   }

   private void appendWaypointUnsafe(double timeAtWaypoint, double position, double velocity, double acceleration)
   {
      waypoints.get(numberOfWaypoints.getIntegerValue()).set(timeAtWaypoint, position, velocity, acceleration);

      numberOfWaypoints.increment();
   }

   public void appendWaypoints(double[] timeAtWaypoints, double[] positions, double[] velocities, double[] accelerations)
   {
      if (timeAtWaypoints.length != positions.length || positions.length != velocities.length)
         throw new RuntimeException("Arguments are inconsistent.");

      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + timeAtWaypoints.length);

      for (int i = 0; i < timeAtWaypoints.length; i++)
         appendWaypointUnsafe(timeAtWaypoints[i], positions[i], velocities[i], accelerations[i]);
   }

   public void appendWaypoint(OneDoFTrajectoryPointBasics waypoint1D)
   {
      appendWaypoint(waypoint1D.getTime(), waypoint1D.getPosition(), waypoint1D.getVelocity(), waypoint1D.getAcceleration());
   }

   public void appendWaypoints(OneDoFTrajectoryPointBasics[] waypoints1D)
   {
      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + waypoints1D.length);

      for (int i = 0; i < waypoints1D.length; i++)
         appendWaypointUnsafe(waypoints1D[i].getTime(), waypoints1D[i].getPosition(), waypoints1D[i].getVelocity(), waypoints1D[i].getAcceleration());
   }

   public void appendWaypoints(RecyclingArrayList<? extends OneDoFTrajectoryPointBasics> waypoints1D)
   {
      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + waypoints1D.size());

      for (int i = 0; i < waypoints1D.size(); i++)
         appendWaypointUnsafe(waypoints1D.get(i).getTime(), waypoints1D.get(i).getPosition(), waypoints1D.get(i).getVelocity(), waypoints1D.get(i).getAcceleration());
   }

   public <W extends OneDoFTrajectoryPointBasics> void appendWaypoints(TrajectoryPointListBasics<W> trajectoryWaypoint1DData)
   {
      for (int i = 0; i < trajectoryWaypoint1DData.getNumberOfTrajectoryPoints(); i++)
         appendWaypoint(trajectoryWaypoint1DData.getTrajectoryPoint(i));
   }

   private void checkNumberOfWaypoints(int length)
   {
      if (length > maximumNumberOfWaypoints)
         throw new RuntimeException("Cannot exceed the maximum number of waypoints. Number of waypoints provided: " + length);
   }

   @Override
   public void initialize()
   {
      if (isEmpty())
      {
         throw new RuntimeException("Trajectory has no waypoints.");
      }

      currentWaypointIndex.set(0);

      if (numberOfWaypoints.getIntegerValue() == 1)
      {
         YoOneDoFTrajectoryPoint firstWaypoint = waypoints.get(0);
         finalPositionProvider.setValue(firstWaypoint.getPosition());
         finalVelocityProvider.setValue(firstWaypoint.getVelocity());
         trajectoryTimeProvider.setValue(0.0);
         subTrajectory.setLinear(trajectoryTimeProvider.getValue(), finalPositionProvider.getValue(), finalVelocityProvider.getValue());
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

      if (waypointIndex > 0)
         initialAccelerationProvider.setValue(subTrajectory.getAcceleration());
      finalAccelerationProvider.setValue(0.0);

      double subTrajectoryTime = waypoints.get(waypointIndex + 1).getTime() - waypoints.get(waypointIndex).getTime();
      trajectoryTimeProvider.setValue(subTrajectoryTime);

      subTrajectory.setQuinticWithZeroTerminalAcceleration(0.0,
                                                           trajectoryTimeProvider.getValue(),
                                                           initialPositionProvider.getValue(),
                                                           initialVelocityProvider.getValue(),
                                                           finalPositionProvider.getValue(),
                                                           finalVelocityProvider.getValue());
   }

   @Override
   public void compute(double time)
   {
      currentTrajectoryTime.set(time);

      if (currentWaypointIndex.getIntegerValue() < numberOfWaypoints.getIntegerValue() - 2
            && time >= waypoints.get(currentWaypointIndex.getIntegerValue() + 1).getTime())
      {
         currentWaypointIndex.increment();
         initializeSubTrajectory(currentWaypointIndex.getIntegerValue());
      }

      double subTrajectoryTime;
      if (time < waypoints.get(numberOfWaypoints.getIntegerValue() - 1).getTime())
         subTrajectoryTime = time - waypoints.get(currentWaypointIndex.getIntegerValue()).getTime();
      else
         subTrajectoryTime = waypoints.get(numberOfWaypoints.getIntegerValue() - 1).getTime()
               - waypoints.get(numberOfWaypoints.getIntegerValue() - 2).getTime();
      subTrajectory.compute(subTrajectoryTime);
   }

   @Override
   public boolean isDone()
   {
      if (isEmpty())
         return true;

      boolean isLastWaypoint = currentWaypointIndex.getIntegerValue() >= numberOfWaypoints.getIntegerValue() - 2;
      if (!isLastWaypoint)
         return false;
      boolean subTrajectoryIsDone = subTrajectory.isDone();
      return subTrajectoryIsDone;
   }

   public boolean isEmpty()
   {
      return numberOfWaypoints.getIntegerValue() == 0;
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

   public int getCurrentNumberOfWaypoints()
   {
      return numberOfWaypoints.getIntegerValue();
   }

   public int getMaximumNumberOfWaypoints()
   {
      return maximumNumberOfWaypoints;
   }

   public double getLastWaypointTime()
   {
      return waypoints.get(numberOfWaypoints.getIntegerValue() - 1).getTime();
   }

   @Override
   public String toString()
   {
      if (isEmpty())
         return namePrefix + ": Has no waypoints.";
      else
         return namePrefix + ": number of waypoints = " + numberOfWaypoints.getIntegerValue() + ", current waypoint index = "
               + currentWaypointIndex.getIntegerValue() + "\nFirst waypoint: " + waypoints.get(0) + ", last waypoint: "
               + waypoints.get(numberOfWaypoints.getIntegerValue() - 1);
   }
}
