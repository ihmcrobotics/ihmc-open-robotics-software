package us.ihmc.robotics.math.trajectories.generators;

import java.util.ArrayList;

import org.apache.commons.math3.util.Precision;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.robotics.math.trajectories.interfaces.DoubleTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.YoFrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.yoVariables.YoPolynomial;
import us.ihmc.robotics.math.trajectories.trajectorypoints.YoOneDoFTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.OneDoFTrajectoryPointBasics;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.TrajectoryPointListBasics;
import us.ihmc.robotics.trajectories.providers.SettableDoubleProvider;
import us.ihmc.yoVariables.euclid.referenceFrame.YoMutableFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoMutableFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class MultipleWaypointsTrajectoryGenerator implements DoubleTrajectoryGenerator
{
   public static final int defaultMaximumNumberOfWaypoints = 30;

   private final int maximumNumberOfWaypoints;

   private final String namePrefix;

   private final YoRegistry registry;

   private final YoDouble currentTrajectoryTime;

   private final YoInteger numberOfWaypoints;
   private final YoInteger currentWaypointIndex;

   private final ArrayList<YoOneDoFTrajectoryPoint> waypoints;

   private final YoDouble currentPosition;
   private final YoDouble currentVelocity;
   private final YoDouble currentAcceleration;
   private final YoPolynomial subTrajectory;

   public MultipleWaypointsTrajectoryGenerator(String namePrefix, YoRegistry parentRegistry)
   {
      this(namePrefix, defaultMaximumNumberOfWaypoints, parentRegistry);
   }

   public MultipleWaypointsTrajectoryGenerator(String namePrefix, int maximumNumberOfWaypoints, YoRegistry parentRegistry)
   {
      this.namePrefix = namePrefix;
      this.maximumNumberOfWaypoints = maximumNumberOfWaypoints;

      registry = new YoRegistry(namePrefix + getClass().getSimpleName());
      parentRegistry.addChild(registry);

      numberOfWaypoints = new YoInteger(namePrefix + "NumberOfWaypoints", registry);
      numberOfWaypoints.set(0);

      currentTrajectoryTime = new YoDouble(namePrefix + "TrajectoryTime", registry);
      currentWaypointIndex = new YoInteger(namePrefix + "CurrentWaypointIndex", registry);

      waypoints = new ArrayList<>(maximumNumberOfWaypoints);

      for (int i = 0; i < maximumNumberOfWaypoints; i++)
      {
         YoOneDoFTrajectoryPoint waypoint = new YoOneDoFTrajectoryPoint(namePrefix, "AtWaypoint" + i, registry);
         waypoints.add(waypoint);
      }

      namePrefix += "SubTrajectory";
      String currentPositionName = namePrefix + "CurrentPosition";
      String currentVelocityName = namePrefix + "CurrentVelocity";
      String currentAccelerationName = namePrefix + "CurrentAcceleration";

      currentPosition = new YoDouble(currentPositionName, registry);
      currentVelocity = new YoDouble(currentVelocityName, registry);
      currentAcceleration = new YoDouble(currentAccelerationName, registry);

      subTrajectory = new YoPolynomial(namePrefix, 4, registry);

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

   public void appendWaypoint(OneDoFTrajectoryPointBasics waypoint1D)
   {
      appendWaypoint(waypoint1D.getTime(), waypoint1D.getPosition(), waypoint1D.getVelocity());
   }

   public void appendWaypoints(OneDoFTrajectoryPointBasics[] waypoints1D)
   {
      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + waypoints1D.length);

      for (int i = 0; i < waypoints1D.length; i++)
         appendWaypointUnsafe(waypoints1D[i].getTime(), waypoints1D[i].getPosition(), waypoints1D[i].getVelocity());
   }

   public void appendWaypoints(RecyclingArrayList<? extends OneDoFTrajectoryPointBasics> waypoints1D)
   {
      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + waypoints1D.size());

      for (int i = 0; i < waypoints1D.size(); i++)
         appendWaypointUnsafe(waypoints1D.get(i).getTime(), waypoints1D.get(i).getPosition(), waypoints1D.get(i).getVelocity());
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
         subTrajectory.setConstant(waypoints.get(0).getPosition());
      }
      else
      {
         initializeSubTrajectory(0);
      }
   }

   private void initializeSubTrajectory(int waypointIndex)
   {
      int secondWaypointIndex = Math.min(waypointIndex + 1, numberOfWaypoints.getValue() - 1);
      YoOneDoFTrajectoryPoint start = waypoints.get(waypointIndex);
      YoOneDoFTrajectoryPoint end = waypoints.get(secondWaypointIndex);

      double subTrajectoryDuration = end.getTime() - start.getTime();
      subTrajectory.setCubic(0.0,
                             subTrajectoryDuration,
                             start.getPosition(),
                             start.getVelocity(),
                             end.getPosition(),
                             end.getVelocity());
   }

   @Override
   public void compute(double time)
   {
      if (isEmpty())
      {
         throw new RuntimeException("Can not call compute on an empty trajectory.");
      }

      currentTrajectoryTime.set(time);
      boolean changedSubTrajectory = false;

      if (currentWaypointIndex.getIntegerValue() > numberOfWaypoints.getIntegerValue() - 1
          && time < waypoints.get(currentWaypointIndex.getIntegerValue()).getTime())
      {
         currentWaypointIndex.set(0);
         changedSubTrajectory = true;
      }

      while (currentWaypointIndex.getIntegerValue() < numberOfWaypoints.getIntegerValue() - 2
            && time >= waypoints.get(currentWaypointIndex.getIntegerValue() + 1).getTime())
      {
         currentWaypointIndex.increment();
         changedSubTrajectory = true;
      }

      if (changedSubTrajectory)
      {
         initializeSubTrajectory(currentWaypointIndex.getIntegerValue());
      }

      int secondWaypointIndex = Math.min(currentWaypointIndex.getValue() + 1, numberOfWaypoints.getValue() - 1);
      YoOneDoFTrajectoryPoint start = waypoints.get(currentWaypointIndex.getValue());
      YoOneDoFTrajectoryPoint end = waypoints.get(secondWaypointIndex);

      if (time < start.getTime())
      {
         currentPosition.set(start.getPosition());
         currentVelocity.set(0.0);
         currentAcceleration.set(0.0);
         return;
      }
      if (time > end.getTime())
      {
         currentPosition.set(end.getPosition());
         currentVelocity.set(0.0);
         currentAcceleration.set(0.0);
         return;
      }

      if (Precision.equals(start.getTime(), end.getTime()))
      {
         currentPosition.set(start.getPosition());
         currentVelocity.set(start.getVelocity());
         currentAcceleration.set(0.0);
         return;
      }

      double subTrajectoryTime = time - waypoints.get(currentWaypointIndex.getIntegerValue()).getTime();
      subTrajectory.compute(subTrajectoryTime);

      currentPosition.set(subTrajectory.getValue());
      currentVelocity.set(subTrajectory.getVelocity());
      currentAcceleration.set(subTrajectory.getAcceleration());
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
      return currentPosition.getValue();
   }

   @Override
   public double getVelocity()
   {
      return currentVelocity.getDoubleValue();
   }

   @Override
   public double getAcceleration()
   {
      return currentAcceleration.getDoubleValue();
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

   public void getLastWaypoint(OneDoFTrajectoryPointBasics pointToPack)
   {
      YoOneDoFTrajectoryPoint lastWaypoint = waypoints.get(numberOfWaypoints.getIntegerValue() - 1);
      pointToPack.setPosition(lastWaypoint.getPosition());
      pointToPack.setVelocity(lastWaypoint.getVelocity());
      pointToPack.setTime(lastWaypoint.getTime());
   }

   public double getFirstWaypointTime()
   {
      return waypoints.get(0).getTime();
   }

   public void getFirstWaypoint(OneDoFTrajectoryPointBasics pointToPack)
   {
      YoOneDoFTrajectoryPoint firstWaypoint = waypoints.get(0);
      pointToPack.setPosition(firstWaypoint.getPosition());
      pointToPack.setVelocity(firstWaypoint.getVelocity());
      pointToPack.setTime(firstWaypoint.getTime());
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
