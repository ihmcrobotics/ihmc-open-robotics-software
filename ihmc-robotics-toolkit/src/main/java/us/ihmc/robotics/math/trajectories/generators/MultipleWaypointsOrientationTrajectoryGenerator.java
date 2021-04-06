package us.ihmc.robotics.math.trajectories.generators;

import static us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsTrajectoryGenerator.defaultMaximumNumberOfWaypoints;

import java.util.ArrayList;

import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.math.trajectories.HermiteCurveBasedOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.OrientationTrajectoryGeneratorInMultipleFrames;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameSO3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.YoFrameSO3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.FrameSO3TrajectoryPointBasics;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.SO3TrajectoryPointBasics;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.TrajectoryPointListBasics;
import us.ihmc.robotics.math.trajectories.trajectorypoints.lists.FrameSO3TrajectoryPointList;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class MultipleWaypointsOrientationTrajectoryGenerator extends OrientationTrajectoryGeneratorInMultipleFrames
{
   private final String namePrefix;

   private final int maximumNumberOfWaypoints;

   private final YoRegistry registry;

   private final YoDouble currentTrajectoryTime;

   private final YoInteger numberOfWaypoints;
   private final YoInteger currentWaypointIndex;
   private final ArrayList<YoFrameSO3TrajectoryPoint> waypoints;

   private final HermiteCurveBasedOrientationTrajectoryGenerator subTrajectory;

   public MultipleWaypointsOrientationTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, YoRegistry parentRegistry)
   {
      this(namePrefix, defaultMaximumNumberOfWaypoints, referenceFrame, parentRegistry);
   }

   public MultipleWaypointsOrientationTrajectoryGenerator(String namePrefix, int maximumNumberOfWaypoints, ReferenceFrame referenceFrame,
                                                          YoRegistry parentRegistry)
   {
      this.namePrefix = namePrefix;
      this.maximumNumberOfWaypoints = maximumNumberOfWaypoints;

      registry = new YoRegistry(namePrefix + getClass().getSimpleName());

      numberOfWaypoints = new YoInteger(namePrefix + "NumberOfWaypoints", registry);
      numberOfWaypoints.set(0);

      waypoints = new ArrayList<>(maximumNumberOfWaypoints);

      currentTrajectoryTime = new YoDouble(namePrefix + "CurrentTrajectoryTime", registry);
      currentWaypointIndex = new YoInteger(namePrefix + "CurrentWaypointIndex", registry);

      subTrajectory = new HermiteCurveBasedOrientationTrajectoryGenerator(namePrefix + "SubTrajectory", referenceFrame, registry);
      registerFrameChangeables(subTrajectory);

      for (int i = 0; i < maximumNumberOfWaypoints; i++)
      {
         YoFrameSO3TrajectoryPoint waypoint = new YoFrameSO3TrajectoryPoint(namePrefix, "AtWaypoint" + i, registry);
         registerFrameChangeables(waypoint);
         waypoints.add(waypoint);
      }

      clear(referenceFrame);

      parentRegistry.addChild(registry);
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

   public void clear(ReferenceFrame referenceFrame)
   {
      clear();
      setReferenceFrame(referenceFrame);
   }

   public void appendWaypoint(double timeAtWaypoint, QuaternionReadOnly orientation, Vector3DReadOnly angularVelocity)
   {
      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + 1);
      appendWaypointUnsafe(timeAtWaypoint, orientation, angularVelocity);
   }

   private void appendWaypointUnsafe(double timeAtWaypoint, QuaternionReadOnly orientation, Vector3DReadOnly angularVelocity)
   {
      waypoints.get(numberOfWaypoints.getIntegerValue()).set(timeAtWaypoint, orientation, angularVelocity);
      numberOfWaypoints.increment();
   }

   public void appendWaypoint(double timeAtWaypoint, FrameQuaternionReadOnly orientation, FrameVector3DReadOnly angularVelocity)
   {
      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + 1);
      appendWaypointUnsafe(timeAtWaypoint, orientation, angularVelocity);
   }

   private void appendWaypointUnsafe(double timeAtWaypoint, FrameQuaternionReadOnly orientation, FrameVector3DReadOnly angularVelocity)
   {
      waypoints.get(numberOfWaypoints.getIntegerValue()).set(timeAtWaypoint, orientation, angularVelocity);
      numberOfWaypoints.increment();
   }

   public void appendWaypoint(SO3TrajectoryPointBasics so3Waypoint)
   {
      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + 1);
      appendWaypointUnsafe(so3Waypoint);
   }

   public void appendWaypoint(FrameSO3TrajectoryPoint frameSO3TrajectoryPoint)
   {
      checkReferenceFrameMatch(frameSO3TrajectoryPoint);
      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + 1);
      appendWaypointUnsafe(frameSO3TrajectoryPoint);
   }

   public void appendWaypoint(FrameSE3TrajectoryPoint frameSE3TrajectoryPoint)
   {
      checkReferenceFrameMatch(frameSE3TrajectoryPoint);
      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + 1);
      appendWaypointUnsafe(frameSE3TrajectoryPoint);
   }

   private void appendWaypointUnsafe(SO3TrajectoryPointBasics so3Waypoint)
   {
      waypoints.get(numberOfWaypoints.getIntegerValue()).set(so3Waypoint);
      numberOfWaypoints.increment();
   }

   public void appendWaypoints(double[] timeAtWaypoints, QuaternionReadOnly[] orientations, Vector3DReadOnly[] angularVelocities)
   {
      if (timeAtWaypoints.length != orientations.length || angularVelocities != null && orientations.length != angularVelocities.length)
         throw new RuntimeException("Arguments are inconsistent.");

      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + timeAtWaypoints.length);

      for (int i = 0; i < timeAtWaypoints.length; i++)
         appendWaypointUnsafe(timeAtWaypoints[i], orientations[i], angularVelocities[i]);
   }

   public void appendWaypoints(double[] timeAtWaypoints, FrameQuaternionReadOnly[] orientations, FrameVector3DReadOnly[] angularVelocities)
   {
      if (timeAtWaypoints.length != orientations.length || orientations.length != angularVelocities.length)
         throw new RuntimeException("Arguments are inconsistent.");

      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + timeAtWaypoints.length);

      for (int i = 0; i < timeAtWaypoints.length; i++)
      {
         appendWaypointUnsafe(timeAtWaypoints[i], orientations[i], angularVelocities[i]);
      }
   }

   public void appendWaypoints(SO3TrajectoryPointBasics[] so3Waypoints)
   {
      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + so3Waypoints.length);

      for (int i = 0; i < so3Waypoints.length; i++)
      {
         appendWaypointUnsafe(so3Waypoints[i]);
      }
   }

   public void appendWaypoints(TrajectoryPointListBasics<? extends SO3TrajectoryPointBasics> trajectoryPointList)
   {
      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + trajectoryPointList.getNumberOfTrajectoryPoints());

      for (int i = 0; i < trajectoryPointList.getNumberOfTrajectoryPoints(); i++)
         appendWaypointUnsafe(trajectoryPointList.getTrajectoryPoint(i));
   }

   public void appendWaypoints(FrameSO3TrajectoryPointList trajectoryPointList)
   {
      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + trajectoryPointList.getNumberOfTrajectoryPoints());
      checkReferenceFrameMatch(trajectoryPointList);

      for (int i = 0; i < trajectoryPointList.getNumberOfTrajectoryPoints(); i++)
      {
         FrameSO3TrajectoryPoint trajectoryPoint = trajectoryPointList.getTrajectoryPoint(i);
         checkReferenceFrameMatch(trajectoryPoint);
         appendWaypointUnsafe(trajectoryPoint);
      }
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
      initializeSubTrajectory(0);
   }

   private void initializeSubTrajectory(int waypointIndex)
   {
      int secondWaypointIndex = Math.min(waypointIndex + 1, numberOfWaypoints.getValue() - 1);
      subTrajectory.setTrajectoryParameters(waypoints.get(waypointIndex), waypoints.get(secondWaypointIndex));
      subTrajectory.initialize();
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

      if (time < waypoints.get(currentWaypointIndex.getIntegerValue()).getTime())
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

      double subTrajectoryTime = time - waypoints.get(currentWaypointIndex.getIntegerValue()).getTime();
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

   public int getCurrentWaypointIndex()
   {
      return currentWaypointIndex.getIntegerValue();
   }

   @Override
   public FrameQuaternionReadOnly getOrientation()
   {
      return subTrajectory.getOrientation();
   }

   @Override
   public FrameVector3DReadOnly getAngularVelocity()
   {
      return subTrajectory.getAngularVelocity();
   }

   @Override
   public FrameVector3DReadOnly getAngularAcceleration()
   {
      return subTrajectory.getAngularAcceleration();
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

   public void getLastWaypoint(FrameSO3TrajectoryPointBasics pointToPack)
   {
      pointToPack.set(waypoints.get(numberOfWaypoints.getIntegerValue() - 1));
   }

   public FrameSO3TrajectoryPointBasics getWaypoint(int index)
   {
      return waypoints.get(index);
   }

   public void removeLastWaypoint()
   {
      waypoints.get(numberOfWaypoints.getIntegerValue() - 1).setToNaN();
      numberOfWaypoints.decrement();
   }

   @Override
   public String toString()
   {
      if (numberOfWaypoints.getIntegerValue() == 0)
         return namePrefix + ": Has no waypoints.";
      else
         return namePrefix + ": number of waypoints = " + numberOfWaypoints.getIntegerValue() + ", current waypoint index = "
               + currentWaypointIndex.getIntegerValue() + "\nFirst waypoint: " + waypoints.get(0) + ", last waypoint: "
               + waypoints.get(numberOfWaypoints.getIntegerValue() - 1);
   }
}
