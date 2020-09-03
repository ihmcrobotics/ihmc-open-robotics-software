package us.ihmc.robotics.math.trajectories.generators;

import static us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsTrajectoryGenerator.defaultMaximumNumberOfWaypoints;

import java.util.ArrayList;

import org.apache.commons.math3.util.Precision;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.math.trajectories.PositionTrajectoryGeneratorInMultipleFrames;
import us.ihmc.robotics.math.trajectories.YoPolynomial3D;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.YoFrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.EuclideanTrajectoryPointBasics;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.TrajectoryPointListBasics;
import us.ihmc.yoVariables.euclid.referenceFrame.YoMutableFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoMutableFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class MultipleWaypointsPositionTrajectoryGenerator extends PositionTrajectoryGeneratorInMultipleFrames
{
   private final String namePrefix;

   private final int maximumNumberOfWaypoints;

   private final YoRegistry registry;

   private final YoDouble currentTrajectoryTime;

   private final YoInteger numberOfWaypoints;
   private final YoInteger currentWaypointIndex;
   private final ArrayList<YoFrameEuclideanTrajectoryPoint> waypoints;

   private final FramePoint3DBasics currentPosition;
   private final FrameVector3DBasics currentVelocity;
   private final FrameVector3DBasics currentAcceleration;
   private final YoPolynomial3D subTrajectory;

   public MultipleWaypointsPositionTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, YoRegistry parentRegistry)
   {
      this(namePrefix, defaultMaximumNumberOfWaypoints, referenceFrame, parentRegistry);
   }

   public MultipleWaypointsPositionTrajectoryGenerator(String namePrefix, int maximumNumberOfWaypoints, ReferenceFrame referenceFrame,
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

      String currentPositionName = namePrefix + "CurrentPosition";
      String currentVelocityName = namePrefix + "CurrentVelocity";
      String currentAccelerationName = namePrefix + "CurrentAcceleration";

      currentPosition = new YoMutableFramePoint3D(currentPositionName, "", registry, referenceFrame);
      currentVelocity = new YoMutableFrameVector3D(currentVelocityName, "", registry, referenceFrame);
      currentAcceleration = new YoMutableFrameVector3D(currentAccelerationName, "", registry, referenceFrame);

      registerFrameChangeables(currentPosition, currentVelocity, currentAcceleration);

      subTrajectory = new YoPolynomial3D(namePrefix, 4, registry);

      for (int i = 0; i < maximumNumberOfWaypoints; i++)
      {
         YoFrameEuclideanTrajectoryPoint waypoint = new YoFrameEuclideanTrajectoryPoint(namePrefix, "AtWaypoint" + i, registry);
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

   public void appendWaypoint(double timeAtWaypoint, Point3DReadOnly position, Vector3DReadOnly linearVelocity)
   {
      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + 1);
      appendWaypointUnsafe(timeAtWaypoint, position, linearVelocity);
   }

   private void appendWaypointUnsafe(double timeAtWaypoint, Point3DReadOnly position, Vector3DReadOnly linearVelocity)
   {
      waypoints.get(numberOfWaypoints.getIntegerValue()).set(timeAtWaypoint, position, linearVelocity);
      numberOfWaypoints.increment();
   }

   public void appendWaypoint(double timeAtWaypoint, FramePoint3DReadOnly position, FrameVector3DReadOnly linearVelocity)
   {
      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + 1);
      appendWaypointUnsafe(timeAtWaypoint, position, linearVelocity);
   }

   private void appendWaypointUnsafe(double timeAtWaypoint, FramePoint3DReadOnly position, FrameVector3DReadOnly linearVelocity)
   {
      waypoints.get(numberOfWaypoints.getIntegerValue()).set(timeAtWaypoint, position, linearVelocity);
      numberOfWaypoints.increment();
   }

   public void appendWaypoint(EuclideanTrajectoryPointBasics euclideanWaypoint)
   {
      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + 1);
      appendWaypointUnsafe(euclideanWaypoint);
   }

   public void appendWaypoint(FrameEuclideanTrajectoryPoint frameEuclideanTrajectoryPoint)
   {
      checkReferenceFrameMatch(frameEuclideanTrajectoryPoint);
      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + 1);
      appendWaypointUnsafe(frameEuclideanTrajectoryPoint);
   }

   public void appendWaypoint(FrameSE3TrajectoryPoint frameSE3TrajectoryPoint)
   {
      checkReferenceFrameMatch(frameSE3TrajectoryPoint);
      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + 1);
      appendWaypointUnsafe(frameSE3TrajectoryPoint);
   }

   private void appendWaypointUnsafe(EuclideanTrajectoryPointBasics euclideanWaypoint)
   {
      waypoints.get(numberOfWaypoints.getIntegerValue()).set(euclideanWaypoint);
      numberOfWaypoints.increment();
   }

   public void appendWaypoints(double[] timeAtWaypoints, FramePoint3DReadOnly[] positions, FrameVector3DReadOnly[] linearVelocities)
   {
      if (timeAtWaypoints.length != positions.length || linearVelocities != null && positions.length != linearVelocities.length)
         throw new RuntimeException("Arguments are inconsistent.");

      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + timeAtWaypoints.length);

      for (int i = 0; i < timeAtWaypoints.length; i++)
         appendWaypointUnsafe(timeAtWaypoints[i], positions[i], linearVelocities[i]);
   }

   public void appendWaypoints(double[] timeAtWaypoints, Point3DReadOnly[] positions, Vector3DReadOnly[] linearVelocities)
   {
      if (timeAtWaypoints.length != positions.length || positions.length != linearVelocities.length)
         throw new RuntimeException("Arguments are inconsistent.");

      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + timeAtWaypoints.length);

      for (int i = 0; i < timeAtWaypoints.length; i++)
         appendWaypointUnsafe(timeAtWaypoints[i], positions[i], linearVelocities[i]);
   }

   public void appendWaypoints(EuclideanTrajectoryPointBasics[] euclideanWaypoint)
   {
      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + euclideanWaypoint.length);

      for (int i = 0; i < euclideanWaypoint.length; i++)
         appendWaypointUnsafe(euclideanWaypoint[i]);
   }

   public void appendWaypoints(RecyclingArrayList<? extends EuclideanTrajectoryPointBasics> euclideanWaypoint)
   {
      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + euclideanWaypoint.size());

      for (int i = 0; i < euclideanWaypoint.size(); i++)
         appendWaypointUnsafe(euclideanWaypoint.get(i));
   }

   public void appendWaypoints(TrajectoryPointListBasics<? extends EuclideanTrajectoryPointBasics> trajectoryPointList)
   {
      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + trajectoryPointList.getNumberOfTrajectoryPoints());

      for (int i = 0; i < trajectoryPointList.getNumberOfTrajectoryPoints(); i++)
         appendWaypointUnsafe(trajectoryPointList.getTrajectoryPoint(i));
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

      if (numberOfWaypoints.getIntegerValue() == 1)
      {
         subTrajectory.setConstant(waypoints.get(0).getPosition());
      }
      else
         initializeSubTrajectory(0);
   }

   private void initializeSubTrajectory(int waypointIndex)
   {
      int secondWaypointIndex = Math.min(waypointIndex + 1, numberOfWaypoints.getValue() - 1);
      YoFrameEuclideanTrajectoryPoint start = waypoints.get(waypointIndex);
      YoFrameEuclideanTrajectoryPoint end = waypoints.get(secondWaypointIndex);
      subTrajectory.setCubic(0.0, end.getTime() - start.getTime(), start.getPosition(), start.getLinearVelocity(), end.getPosition(), end.getLinearVelocity());
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

      int secondWaypointIndex = Math.min(currentWaypointIndex.getValue() + 1, numberOfWaypoints.getValue() - 1);
      YoFrameEuclideanTrajectoryPoint start = waypoints.get(currentWaypointIndex.getValue());
      YoFrameEuclideanTrajectoryPoint end = waypoints.get(secondWaypointIndex);

      if (time < start.getTime())
      {
         currentPosition.set(start.getPosition());
         currentVelocity.setToZero();
         currentAcceleration.setToZero();
         return;
      }
      if (time > end.getTime())
      {
         currentPosition.set(end.getPosition());
         currentVelocity.setToZero();
         currentAcceleration.setToZero();
         return;
      }

      if (Precision.equals(start.getTime(), end.getTime()))
      {
         currentPosition.set(start.getPosition());
         currentVelocity.set(start.getLinearVelocity());
         currentAcceleration.setToZero();
         return;
      }

      double subTrajectoryTime = MathTools.clamp(time - start.getTime(), 0.0, end.getTime() - start.getTime());
      subTrajectory.compute(subTrajectoryTime);

      currentPosition.set(subTrajectory.getPosition());
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
      return currentTrajectoryTime.getValue() >= waypoints.get(currentWaypointIndex.getValue() + 1).getTime();
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
   public void showVisualization()
   {
   }

   @Override
   public void hideVisualization()
   {
   }

   @Override
   public void getPosition(FramePoint3D positionToPack)
   {
      positionToPack.setIncludingFrame(currentPosition);
   }

   @Override
   public void getVelocity(FrameVector3D linearVelocityToPack)
   {
      linearVelocityToPack.setIncludingFrame(currentVelocity);
   }

   @Override
   public void getAcceleration(FrameVector3D linearAccelerationToPack)
   {
      linearAccelerationToPack.setIncludingFrame(currentAcceleration);
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
      if (numberOfWaypoints.getIntegerValue() == 0)
         return namePrefix + ": Has no waypoints.";
      else
         return namePrefix + ": number of waypoints = " + numberOfWaypoints.getIntegerValue() + ", current waypoint index = "
               + currentWaypointIndex.getIntegerValue() + "\nFirst waypoint: " + waypoints.get(0) + ", last waypoint: "
               + waypoints.get(numberOfWaypoints.getIntegerValue() - 1);
   }
}
