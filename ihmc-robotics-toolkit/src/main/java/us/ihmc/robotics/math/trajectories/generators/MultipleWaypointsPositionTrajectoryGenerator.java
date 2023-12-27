package us.ihmc.robotics.math.trajectories.generators;

import static us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsTrajectoryGenerator.defaultMaximumNumberOfWaypoints;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math3.util.Precision;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameFactories;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.math.trajectories.core.Polynomial3D;
import us.ihmc.robotics.math.trajectories.interfaces.FramePositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.YoEuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.EuclideanTrajectoryPointReadOnly;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.FixedFrameEuclideanTrajectoryPointBasics;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.FrameEuclideanTrajectoryPointReadOnly;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.FrameSE3TrajectoryPointReadOnly;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.TrajectoryPointListBasics;
import us.ihmc.yoVariables.euclid.YoPoint3D;
import us.ihmc.yoVariables.euclid.YoVector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.interfaces.FrameIndexMap;
import us.ihmc.yoVariables.euclid.referenceFrame.interfaces.YoMutableFrameObject;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.tools.YoGeometryNameTools;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoLong;

/**
 * A tool to generate a point trajectory as a cubic 3D spline through a series
 * of provided waypoints. Given a query time, it provides the corresponding position,
 * spatial velocity, and spatial acceleration.
 */
public class MultipleWaypointsPositionTrajectoryGenerator implements FramePositionTrajectoryGenerator, YoMutableFrameObject
{
   private final String namePrefix;

   private final int maximumNumberOfWaypoints;

   private final YoRegistry registry;

   private final YoLong frameId;
   private final FrameIndexMap frameIndexMap;

   private final YoDouble currentTrajectoryTime;

   private final YoInteger numberOfWaypoints;
   private final YoInteger currentWaypointIndex;
   private final List<FixedFrameEuclideanTrajectoryPointBasics> waypoints;

   private final FixedFramePoint3DBasics currentPosition;
   private final FixedFrameVector3DBasics currentVelocity;
   private final FixedFrameVector3DBasics currentAcceleration;
   private final Polynomial3D subTrajectory;

   public MultipleWaypointsPositionTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, YoRegistry parentRegistry)
   {
      this(namePrefix, defaultMaximumNumberOfWaypoints, referenceFrame, parentRegistry);
   }

   public MultipleWaypointsPositionTrajectoryGenerator(String namePrefix,
                                                       int maximumNumberOfWaypoints,
                                                       ReferenceFrame referenceFrame,
                                                       YoRegistry parentRegistry)
   {
      this.namePrefix = namePrefix;
      this.maximumNumberOfWaypoints = maximumNumberOfWaypoints;

      registry = new YoRegistry(namePrefix + getClass().getSimpleName());

      frameId = new YoLong(YoGeometryNameTools.assembleName(namePrefix, "frame"), registry);
      frameIndexMap = new FrameIndexMap.FrameIndexHashMap();

      numberOfWaypoints = new YoInteger(namePrefix + "NumberOfWaypoints", registry);
      numberOfWaypoints.set(0);

      waypoints = new ArrayList<>(maximumNumberOfWaypoints);

      currentTrajectoryTime = new YoDouble(namePrefix + "CurrentTrajectoryTime", registry);
      currentWaypointIndex = new YoInteger(namePrefix + "CurrentWaypointIndex", registry);

      String currentPositionName = namePrefix + "CurrentPosition";
      String currentVelocityName = namePrefix + "CurrentVelocity";
      String currentAccelerationName = namePrefix + "CurrentAcceleration";

      currentPosition = EuclidFrameFactories.newLinkedFixedFramePoint3DBasics(this, new YoPoint3D(currentPositionName, registry));
      currentVelocity = EuclidFrameFactories.newLinkedFixedFrameVector3DBasics(this, new YoVector3D(currentVelocityName, registry));
      currentAcceleration = EuclidFrameFactories.newLinkedFixedFrameVector3DBasics(this, new YoVector3D(currentAccelerationName, registry));

      subTrajectory = new Polynomial3D(4);

      for (int i = 0; i < maximumNumberOfWaypoints; i++)
      {
         YoEuclideanTrajectoryPoint yoTrajectoryPoint = new YoEuclideanTrajectoryPoint(namePrefix, "AtWaypoint" + i, registry);
         waypoints.add(FixedFrameEuclideanTrajectoryPointBasics.newLinkedFixedFrameEuclideanTrajectoryPointBasics(this, yoTrajectoryPoint));
      }

      clear(referenceFrame);

      parentRegistry.addChild(registry);
   }

   public void clear()
   {
      for (int i = 0; i < numberOfWaypoints.getIntegerValue(); i++)
         waypoints.get(i).setToNaN();

      numberOfWaypoints.set(0);
      currentWaypointIndex.set(0);
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

   public void appendWaypoint(EuclideanTrajectoryPointReadOnly euclideanWaypoint)
   {
      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + 1);
      appendWaypointUnsafe(euclideanWaypoint);
   }

   public void appendWaypoint(FrameEuclideanTrajectoryPointReadOnly frameEuclideanTrajectoryPoint)
   {
      checkReferenceFrameMatch(frameEuclideanTrajectoryPoint);
      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + 1);
      appendWaypointUnsafe(frameEuclideanTrajectoryPoint);
   }

   public void appendWaypoint(FrameSE3TrajectoryPointReadOnly frameSE3TrajectoryPoint)
   {
      checkReferenceFrameMatch(frameSE3TrajectoryPoint);
      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + 1);
      appendWaypointUnsafe(frameSE3TrajectoryPoint);
   }

   private void appendWaypointUnsafe(EuclideanTrajectoryPointReadOnly euclideanWaypoint)
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

   public void appendWaypoints(EuclideanTrajectoryPointReadOnly[] euclideanWaypoint)
   {
      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + euclideanWaypoint.length);

      for (int i = 0; i < euclideanWaypoint.length; i++)
         appendWaypointUnsafe(euclideanWaypoint[i]);
   }

   public void appendWaypoints(RecyclingArrayList<? extends EuclideanTrajectoryPointReadOnly> euclideanWaypoint)
   {
      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + euclideanWaypoint.size());

      for (int i = 0; i < euclideanWaypoint.size(); i++)
         appendWaypointUnsafe(euclideanWaypoint.get(i));
   }

   public void appendWaypoints(TrajectoryPointListBasics<? extends EuclideanTrajectoryPointReadOnly> trajectoryPointList)
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
   }

   @Override
   public void compute(double time)
   {
      if (isEmpty())
      {
         throw new RuntimeException("Can not call compute on an empty trajectory.");
      }

      currentTrajectoryTime.set(time);

      if (time < waypoints.get(currentWaypointIndex.getIntegerValue()).getTime())
      {
         currentWaypointIndex.set(0);
      }

      while (currentWaypointIndex.getIntegerValue() < numberOfWaypoints.getIntegerValue() - 2
            && time >= waypoints.get(currentWaypointIndex.getIntegerValue() + 1).getTime())
      {
         currentWaypointIndex.increment();
      }

      int secondWaypointIndex = Math.min(currentWaypointIndex.getValue() + 1, numberOfWaypoints.getValue() - 1);
      FixedFrameEuclideanTrajectoryPointBasics start = waypoints.get(currentWaypointIndex.getValue());
      FixedFrameEuclideanTrajectoryPointBasics end = waypoints.get(secondWaypointIndex);

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

      // Initialize the segment trajectory, in case the index or waypoints have changed
      subTrajectory.setCubicDirectly(end.getTime()
            - start.getTime(), start.getPosition(), start.getLinearVelocity(), end.getPosition(), end.getLinearVelocity());
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
   public FramePoint3DReadOnly getPosition()
   {
      return currentPosition;
   }

   @Override
   public FrameVector3DReadOnly getVelocity()
   {
      return currentVelocity;
   }

   @Override
   public FrameVector3DReadOnly getAcceleration()
   {
      return currentAcceleration;
   }

   public int getCurrentNumberOfWaypoints()
   {
      return numberOfWaypoints.getIntegerValue();
   }

   public FixedFrameEuclideanTrajectoryPointBasics getWaypoint(int idx)
   {
      return waypoints.get(idx);
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
   public ReferenceFrame getReferenceFrame()
   {
      return YoMutableFrameObject.super.getReferenceFrame();
   }

   @Override
   public YoLong getYoFrameIndex()
   {
      return frameId;
   }

   @Override
   public FrameIndexMap getFrameIndexMap()
   {
      return frameIndexMap;
   }

   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      YoMutableFrameObject.super.setReferenceFrame(referenceFrame);
   }

   @Override
   public void applyTransform(Transform transform)
   {
      currentPosition.applyTransform(transform);
      currentVelocity.applyTransform(transform);
      currentAcceleration.applyTransform(transform);

      for (int i = 0; i < numberOfWaypoints.getValue(); i++)
      {
         waypoints.get(i).applyTransform(transform);
      }
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      currentPosition.applyInverseTransform(transform);
      currentVelocity.applyInverseTransform(transform);
      currentAcceleration.applyInverseTransform(transform);

      for (int i = 0; i < numberOfWaypoints.getValue(); i++)
      {
         waypoints.get(i).applyInverseTransform(transform);
      }
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
