package us.ihmc.robotics.math.trajectories.waypoints;

import static us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsTrajectoryGenerator.*;

import java.util.ArrayList;

import org.apache.commons.math3.util.Precision;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.math.frames.YoFramePointInMultipleFrames;
import us.ihmc.robotics.math.frames.YoFrameVectorInMultipleFrames;
import us.ihmc.robotics.math.trajectories.PositionTrajectoryGeneratorInMultipleFrames;
import us.ihmc.robotics.math.trajectories.YoPolynomial3D;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.EuclideanTrajectoryPointInterface;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.TrajectoryPointListInterface;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.yoVariables.variable.YoInteger;

public class MultipleWaypointsPositionTrajectoryGenerator extends PositionTrajectoryGeneratorInMultipleFrames
{
   private final String namePrefix;

   private final int maximumNumberOfWaypoints;

   private final YoVariableRegistry registry;

   private final YoDouble currentTrajectoryTime;

   private final YoInteger numberOfWaypoints;
   private final YoInteger currentWaypointIndex;
   private final ArrayList<YoFrameEuclideanTrajectoryPoint> waypoints;

   private final YoFramePoint3D currentPosition;
   private final YoFrameVector3D currentVelocity;
   private final YoFrameVector3D currentAcceleration;
   private final YoPolynomial3D subTrajectory;

   public MultipleWaypointsPositionTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry)
   {
      this(namePrefix, defaultMaximumNumberOfWaypoints, referenceFrame, parentRegistry);
   }

   public MultipleWaypointsPositionTrajectoryGenerator(String namePrefix, boolean allowMultipleFrames, ReferenceFrame referenceFrame,
                                                       YoVariableRegistry parentRegistry)
   {
      this(namePrefix, defaultMaximumNumberOfWaypoints, allowMultipleFrames, referenceFrame, parentRegistry);
   }

   public MultipleWaypointsPositionTrajectoryGenerator(String namePrefix, int maximumNumberOfWaypoints, ReferenceFrame referenceFrame,
                                                       YoVariableRegistry parentRegistry)
   {
      this(namePrefix, maximumNumberOfWaypoints, false, referenceFrame, parentRegistry);
   }

   public MultipleWaypointsPositionTrajectoryGenerator(String namePrefix, int maximumNumberOfWaypoints, boolean allowMultipleFrames,
                                                       ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry)
   {
      super(allowMultipleFrames, referenceFrame);

      this.namePrefix = namePrefix;
      this.maximumNumberOfWaypoints = maximumNumberOfWaypoints;

      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());

      numberOfWaypoints = new YoInteger(namePrefix + "NumberOfWaypoints", registry);
      numberOfWaypoints.set(0);

      waypoints = new ArrayList<>(maximumNumberOfWaypoints);

      currentTrajectoryTime = new YoDouble(namePrefix + "CurrentTrajectoryTime", registry);
      currentWaypointIndex = new YoInteger(namePrefix + "CurrentWaypointIndex", registry);

      String currentPositionName = namePrefix + "CurrentPosition";
      String currentVelocityName = namePrefix + "CurrentVelocity";
      String currentAccelerationName = namePrefix + "CurrentAcceleration";

      if (allowMultipleFrames)
      {
         YoFramePointInMultipleFrames currentPosition = new YoFramePointInMultipleFrames(currentPositionName, registry, referenceFrame);
         YoFrameVectorInMultipleFrames currentVelocity = new YoFrameVectorInMultipleFrames(currentVelocityName, registry, referenceFrame);
         YoFrameVectorInMultipleFrames currentAcceleration = new YoFrameVectorInMultipleFrames(currentAccelerationName, registry, referenceFrame);

         registerMultipleFramesHolders(currentPosition, currentVelocity, currentAcceleration);
         this.currentPosition = currentPosition;
         this.currentVelocity = currentVelocity;
         this.currentAcceleration = currentAcceleration;
      }
      else
      {
         currentPosition = new YoFramePoint3D(currentPositionName, referenceFrame, registry);
         currentVelocity = new YoFrameVector3D(currentVelocityName, referenceFrame, registry);
         currentAcceleration = new YoFrameVector3D(currentAccelerationName, referenceFrame, registry);
      }

      subTrajectory = new YoPolynomial3D(namePrefix, 4, registry);

      for (int i = 0; i < maximumNumberOfWaypoints; i++)
      {
         YoFrameEuclideanTrajectoryPoint waypoint = new YoFrameEuclideanTrajectoryPoint(namePrefix, "AtWaypoint" + i, registry, referenceFrame);
         waypoints.add(waypoint);
         if (allowMultipleFrames)
            registerMultipleFramesHolders(waypoint);
      }

      clear();

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
      switchTrajectoryFrame(referenceFrame);
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

   public void appendWaypoint(EuclideanTrajectoryPointInterface<?> euclideanWaypoint)
   {
      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + 1);
      appendWaypointUnsafe(euclideanWaypoint);
   }

   public void appendWaypoint(FrameEuclideanTrajectoryPoint frameEuclideanTrajectoryPoint)
   {
      frameEuclideanTrajectoryPoint.checkReferenceFrameMatch(getCurrentTrajectoryFrame());
      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + 1);
      appendWaypointUnsafe(frameEuclideanTrajectoryPoint);
   }

   public void appendWaypoint(FrameSE3TrajectoryPoint frameSE3TrajectoryPoint)
   {
      frameSE3TrajectoryPoint.checkReferenceFrameMatch(getCurrentTrajectoryFrame());
      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + 1);
      appendWaypointUnsafe(frameSE3TrajectoryPoint);
   }

   private void appendWaypointUnsafe(EuclideanTrajectoryPointInterface<?> euclideanWaypoint)
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

   public void appendWaypoints(EuclideanTrajectoryPointInterface<?>[] euclideanWaypoint)
   {
      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + euclideanWaypoint.length);

      for (int i = 0; i < euclideanWaypoint.length; i++)
         appendWaypointUnsafe(euclideanWaypoint[i]);
   }

   public void appendWaypoints(RecyclingArrayList<? extends EuclideanTrajectoryPointInterface<?>> euclideanWaypoint)
   {
      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + euclideanWaypoint.size());

      for (int i = 0; i < euclideanWaypoint.size(); i++)
         appendWaypointUnsafe(euclideanWaypoint.get(i));
   }

   public void appendWaypoints(TrajectoryPointListInterface<?, ? extends EuclideanTrajectoryPointInterface<?>> trajectoryPointList)
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
      YoFrameEuclideanTrajectoryPoint start = waypoints.get(waypointIndex);
      YoFrameEuclideanTrajectoryPoint end = waypoints.get(waypointIndex + 1);
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

      YoFrameEuclideanTrajectoryPoint start = waypoints.get(currentWaypointIndex.getValue());
      YoFrameEuclideanTrajectoryPoint end = waypoints.get(currentWaypointIndex.getValue() + 1);

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
