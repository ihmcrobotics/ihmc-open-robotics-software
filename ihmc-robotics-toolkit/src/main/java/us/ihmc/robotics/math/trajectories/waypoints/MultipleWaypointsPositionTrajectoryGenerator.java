package us.ihmc.robotics.math.trajectories.waypoints;

import static us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsTrajectoryGenerator.defaultMaximumNumberOfWaypoints;

import java.util.ArrayList;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.trajectories.PositionTrajectoryGeneratorInMultipleFrames;
import us.ihmc.robotics.math.trajectories.VelocityConstrainedPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.EuclideanTrajectoryPointInterface;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.TrajectoryPointListInterface;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
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

   private final VelocityConstrainedPositionTrajectoryGenerator subTrajectory;

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

      subTrajectory = new VelocityConstrainedPositionTrajectoryGenerator(namePrefix + "SubTrajectory", allowMultipleFrames, referenceFrame, registry);
      registerTrajectoryGeneratorsInMultipleFrames(subTrajectory);

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

   public void appendWaypoint(double timeAtWaypoint, Point3D position, Vector3D linearVelocity)
   {
      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + 1);
      appendWaypointUnsafe(timeAtWaypoint, position, linearVelocity);
   }

   private void appendWaypointUnsafe(double timeAtWaypoint, Point3D position, Vector3D linearVelocity)
   {
      waypoints.get(numberOfWaypoints.getIntegerValue()).set(timeAtWaypoint, position, linearVelocity);
      numberOfWaypoints.increment();
   }

   public void appendWaypoint(double timeAtWaypoint, FramePoint3D position, FrameVector3D linearVelocity)
   {
      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + 1);
      appendWaypointUnsafe(timeAtWaypoint, position, linearVelocity);
   }

   private void appendWaypointUnsafe(double timeAtWaypoint, FramePoint3D position, FrameVector3D linearVelocity)
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

   public void appendWaypoints(double[] timeAtWaypoints, FramePoint3D[] positions, FrameVector3D[] linearVelocities)
   {
      if (timeAtWaypoints.length != positions.length || linearVelocities != null && positions.length != linearVelocities.length)
         throw new RuntimeException("Arguments are inconsistent.");

      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + timeAtWaypoints.length);

      for (int i = 0; i < timeAtWaypoints.length; i++)
         appendWaypointUnsafe(timeAtWaypoints[i], positions[i], linearVelocities[i]);
   }

   public void appendWaypoints(double[] timeAtWaypoints, Point3D[] positions, Vector3D[] linearVelocities)
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
         subTrajectory.setTrajectoryParameters(waypoints.get(0), waypoints.get(0));
         subTrajectory.initialize();
      }
      else
         initializeSubTrajectory(0);
   }

   private void initializeSubTrajectory(int waypointIndex)
   {
      subTrajectory.setTrajectoryParameters(waypoints.get(waypointIndex), waypoints.get(waypointIndex + 1));
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
      subTrajectory.getPosition(positionToPack);
   }

   @Override
   public void getVelocity(FrameVector3D linearVelocityToPack)
   {
      subTrajectory.getVelocity(linearVelocityToPack);
   }

   @Override
   public void getAcceleration(FrameVector3D linearAccelerationToPack)
   {
      subTrajectory.getAcceleration(linearAccelerationToPack);
   }

   @Override
   public void getLinearData(FramePoint3D positionToPack, FrameVector3D linearVelocityToPack, FrameVector3D linearAccelerationToPack)
   {
      subTrajectory.getLinearData(positionToPack, linearVelocityToPack, linearAccelerationToPack);
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
