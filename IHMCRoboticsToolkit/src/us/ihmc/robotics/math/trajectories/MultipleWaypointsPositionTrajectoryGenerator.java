package us.ihmc.robotics.math.trajectories;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

/*
 * Note: this class can be used to interpolate N variables simultaneously.
 * You don't have a provider for the waypoint, therefore you should use method initialize(
 */
public class MultipleWaypointsPositionTrajectoryGenerator extends PositionTrajectoryGeneratorInMultipleFrames
{
   private final String namePrefix;

   private final int maximumNumberOfWaypoints;

   private final YoVariableRegistry registry;

   private final DoubleYoVariable currentTrajectoryTime;

   private final IntegerYoVariable numberOfWaypoints;
   private final IntegerYoVariable currentWaypointIndex;
   private final ArrayList<YoFrameEuclideanWaypoint> waypoints;

   private final VelocityConstrainedPositionTrajectoryGenerator subTrajectory;

   public MultipleWaypointsPositionTrajectoryGenerator(String namePrefix, int maximumNumberOfWaypoints, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry)
   {
      this(namePrefix, maximumNumberOfWaypoints, false, referenceFrame, parentRegistry);
   }

   public MultipleWaypointsPositionTrajectoryGenerator(String namePrefix, int maximumNumberOfWaypoints, boolean allowMultipleFrames, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry)
   {
      super(allowMultipleFrames, referenceFrame);

      this.namePrefix = namePrefix;
      this.maximumNumberOfWaypoints = maximumNumberOfWaypoints;

      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());

      numberOfWaypoints = new IntegerYoVariable(namePrefix + "NumberOfWaypoints", registry);
      numberOfWaypoints.set(0);

      waypoints = new ArrayList<>(maximumNumberOfWaypoints);

      currentTrajectoryTime = new DoubleYoVariable(namePrefix + "CurrentTrajectoryTime", registry);
      currentWaypointIndex = new IntegerYoVariable(namePrefix + "CurrentWaypointIndex", registry);

      subTrajectory = new VelocityConstrainedPositionTrajectoryGenerator(namePrefix + "SubTrajectory", allowMultipleFrames, referenceFrame, registry);
      registerTrajectoryGeneratorsInMultipleFrames(subTrajectory);

      for (int i = 0; i < maximumNumberOfWaypoints; i++)
      {
         YoFrameEuclideanWaypoint waypoint = new YoFrameEuclideanWaypoint(namePrefix, "AtWaypoint" + i, registry, referenceFrame);
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

   public void appendWaypoint(double timeAtWaypoint, Point3d position, Vector3d linearVelocity)
   {
      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + 1);
      appendWaypointUnsafe(timeAtWaypoint, position, linearVelocity);
   }

   private void appendWaypointUnsafe(double timeAtWaypoint, Point3d position, Vector3d linearVelocity)
   {
      waypoints.get(numberOfWaypoints.getIntegerValue()).set(timeAtWaypoint, position, linearVelocity);
      numberOfWaypoints.increment();
   }

   public void appendWaypoint(double timeAtWaypoint, FramePoint position, FrameVector linearVelocity)
   {
      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + 1);
      appendWaypointUnsafe(timeAtWaypoint, position, linearVelocity);
   }

   private void appendWaypointUnsafe(double timeAtWaypoint, FramePoint position, FrameVector linearVelocity)
   {
      waypoints.get(numberOfWaypoints.getIntegerValue()).set(timeAtWaypoint, position, linearVelocity);
      numberOfWaypoints.increment();
   }

   public void appendWaypoint(EuclideanWaypointInterface euclideanWaypoint)
   {
      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + 1);
      appendWaypointUnsafe(euclideanWaypoint);
   }
   
   private void appendWaypointUnsafe(EuclideanWaypointInterface euclideanWaypoint)
   {
      waypoints.get(numberOfWaypoints.getIntegerValue()).set(euclideanWaypoint);
      numberOfWaypoints.increment();
   }

   public void appendWaypoints(double[] timeAtWaypoints, FramePoint[] positions, FrameVector[] linearVelocities)
   {
      if (timeAtWaypoints.length != positions.length || (linearVelocities != null && positions.length != linearVelocities.length))
         throw new RuntimeException("Arguments are inconsistent.");

      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + timeAtWaypoints.length);

      for (int i = 0; i < timeAtWaypoints.length; i++)
         appendWaypointUnsafe(timeAtWaypoints[i], positions[i], linearVelocities[i]);
   }

   public void appendWaypoints(double[] timeAtWaypoints, Point3d[] positions, Vector3d[] linearVelocities)
   {
      if (timeAtWaypoints.length != positions.length || positions.length != linearVelocities.length)
         throw new RuntimeException("Arguments are inconsistent.");

      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + timeAtWaypoints.length);

      for (int i = 0; i < timeAtWaypoints.length; i++)
         appendWaypointUnsafe(timeAtWaypoints[i], positions[i], linearVelocities[i]);
   }

   public void appendWaypoints(EuclideanWaypointInterface[] euclideanWaypoint)
   {
      for (int i = 0; i < euclideanWaypoint.length; i++)
         appendWaypointUnsafe(euclideanWaypoint[i]);
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
         waypoints.get(i).subtractTimeOffset(timeAtFirstWaypoint);

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
   public void get(FramePoint positionToPack)
   {
      subTrajectory.get(positionToPack);
   }

   @Override
   public void packVelocity(FrameVector linearVelocityToPack)
   {
      subTrajectory.packVelocity(linearVelocityToPack);
   }

   @Override
   public void packAcceleration(FrameVector linearAccelerationToPack)
   {
      subTrajectory.packAcceleration(linearAccelerationToPack);
   }

   @Override
   public void packLinearData(FramePoint positionToPack, FrameVector linearVelocityToPack, FrameVector linearAccelerationToPack)
   {
      subTrajectory.packLinearData(positionToPack, linearVelocityToPack, linearAccelerationToPack);
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
