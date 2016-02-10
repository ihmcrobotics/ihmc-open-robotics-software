package us.ihmc.robotics.math.trajectories;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePointInMultipleFrames;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.frames.YoFrameVectorInMultipleFrames;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

/*
 * Note: this class can be used to interpolate N variables simultaneously.
 * You don't have a provider for the waypoint, therefore you should use method initialize(
 */
public class MultipleWaypointsPositionTrajectoryGenerator extends PositionTrajectoryGeneratorInMultipleFrames
{
   private final int maximumNumberOfWaypoints;

   private final YoVariableRegistry registry;

   private final DoubleYoVariable currentTrajectoryTime;

   private final IntegerYoVariable numberOfWaypoints;
   private final IntegerYoVariable currentWaypointIndex;
   private final ArrayList<DoubleYoVariable> timeAtWaypoints;
   private final ArrayList<YoFramePoint> positionAtWaypoints;
   private final ArrayList<YoFrameVector> linearVelocityAtWaypoints;

   private final VelocityConstrainedPositionTrajectoryGenerator subTrajectory;

   public MultipleWaypointsPositionTrajectoryGenerator(String namePrefix, int maximumNumberOfWaypoints, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry)
   {
      this(namePrefix, maximumNumberOfWaypoints, false, referenceFrame, parentRegistry);
   }

   public MultipleWaypointsPositionTrajectoryGenerator(String namePrefix, int maximumNumberOfWaypoints, boolean allowMultipleFrames, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry)
   {
      super(allowMultipleFrames, referenceFrame);

      this.maximumNumberOfWaypoints = maximumNumberOfWaypoints;

      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());

      numberOfWaypoints = new IntegerYoVariable(namePrefix + "NumberOfWaypoints", registry);
      numberOfWaypoints.set(0);

      timeAtWaypoints = new ArrayList<>(maximumNumberOfWaypoints);
      positionAtWaypoints = new ArrayList<>(maximumNumberOfWaypoints);
      linearVelocityAtWaypoints = new ArrayList<>(maximumNumberOfWaypoints);

      currentTrajectoryTime = new DoubleYoVariable(namePrefix + "CurrentTrajectoryTime", registry);
      currentWaypointIndex = new IntegerYoVariable(namePrefix + "CurrentWaypointIndex", registry);

      subTrajectory = new VelocityConstrainedPositionTrajectoryGenerator(namePrefix + "SubTajectory", allowMultipleFrames, referenceFrame, registry);
      registerTrajectoryGeneratorsInMultipleFrames(subTrajectory);

      String positionAtWayPointName = namePrefix + "PositionAtWaypoint";
      String linearVelocityAtWaypointName = namePrefix + "LinearVelocityAtWaypoint";

      for (int i = 0; i < maximumNumberOfWaypoints; i++)
      {
         DoubleYoVariable timeAtWaypoint = new DoubleYoVariable(namePrefix + "TimeAtWaypoint" + i, registry);
         timeAtWaypoints.add(timeAtWaypoint);

         if (allowMultipleFrames)
         {
            YoFramePointInMultipleFrames positionAtWaypoint = new YoFramePointInMultipleFrames(positionAtWayPointName + i, registry, referenceFrame);
            positionAtWaypoints.add(positionAtWaypoint);

            YoFrameVectorInMultipleFrames linearVelocityAtWaypoint = new YoFrameVectorInMultipleFrames(linearVelocityAtWaypointName + i, registry, referenceFrame);
            linearVelocityAtWaypoints.add(linearVelocityAtWaypoint);

            registerMultipleFramesHolders(positionAtWaypoint, linearVelocityAtWaypoint);
         }
         else
         {
            YoFramePoint positionAtWaypoint = new YoFramePoint(positionAtWayPointName + i, referenceFrame, registry);
            positionAtWaypoints.add(positionAtWaypoint);

            YoFrameVector linearVelocityAtWaypoint = new YoFrameVector(linearVelocityAtWaypointName + i, referenceFrame, registry);
            linearVelocityAtWaypoints.add(linearVelocityAtWaypoint);
         }
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
         timeAtWaypoints.get(i).set(Double.NaN);
         positionAtWaypoints.get(i).setToNaN();
         linearVelocityAtWaypoints.get(i).setToNaN();
      }
   }

   public void appendWaypoint(double timeAtWaypoint, Point3d position, Vector3d linearVelocity)
   {
      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + 1);

      timeAtWaypoints.get(numberOfWaypoints.getIntegerValue()).set(timeAtWaypoint);
      positionAtWaypoints.get(numberOfWaypoints.getIntegerValue()).set(position);
      if (linearVelocity != null)
         linearVelocityAtWaypoints.get(numberOfWaypoints.getIntegerValue()).set(linearVelocity);
      else
         linearVelocityAtWaypoints.get(numberOfWaypoints.getIntegerValue()).setToZero();

      numberOfWaypoints.increment();
   }

   public void appendWaypoint(double timeAtWaypoint, FramePoint position, FrameVector linearVelocity)
   {
      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + 1);

      timeAtWaypoints.get(numberOfWaypoints.getIntegerValue()).set(timeAtWaypoint);
      positionAtWaypoints.get(numberOfWaypoints.getIntegerValue()).set(position);
      linearVelocityAtWaypoints.get(numberOfWaypoints.getIntegerValue()).set(linearVelocity);

      numberOfWaypoints.increment();
   }

   public void appendWaypoints(double[] timeAtWaypoints, FramePoint[] positions, FrameVector[] linearVelocities)
   {
      if (timeAtWaypoints.length != positions.length || (linearVelocities != null && positions.length != linearVelocities.length))
         throw new RuntimeException("Arguments are inconsistent.");

      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + timeAtWaypoints.length);

      for (int i = 0; i < timeAtWaypoints.length; i++)
         appendWaypoint(timeAtWaypoints[i], positions[i], linearVelocities[i]);
   }

   public void appendWaypoints(double[] timeAtWaypoints, Point3d[] positions, Vector3d[] linearVelocities)
   {
      if (timeAtWaypoints.length != positions.length || positions.length != linearVelocities.length)
         throw new RuntimeException("Arguments are inconsistent.");

      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + timeAtWaypoints.length);

      for (int i = 0; i < timeAtWaypoints.length; i++)
      {
         appendWaypoint(timeAtWaypoints[i], positions[i], linearVelocities[i]);
      }
   }

   public void appendWaypoint(EuclideanWaypointInterface euclideanWaypoint)
   {
      appendWaypoint(euclideanWaypoint.getTime(), euclideanWaypoint.getPosition(), euclideanWaypoint.getLinearVelocity());
   }

   public void appendWaypoints(EuclideanWaypointInterface[] euclideanWaypoint)
   {
      for (int i = 0; i < euclideanWaypoint.length; i++)
      {
         appendWaypoint(euclideanWaypoint[i]);
      }
   }

   public void appendWaypoints(WaypointPositionTrajectoryData trajectoryData)
   {
      trajectoryData.checkReferenceFrameMatch(getCurrentTrajectoryFrame());
      appendWaypoints(trajectoryData.getTimeAtWaypoints(), trajectoryData.getPositions(), trajectoryData.getVelocities());
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
         subTrajectory.setFinalConditions(positionAtWaypoints.get(0), linearVelocityAtWaypoints.get(0));
         subTrajectory.setTrajectoryTime(0.0);
         subTrajectory.initialize();
      }
      else
         initializeSubTrajectory(0);
   }

   private void initializeSubTrajectory(int waypointIndex)
   {
      YoFramePoint initialPosition = positionAtWaypoints.get(waypointIndex);
      YoFrameVector initialLinearVelocity = linearVelocityAtWaypoints.get(waypointIndex);
      subTrajectory.setInitialConditions(initialPosition, initialLinearVelocity);

      YoFramePoint finalPosition = positionAtWaypoints.get(waypointIndex + 1);
      YoFrameVector finalLinearVelocity = linearVelocityAtWaypoints.get(waypointIndex + 1);
      subTrajectory.setFinalConditions(finalPosition, finalLinearVelocity);

      double subTrajectoryTime = timeAtWaypoints.get(waypointIndex + 1).getDoubleValue() - timeAtWaypoints.get(waypointIndex).getDoubleValue();
      subTrajectory.setTrajectoryTime(subTrajectoryTime);

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
}
