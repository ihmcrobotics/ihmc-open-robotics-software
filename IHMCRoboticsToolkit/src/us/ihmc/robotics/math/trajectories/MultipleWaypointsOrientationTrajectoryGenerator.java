package us.ihmc.robotics.math.trajectories;

import java.util.ArrayList;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameQuaternionInMultipleFrames;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.frames.YoFrameVectorInMultipleFrames;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class MultipleWaypointsOrientationTrajectoryGenerator extends OrientationTrajectoryGeneratorInMultipleFrames
{
   private final int maximumNumberOfWaypoints;

   private final YoVariableRegistry registry;

   private final DoubleYoVariable currentTrajectoryTime;

   private final IntegerYoVariable numberOfWaypoints;
   private final IntegerYoVariable currentWaypointIndex;
   private final ArrayList<DoubleYoVariable> timeAtWaypoints;
   private final ArrayList<YoFrameQuaternion> orientationAtWaypoints;
   private final ArrayList<YoFrameVector> angularVelocityAtWaypoints;

   private final HermiteCurveBasedOrientationTrajectoryGenerator subTrajectory;

   public MultipleWaypointsOrientationTrajectoryGenerator(String namePrefix, int maximumNumberOfWaypoints, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry)
   {
      this(namePrefix, maximumNumberOfWaypoints, false, referenceFrame, parentRegistry);
   }

   public MultipleWaypointsOrientationTrajectoryGenerator(String namePrefix, int maximumNumberOfWaypoints, boolean allowMultipleFrames, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry)
   {
      super(allowMultipleFrames, referenceFrame);

      this.maximumNumberOfWaypoints = maximumNumberOfWaypoints;

      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());

      numberOfWaypoints = new IntegerYoVariable(namePrefix + "NumberOfWaypoints", registry);
      numberOfWaypoints.set(0);

      timeAtWaypoints = new ArrayList<>(maximumNumberOfWaypoints);
      orientationAtWaypoints = new ArrayList<>(maximumNumberOfWaypoints);
      angularVelocityAtWaypoints = new ArrayList<>(maximumNumberOfWaypoints);

      currentTrajectoryTime = new DoubleYoVariable(namePrefix + "CurrentTrajectoryTime", registry);
      currentWaypointIndex = new IntegerYoVariable(namePrefix + "CurrentWaypointIndex", registry);

      subTrajectory = new HermiteCurveBasedOrientationTrajectoryGenerator(namePrefix + "SubTajectory", allowMultipleFrames, referenceFrame, registry);
      registerTrajectoryGeneratorsInMultipleFrames(subTrajectory);

      String orientationAtWayPointName = namePrefix + "OrientationAtWaypoint";
      String angularVelocityAtWaypointName = namePrefix + "AngularVelocityAtWaypoint";

      for (int i = 0; i < maximumNumberOfWaypoints; i++)
      {
         DoubleYoVariable timeAtWaypoint = new DoubleYoVariable(namePrefix + "TimeAtWaypoint" + i, registry);
         timeAtWaypoints.add(timeAtWaypoint);

         if (allowMultipleFrames)
         {
            YoFrameQuaternionInMultipleFrames orientationAtWaypoint = new YoFrameQuaternionInMultipleFrames(orientationAtWayPointName + i, registry, referenceFrame);
            orientationAtWaypoints.add(orientationAtWaypoint);

            YoFrameVectorInMultipleFrames angularVelocityAtWaypoint = new YoFrameVectorInMultipleFrames(angularVelocityAtWaypointName + i, registry, referenceFrame);
            angularVelocityAtWaypoints.add(angularVelocityAtWaypoint);

            registerMultipleFramesHolders(orientationAtWaypoint, angularVelocityAtWaypoint);
         }
         else
         {
            YoFrameQuaternion orientationAtWaypoint = new YoFrameQuaternion(orientationAtWayPointName + i, referenceFrame, registry);
            orientationAtWaypoints.add(orientationAtWaypoint);

            YoFrameVector angularVelocityAtWaypoint = new YoFrameVector(angularVelocityAtWaypointName + i, referenceFrame, registry);
            angularVelocityAtWaypoints.add(angularVelocityAtWaypoint);
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
         orientationAtWaypoints.get(i).setToNaN();
         angularVelocityAtWaypoints.get(i).setToNaN();
      }
   }

   public void appendWaypoint(double timeAtWaypoint, Quat4d orientation, Vector3d angularVelocity)
   {
      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + 1);

      timeAtWaypoints.get(numberOfWaypoints.getIntegerValue()).set(timeAtWaypoint);
      orientationAtWaypoints.get(numberOfWaypoints.getIntegerValue()).set(orientation);
      if (angularVelocity != null)
         angularVelocityAtWaypoints.get(numberOfWaypoints.getIntegerValue()).set(angularVelocity);
      else
         angularVelocityAtWaypoints.get(numberOfWaypoints.getIntegerValue()).setToZero();

      numberOfWaypoints.increment();
   }

   public void appendWaypoint(double timeAtWaypoint, FrameOrientation orientation, FrameVector angularVelocity)
   {
      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + 1);

      timeAtWaypoints.get(numberOfWaypoints.getIntegerValue()).set(timeAtWaypoint);
      orientationAtWaypoints.get(numberOfWaypoints.getIntegerValue()).set(orientation);
      angularVelocityAtWaypoints.get(numberOfWaypoints.getIntegerValue()).set(angularVelocity);

      numberOfWaypoints.increment();
   }

   public void appendWaypoints(double[] timeAtWaypoints, Quat4d[] orientations, Vector3d[] angularVelocities)
   {
      if (timeAtWaypoints.length != orientations.length || (angularVelocities != null && orientations.length != angularVelocities.length))
         throw new RuntimeException("Arguments are inconsistent.");

      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + timeAtWaypoints.length);

      for (int i = 0; i < timeAtWaypoints.length; i++)
         appendWaypoint(timeAtWaypoints[i], orientations[i], angularVelocities[i]);
   }

   public void appendWaypoints(double[] timeAtWaypoints, FrameOrientation[] orientations, FrameVector[] angularVelocities)
   {
      if (timeAtWaypoints.length != orientations.length || orientations.length != angularVelocities.length)
         throw new RuntimeException("Arguments are inconsistent.");

      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + timeAtWaypoints.length);

      for (int i = 0; i < timeAtWaypoints.length; i++)
      {
         appendWaypoint(timeAtWaypoints[i], orientations[i], angularVelocities[i]);
      }
   }

   public void appendWaypoint(SO3WaypointInterface so3Waypoint)
   {
      appendWaypoint(so3Waypoint.getTime(), so3Waypoint.getOrientation(), so3Waypoint.getAngularVelocity());
   }

   public void appendWaypoints(SO3WaypointInterface[] so3Waypoints)
   {
      for (int i = 0; i < so3Waypoints.length; i++)
      {
         appendWaypoint(so3Waypoints[i]);
      }
   }

   public void appendWaypoints(WaypointOrientationTrajectoryData trajectoryData)
   {
      trajectoryData.checkReferenceFrameMatch(getCurrentTrajectoryFrame());
      appendWaypoints(trajectoryData.getTimeAtWaypoints(), trajectoryData.getOrientations(), trajectoryData.getAngularVelocities());
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
         subTrajectory.setFinalConditions(orientationAtWaypoints.get(0), angularVelocityAtWaypoints.get(0));
         subTrajectory.setTrajectoryTime(0.0);
         subTrajectory.initialize();
      }
      else
         initializeSubTrajectory(0);
   }

   private void initializeSubTrajectory(int waypointIndex)
   {
      YoFrameQuaternion initialOrientation = orientationAtWaypoints.get(waypointIndex);
      YoFrameVector initialAngularVelocity = angularVelocityAtWaypoints.get(waypointIndex);
      subTrajectory.setInitialConditions(initialOrientation, initialAngularVelocity);

      YoFrameQuaternion finalOrientation = orientationAtWaypoints.get(waypointIndex + 1);
      YoFrameVector finalAngularVelocity = angularVelocityAtWaypoints.get(waypointIndex + 1);
      subTrajectory.setFinalConditions(finalOrientation, finalAngularVelocity);

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
   public void get(FrameOrientation orientationToPack)
   {
      subTrajectory.get(orientationToPack);
   }

   @Override
   public void packAngularVelocity(FrameVector angularVelocityToPack)
   {
      subTrajectory.packAngularVelocity(angularVelocityToPack);
   }

   @Override
   public void packAngularAcceleration(FrameVector angularAccelerationToPack)
   {
      subTrajectory.packAngularAcceleration(angularAccelerationToPack);
   }

   @Override
   public void packAngularData(FrameOrientation orientationToPack, FrameVector angularVelocityToPack, FrameVector angularAccelerationToPack)
   {
      subTrajectory.packAngularData(orientationToPack, angularVelocityToPack, angularAccelerationToPack);
   }
}
