package us.ihmc.robotics.math.trajectories;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoMutableFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class LinearSpatialVectorTrajectoryGenerator
{
   private final String namePrefix;

   private final int maximumNumberOfWaypoints;

   private final YoRegistry registry;

   private final YoDouble currentTrajectoryTime;

   private final YoInteger numberOfWaypoints;
   private final YoInteger currentWaypointIndex;

   private final List<YoSpatialWaypoint> waypoints;
   private final YoSpatialWaypoint currentValue;

   public LinearSpatialVectorTrajectoryGenerator(String namePrefix, int maximumNumberOfWaypoints, ReferenceFrame referenceFrame,
                                                 YoRegistry parentRegistry)
   {
      this.namePrefix = namePrefix;
      this.maximumNumberOfWaypoints = maximumNumberOfWaypoints;

      registry = new YoRegistry(namePrefix + getClass().getSimpleName());
      parentRegistry.addChild(registry);

      currentTrajectoryTime = new YoDouble(namePrefix + "CurrentTrajectoryTime", registry);
      numberOfWaypoints = new YoInteger(namePrefix + "NumberOfWaypoints", registry);
      numberOfWaypoints.set(0);
      currentWaypointIndex = new YoInteger(namePrefix + "CurrentWaypointIndex", registry);

      currentValue = new YoSpatialWaypoint(namePrefix + "CurrentValue", registry, referenceFrame);

      waypoints = new ArrayList<>(maximumNumberOfWaypoints);

      for (int i = 0; i < maximumNumberOfWaypoints; i++)
      {
         YoSpatialWaypoint waypoint = new YoSpatialWaypoint(namePrefix + "Waypoint" + i, registry, referenceFrame);
         waypoints.add(waypoint);
      }
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
      numberOfWaypoints.set(0);
      currentWaypointIndex.set(0);

      for (int i = 0; i < maximumNumberOfWaypoints; i++)
      {
         waypoints.get(i).setToNaN(referenceFrame);
      }
      currentValue.setReferenceFrame(referenceFrame);
   }

   public void appendWaypoint(double timeAtWaypoint, Vector3DReadOnly angularPart, Vector3DReadOnly linearPart)
   {
      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + 1);
      appendWaypointUnsafe(timeAtWaypoint, angularPart, linearPart);
   }

   private void appendWaypointUnsafe(double timeAtWaypoint, Vector3DReadOnly angularPart, Vector3DReadOnly linearPart)
   {
      waypoints.get(numberOfWaypoints.getIntegerValue()).setIncludingFrame(getCurrentTrajectoryFrame(), timeAtWaypoint, angularPart, linearPart);
      numberOfWaypoints.increment();
   }

   public void appendWaypoint(double timeAtWaypoint, FrameVector3DReadOnly angularPart, FrameVector3DReadOnly linearPart)
   {
      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + 1);
      appendWaypointUnsafe(timeAtWaypoint, angularPart, linearPart);
   }

   private void appendWaypointUnsafe(double timeAtWaypoint, FrameVector3DReadOnly angularPart, FrameVector3DReadOnly linearPart)
   {
      angularPart.checkReferenceFrameMatch(getCurrentTrajectoryFrame());
      linearPart.checkReferenceFrameMatch(getCurrentTrajectoryFrame());
      waypoints.get(numberOfWaypoints.getIntegerValue()).setIncludingFrame(timeAtWaypoint, angularPart, linearPart);
      numberOfWaypoints.increment();
   }

   public void appendWaypoint(double timeAtWaypoint, SpatialVectorReadOnly waypoint)
   {
      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + 1);
      appendWaypointUnsafe(timeAtWaypoint, waypoint.getAngularPart(), waypoint.getLinearPart());
   }

   public void appendWaypoint(SpatialWaypointBasics waypoint)
   {
      checkNumberOfWaypoints(numberOfWaypoints.getIntegerValue() + 1);
      appendWaypointUnsafe(waypoint.getTime(), waypoint.getAngularPart(), waypoint.getLinearPart());
   }

   private void checkNumberOfWaypoints(int length)
   {
      if (length > maximumNumberOfWaypoints)
         throw new RuntimeException("Cannot exceed the maximum number of waypoints. Number of waypoints provided: " + length);
   }

   public void initialize()
   {
      if (numberOfWaypoints.getIntegerValue() == 0)
      {
         throw new RuntimeException("Trajectory has no waypoints.");
      }

      currentWaypointIndex.set(0);
   }

   public void changeFrame(ReferenceFrame desiredFrame)
   {
      for (int i = 0; i < numberOfWaypoints.getIntegerValue(); i++)
         waypoints.get(i).changeFrame(desiredFrame);
      currentValue.changeFrame(desiredFrame);
   }

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
      YoSpatialWaypoint start = waypoints.get(currentWaypointIndex.getValue());
      YoSpatialWaypoint end = waypoints.get(secondWaypointIndex);

      double alpha = (time - start.getTime()) / (end.getTime() - start.getTime());
      alpha = MathTools.clamp(alpha, 0.0, 1.0);
      currentValue.getAngularPart().interpolate(start.getAngularPart(), end.getAngularPart(), alpha);
      currentValue.getLinearPart().interpolate(start.getLinearPart(), end.getLinearPart(), alpha);
   }

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

   public SpatialVectorReadOnly getCurrentValue()
   {
      return currentValue;
   }

   public double getLastWaypointTime()
   {
      return waypoints.get(numberOfWaypoints.getIntegerValue() - 1).getTime();
   }

   public int getCurrentNumberOfWaypoints()
   {
      return numberOfWaypoints.getValue();
   }

   public ReferenceFrame getCurrentTrajectoryFrame()
   {
      return currentValue.getReferenceFrame();
   }

   public String getNamePrefix()
   {
      return namePrefix;
   }

   private static class YoSpatialWaypoint implements SpatialWaypointBasics
   {
      private final YoDouble time;
      private final FrameVector3DBasics angularPart, linearPart;

      public YoSpatialWaypoint(String namePrefix, YoRegistry registry, ReferenceFrame referenceFrame)
      {
         time = new YoDouble(namePrefix + "Time", registry);

         angularPart = new YoMutableFrameVector3D(namePrefix + "Angular", "", registry, referenceFrame);
         linearPart = new YoMutableFrameVector3D(namePrefix + "Linear", "", registry, referenceFrame);
      }

      @Override
      public void setTime(double time)
      {
         this.time.set(time);
      }

      @Override
      public double getTime()
      {
         return time.getValue();
      }

      @Override
      public FixedFrameVector3DBasics getAngularPart()
      {
         return angularPart;
      }

      @Override
      public FixedFrameVector3DBasics getLinearPart()
      {
         return linearPart;
      }

      @Override
      public ReferenceFrame getReferenceFrame()
      {
         return linearPart.getReferenceFrame();
      }

      @Override
      public void setReferenceFrame(ReferenceFrame expressedInFrame)
      {
         linearPart.setReferenceFrame(expressedInFrame);
         angularPart.setReferenceFrame(expressedInFrame);
      }
   }

   public static class SpatialWaypoint implements SpatialWaypointBasics
   {
      private double time;
      private final SpatialVector spatialVector = new SpatialVector();

      public SpatialWaypoint()
      {
         setToZero();
      }

      @Override
      public void setTime(double time)
      {
         this.time = time;
      }

      @Override
      public double getTime()
      {
         return time;
      }

      @Override
      public FixedFrameVector3DBasics getAngularPart()
      {
         return spatialVector.getAngularPart();
      }

      @Override
      public FixedFrameVector3DBasics getLinearPart()
      {
         return spatialVector.getLinearPart();
      }

      @Override
      public ReferenceFrame getReferenceFrame()
      {
         return spatialVector.getReferenceFrame();
      }

      @Override
      public void setReferenceFrame(ReferenceFrame expressedInFrame)
      {
         spatialVector.setReferenceFrame(expressedInFrame);
      }
   }

   public static interface SpatialWaypointBasics extends SpatialVectorBasics
   {
      void setTime(double time);

      double getTime();

      @Override
      default void setToZero()
      {
         setTime(0.0);
         SpatialVectorBasics.super.setToZero();
      }

      @Override
      default void setToNaN()
      {
         setTime(Double.NaN);
         SpatialVectorBasics.super.setToNaN();
      }

      @Override
      default boolean containsNaN()
      {
         return SpatialVectorBasics.super.containsNaN();
      }

      default void set(double time, Vector3DReadOnly angularPart, Vector3DReadOnly linearPart)
      {
         setTime(time);
         SpatialVectorBasics.super.set(angularPart, linearPart);
      }

      default void set(double time, FrameVector3DReadOnly angularPart, FrameVector3DReadOnly linearPart)
      {
         setTime(time);
         SpatialVectorBasics.super.set(angularPart, linearPart);
      }

      default void set(double time, SpatialVectorReadOnly spatialVector)
      {
         setTime(time);
         SpatialVectorBasics.super.set(spatialVector);
      }

      default void set(SpatialWaypointBasics other)
      {
         set(other.getTime(), other);
      }

      default void setIncludingFrame(ReferenceFrame expressedInFrame, double time, Vector3DReadOnly angularPart, Vector3DReadOnly linearPart)
      {
         setTime(time);
         SpatialVectorBasics.super.setIncludingFrame(expressedInFrame, angularPart, linearPart);
      }

      default void setIncludingFrame(double time, FrameVector3DReadOnly angularPart, FrameVector3DReadOnly linearPart)
      {
         setTime(time);
         SpatialVectorBasics.super.setIncludingFrame(angularPart, linearPart);
      }

      default void setIncludingFrame(double time, SpatialVectorReadOnly spatialVector)
      {
         setTime(time);
         SpatialVectorBasics.super.setIncludingFrame(spatialVector);
      }

      default void setIncludingFrame(SpatialWaypointBasics other)
      {
         setIncludingFrame(other.getTime(), other);
      }
   }
}
