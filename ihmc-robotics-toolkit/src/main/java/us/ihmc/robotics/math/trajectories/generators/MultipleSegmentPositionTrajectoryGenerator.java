package us.ihmc.robotics.math.trajectories.generators;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.robotics.math.trajectories.interfaces.FixedFramePositionTrajectoryGenerator;
import us.ihmc.robotics.time.TimeIntervalProvider;
import us.ihmc.robotics.time.TimeIntervalReadOnly;
import us.ihmc.robotics.time.TimeIntervalTools;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import static us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsTrajectoryGenerator.defaultMaximumNumberOfWaypoints;

public class MultipleSegmentPositionTrajectoryGenerator<T extends FixedFramePositionTrajectoryGenerator & TimeIntervalProvider & Settable<T>> implements FixedFramePositionTrajectoryGenerator
{
   private final String namePrefix;

   private final int maximumNumberOfSegments;

   private final YoRegistry registry;

   private final YoDouble currentSegmentTime;

   protected final YoInteger numberOfSegments;
   private final YoInteger currentSegmentIndex;
   protected final RecyclingArrayList<T> segments;

   private final FixedFramePoint3DBasics currentPosition;
   private final FixedFrameVector3DBasics currentVelocity;
   private final FixedFrameVector3DBasics currentAcceleration;

   public MultipleSegmentPositionTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, Supplier<T> trajectorySupplier, YoRegistry parentRegistry)
   {
      this(namePrefix, defaultMaximumNumberOfWaypoints, referenceFrame, trajectorySupplier, parentRegistry);
   }

   public MultipleSegmentPositionTrajectoryGenerator(String namePrefix,
                                                     int maximumNumberOfWaypoints,
                                                     ReferenceFrame referenceFrame,
                                                     Supplier<T> trajectorySupplier,
                                                     YoRegistry parentRegistry)
   {
      this.namePrefix = namePrefix;
      this.maximumNumberOfSegments = maximumNumberOfWaypoints;

      registry = new YoRegistry(namePrefix + getClass().getSimpleName());

      numberOfSegments = new YoInteger(namePrefix + "NumberOfSegments", registry);
      numberOfSegments.set(0);

      segments = new RecyclingArrayList<>(trajectorySupplier);

      currentSegmentTime = new YoDouble(namePrefix + "CurrentTrajectoryTime", registry);
      currentSegmentIndex = new YoInteger(namePrefix + "CurrentSegmentIndex", registry);

      String currentPositionName = namePrefix + "CurrentPosition";
      String currentVelocityName = namePrefix + "CurrentVelocity";
      String currentAccelerationName = namePrefix + "CurrentAcceleration";

      currentPosition = new YoFramePoint3D(currentPositionName, referenceFrame, registry);
      currentVelocity = new YoFrameVector3D(currentVelocityName, referenceFrame, registry);
      currentAcceleration = new YoFrameVector3D(currentAccelerationName, referenceFrame, registry);

      clear();

      parentRegistry.addChild(registry);
   }

   public void clear()
   {
      numberOfSegments.set(0);
      currentSegmentIndex.set(0);
      segments.clear();
   }

   public void appendSegment(T segment)
   {
      checkReferenceFrameMatch(segment);
      checkNumberOfSegments(numberOfSegments.getIntegerValue() + 1);
      checkNextSegmentIsContinuous(segment);
      appendSegmentsUnsafe(segment);
   }

   private void appendSegmentsUnsafe(T segment)
   {
      segments.add().set(segment);
      numberOfSegments.increment();
   }

   public void appendSegments(T[] segments)
   {
      checkNumberOfSegments(numberOfSegments.getIntegerValue() + segments.length);

      for (int i = 0; i < segments.length; i++)
         appendSegment(segments[i]);
   }

   public void appendSegments(List<T> segments)
   {
      checkNumberOfSegments(numberOfSegments.getIntegerValue() + segments.size());

      for (int i = 0; i < segments.size(); i++)
         appendSegment(segments.get(i));
   }

   protected void checkNumberOfSegments(int length)
   {
      if (length > maximumNumberOfSegments)
         throw new RuntimeException("Cannot exceed the maximum number of segments. Number of segments provided: " + length);
   }

   protected void checkNextSegmentIsContinuous(T segment)
   {
      if (getCurrentNumberOfSegments() == 0)
         return;

      if (!TimeIntervalTools.areTimeIntervalsConsecutive(segments.get(getCurrentNumberOfSegments() - 1), segment, 5e-3))
         throw new RuntimeException("The next segment doesn't start where the previous one ended.");
   }

   @Override
   public void initialize()
   {
      if (numberOfSegments.getIntegerValue() == 0)
      {
         throw new RuntimeException("Trajectory has no segments.");
      }

      if (!TimeIntervalTools.isTimeSequenceContinuous(segments))
         throw new RuntimeException("The segments do not represent a continuous time trajectory.");

      currentSegmentIndex.set(0);
   }

   @Override
   public void compute(double time)
   {
      if (isEmpty())
      {
         throw new RuntimeException("Can not call compute on an empty trajectory.");
      }

      currentSegmentTime.set(time);

      if (time < segments.get(currentSegmentIndex.getIntegerValue()).getTimeInterval().getStartTime())
      {
         currentSegmentIndex.set(0);
      }

      while (currentSegmentIndex.getIntegerValue() < numberOfSegments.getIntegerValue() - 1
             && time > segments.get(currentSegmentIndex.getIntegerValue()).getTimeInterval().getEndTime())
      {
         currentSegmentIndex.increment();
      }

      T segment = segments.get(currentSegmentIndex.getValue());
      TimeIntervalReadOnly timeInterval = segment.getTimeInterval();

      double subTrajectoryTime = MathTools.clamp(time - timeInterval.getStartTime(), 0.0, timeInterval.getDuration());
      segment.compute(subTrajectoryTime);

      currentPosition.set(segment.getPosition());
      currentVelocity.set(segment.getVelocity());
      currentAcceleration.set(segment.getAcceleration());
   }

   @Override
   public boolean isDone()
   {
      if (isEmpty())
         return true;

      boolean isLastWaypoint = currentSegmentIndex.getIntegerValue() >= numberOfSegments.getIntegerValue() - 1;
      if (!isLastWaypoint)
         return false;
      return currentSegmentTime.getValue() >= getEndTime();
   }

   public boolean isEmpty()
   {
      return numberOfSegments.getIntegerValue() == 0;
   }

   public int getCurrentSegmentIndex()
   {
      return currentSegmentIndex.getIntegerValue();
   }

   public double getCurrentSegmentTrajectoryTime()
   {
      return currentSegmentTime.getDoubleValue();
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

   public int getCurrentNumberOfSegments()
   {
      return numberOfSegments.getIntegerValue();
   }

   public int getMaximumNumberOfSegments()
   {
      return maximumNumberOfSegments;
   }

   public double getEndTime()
   {
      return segments.get(getCurrentNumberOfSegments() - 1).getTimeInterval().getEndTime();
   }

   public List<T> getSegments()
   {
      return segments;
   }

   public T getSegment(int segmentIdx)
   {
      return segments.get(segmentIdx);
   }

   public void removeSegment(int segmentIdx)
   {
      segments.remove(segmentIdx);
      numberOfSegments.decrement();
   }

   @Override
   public String toString()
   {
      if (numberOfSegments.getIntegerValue() == 0)
         return namePrefix + ": Has no waypoints.";
      else
         return namePrefix + ": number of waypoints = " + numberOfSegments.getIntegerValue() + ", current waypoint index = "
                + currentSegmentIndex.getIntegerValue();
   }
}
