package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPlane;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.mecano.spatial.interfaces.FixedFrameWrenchBasics;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameWrench;
import us.ihmc.robotics.time.TimeIntervalReadOnly;
import us.ihmc.robotics.time.TimeIntervalTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.List;

import static us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsTrajectoryGenerator.defaultMaximumNumberOfWaypoints;

public class MultipleWrenchSegmentTrajectoryGenerator implements ReferenceFrameHolder
{
   private final String namePrefix;

   private final int maximumNumberOfSegments;

   private final YoRegistry registry;

   private final YoDouble currentSegmentTime;

   protected final YoInteger numberOfSegments;
   private final YoInteger currentSegmentIndex;
   protected final RecyclingArrayList<WrenchTrajectorySegment> segments;

   private final FixedFrameWrenchBasics currentWrench;

   public MultipleWrenchSegmentTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, YoRegistry parentRegistry)
   {
      this(namePrefix, defaultMaximumNumberOfWaypoints, referenceFrame, parentRegistry);
   }

   public MultipleWrenchSegmentTrajectoryGenerator(String namePrefix,
                                                   int maximumNumberOfSegments,
                                                   ReferenceFrame referenceFrame,
                                                   YoRegistry parentRegistry)
   {
      this.namePrefix = namePrefix;
      this.maximumNumberOfSegments = maximumNumberOfSegments;

      registry = new YoRegistry(namePrefix + getClass().getSimpleName());

      numberOfSegments = new YoInteger(namePrefix + "NumberOfSegments", registry);
      numberOfSegments.set(0);

      segments = new RecyclingArrayList<>(WrenchTrajectorySegment::new);

      currentSegmentTime = new YoDouble(namePrefix + "CurrentTrajectoryTime", registry);
      currentSegmentIndex = new YoInteger(namePrefix + "CurrentSegmentIndex", registry);

      String currentPositionName = namePrefix + "CurrentWrench";

      currentWrench = new YoFixedFrameWrench(currentPositionName, referenceFrame, referenceFrame, registry);

      clear();

      parentRegistry.addChild(registry);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return getWrench().getReferenceFrame();
   }

   public void clear()
   {
      numberOfSegments.set(0);
      currentSegmentIndex.set(0);
      segments.clear();
   }

   public void appendSegment(WrenchTrajectorySegment segment)
   {
      checkReferenceFrameMatch(segment);
      checkNumberOfSegments(numberOfSegments.getIntegerValue() + 1);
      checkNextSegmentIsContinuous(segment);
      appendSegmentsUnsafe(segment);
   }

   private void appendSegmentsUnsafe(WrenchTrajectorySegment segment)
   {
      segments.add().set(segment);
      numberOfSegments.increment();
   }

   public void appendSegment(TimeIntervalReadOnly timeInterval, double mass, double omega, List<MPCContactPlane> contactPlanes)
   {
      checkNumberOfSegments(numberOfSegments.getIntegerValue() + 1);
      WrenchTrajectorySegment segment = segments.add();
      checkReferenceFrameMatch(segment);

      segment.getTimeInterval().set(timeInterval);
      checkNextSegmentIsContinuous(segment);

      segment.setOmega(omega);
      segment.setMass(mass);
      segment.setCoefficients(contactPlanes);

      numberOfSegments.increment();
   }

   public void appendSegments(WrenchTrajectorySegment[] segments)
   {
      checkNumberOfSegments(numberOfSegments.getIntegerValue() + segments.length);

      for (int i = 0; i < segments.length; i++)
         appendSegment(segments[i]);
   }

   public void appendSegments(List<WrenchTrajectorySegment> segments)
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

   protected void checkNextSegmentIsContinuous(WrenchTrajectorySegment segment)
   {
      if (getCurrentNumberOfSegments() == 0)
         return;

      if (!TimeIntervalTools.areTimeIntervalsConsecutive(segments.get(getCurrentNumberOfSegments() - 1), segment, 5e-3))
         throw new RuntimeException("The next segment doesn't start where the previous one ended.");
   }

   public void initialize()
   {
      if (numberOfSegments.getIntegerValue() == 0)
      {
         throw new RuntimeException("Trajectory has no segments.");
      }

      currentSegmentIndex.set(0);
   }

   public void compute(double time)
   {
      if (isEmpty())
      {
         throw new RuntimeException("Can not call compute on an empty trajectory.");
      }

      currentSegmentTime.set(time);

      if (!TimeIntervalTools.isTimeSequenceContinuous(segments))
         throw new RuntimeException("The segments do not represent a continuous time trajectory.");

      if (time < segments.get(currentSegmentIndex.getIntegerValue()).getTimeInterval().getStartTime())
      {
         currentSegmentIndex.set(0);
      }

      while (currentSegmentIndex.getIntegerValue() < numberOfSegments.getIntegerValue() - 1
             && time > segments.get(currentSegmentIndex.getIntegerValue()).getTimeInterval().getEndTime())
      {
         currentSegmentIndex.increment();
      }

      WrenchTrajectorySegment segment = segments.get(currentSegmentIndex.getValue());
      TimeIntervalReadOnly timeInterval = segment.getTimeInterval();

      double subTrajectoryTime = MathTools.clamp(time - timeInterval.getStartTime(), 0.0, timeInterval.getDuration());
      segment.compute(subTrajectoryTime);

      currentWrench.set(segment.getWrench());
   }

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

   public WrenchReadOnly getWrench()
   {
      return currentWrench;
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

   public List<WrenchTrajectorySegment> getSegments()
   {
      return segments;
   }

   public WrenchTrajectorySegment getSegment(int segmentIdx)
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
         return namePrefix + ": Has no segments.";
      else
         return namePrefix + ": number of segments = " + numberOfSegments.getIntegerValue() + ", current segment index = "
                + currentSegmentIndex.getIntegerValue();
   }
}
