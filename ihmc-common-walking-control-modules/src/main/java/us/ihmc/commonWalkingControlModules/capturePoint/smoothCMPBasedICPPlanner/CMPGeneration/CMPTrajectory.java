package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CMPGeneration;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.robotics.math.trajectories.FrameTrajectory3D;
import us.ihmc.robotics.math.trajectories.SegmentedFrameTrajectory3D;

public class CMPTrajectory extends SegmentedFrameTrajectory3D
{
   private double timeIntoStep;

   public CMPTrajectory(int maxNumberOfSegments, int maxNumberOfCoefficients)
   {
      super(maxNumberOfSegments, maxNumberOfCoefficients);
      reset();
   }

   public void reset()
   {
      super.reset();
      timeIntoStep = Double.NaN;
   }

   public void update(double timeInState)
   {
      super.update(timeInState);
      timeIntoStep = timeInState;
   }

   public void update(double timeInState, FramePoint3D desiredCMPToPack)
   {
      update(timeInState);
      currentSegment.getFramePosition(desiredCMPToPack);
   }

   public void update(double timeInState, FramePoint3D desiredCMPToPack, FrameVector3D desiredCMPVelocityToPack)
   {
      update(timeInState, desiredCMPToPack);
      currentSegment.getFrameVelocity(desiredCMPVelocityToPack);
   }

   public void update(double timeInState, FramePoint3D desiredCMPToPack, FrameVector3D desiredCMPVelocityToPack, FrameVector3D desiredCMPAccelerationToPack)
   {
      update(timeInState, desiredCMPToPack, desiredCMPVelocityToPack);
      currentSegment.getFrameAcceleration(desiredCMPAccelerationToPack);
   }

   public boolean isDone()
   {
      boolean currentIsLast = currentSegmentIndex  == getNumberOfSegments() - 1;
      boolean currentIsDone = !currentSegment.timeIntervalContains(timeIntoStep);

      return currentIsLast && currentIsDone;
   }
   
   public void getExitCMPLocation(FramePoint3D exitCMPLocationToPack)
   {
      FrameTrajectory3D segment = segments.getLast();
      segment.compute(segment.getFinalTime());
      exitCMPLocationToPack.setIncludingFrame(segment.getFramePosition());
   }

   public void getEntryCMPLocation(FramePoint3D entryCMPLocationToPack)
   {
      FrameTrajectory3D segment = segments.getFirst();
      segment.compute(segment.getInitialTime());
      entryCMPLocationToPack.setIncludingFrame(segment.getFramePosition());
   }
}
