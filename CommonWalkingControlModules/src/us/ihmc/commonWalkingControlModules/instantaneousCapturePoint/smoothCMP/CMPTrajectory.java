package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP;

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

   public FrameTrajectory3D getCurrentSegment()
   {
      return currentSegment;
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
      boolean currentIsLast = currentSegmentIndex  == numberOfSegments - 1;
      boolean currentIsDone = !currentSegment.timeIntervalContains(timeIntoStep);

      return currentIsLast && currentIsDone;
   }
   
   public void getExitCMPLocation(FramePoint3D exitCMPLocationToPack)
   {
      segments.get(numberOfSegments - 1).compute(segments.get(numberOfSegments -1).getFinalTime());
      exitCMPLocationToPack.setIncludingFrame(segments.get(numberOfSegments - 1).getFramePosition());
   }

   public void getEntryCMPLocation(FramePoint3D entryCMPLocationToPack)
   {
      segments.get(0).compute(segments.get(0).getInitialTime());
      entryCMPLocationToPack.setIncludingFrame(segments.get(0).getFramePosition());
   }
}
