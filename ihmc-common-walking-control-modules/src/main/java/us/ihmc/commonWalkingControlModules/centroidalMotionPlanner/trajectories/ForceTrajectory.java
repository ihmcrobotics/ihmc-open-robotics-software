package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.trajectories;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.robotics.math.trajectories.FrameTrajectory3D;
import us.ihmc.robotics.math.trajectories.SegmentedFrameTrajectory3D;

public class ForceTrajectory extends SegmentedFrameTrajectory3D
{
   public ForceTrajectory(int maxNumberOfSegments, int maxNumberOfCoefficients)
   {
      super(maxNumberOfSegments, maxNumberOfCoefficients);
   }

   public void set(FrameTrajectory3D trajectory)
   {
      FrameTrajectory3D newSegment = segments.add();
      newSegment.set(trajectory);
   }

   public void update(double time, FrameVector3D forceValue)
   {
      update(time);
      forceValue.setIncludingFrame(currentSegment.getFramePosition());
   }

}
