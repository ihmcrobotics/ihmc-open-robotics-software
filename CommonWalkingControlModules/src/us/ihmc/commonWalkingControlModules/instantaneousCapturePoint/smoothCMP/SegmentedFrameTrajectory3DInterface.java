package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP;

import java.util.List;

import us.ihmc.robotics.trajectories.YoFrameTrajectory3D;

public interface SegmentedFrameTrajectory3DInterface
{
   void reset();
   List<YoFrameTrajectory3D> getSegments();
   YoFrameTrajectory3D getSegment(int index);
   YoFrameTrajectory3D getCurrentSegment(double timeInState);
   public double[] getNodeTimes();
   int getNumberOfSegments();
}