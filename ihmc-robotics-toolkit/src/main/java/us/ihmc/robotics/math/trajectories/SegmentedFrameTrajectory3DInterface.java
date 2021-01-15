package us.ihmc.robotics.math.trajectories;

import java.util.List;

public interface SegmentedFrameTrajectory3DInterface
{
   void reset();
   List<?  extends FrameTrajectory3D> getSegments();
   FrameTrajectory3D getSegment(int index);
   FrameTrajectory3D getCurrentSegment(double timeInState);
   public double[] getNodeTimes();
   int getNumberOfSegments();
}