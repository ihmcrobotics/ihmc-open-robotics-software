package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP;

import java.util.List;

import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.YoFrameTrajectory3D;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameTuple;
import us.ihmc.robotics.geometry.FrameVector;

public interface SegmentedFrameTrajectory3DInterface
{
   void reset();
   List<YoFrameTrajectory3D> getSegments();
   YoFrameTrajectory3D getSegment(int index);
   YoFrameTrajectory3D getCurrentSegment(double timeInState);
   public double[] getNodeTimes();
   int getNumberOfSegments();
}