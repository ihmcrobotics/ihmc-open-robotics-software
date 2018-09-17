package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.AMGeneration;

import us.ihmc.euclid.Axis;
import us.ihmc.robotics.math.trajectories.FrameTrajectory3D;
import us.ihmc.robotics.math.trajectories.SegmentedFrameTrajectory3D;
import us.ihmc.robotics.math.trajectories.TrajectoryMathTools;

public class TorqueTrajectory extends SegmentedFrameTrajectory3D
{
   public TorqueTrajectory(int maxNumberOfSegments, int maxNumberOfCoefficients)
   {
      super(maxNumberOfSegments, maxNumberOfCoefficients);
   }

   public void setNext(AngularMomentumTrajectory angularMomentumTrajectory)
   {
      this.reset();
      for(int i = 0; i < angularMomentumTrajectory.getNumberOfSegments(); i++)
      {
         FrameTrajectory3D segment = segments.add();
         TrajectoryMathTools.getDerivative(segment.getTrajectoryY(), angularMomentumTrajectory.getSegment(i).getTrajectoryX());
         TrajectoryMathTools.scale(segment.getTrajectoryY(), -1.0);
         TrajectoryMathTools.getDerivative(segment.getTrajectoryX(), angularMomentumTrajectory.getSegment(i).getTrajectoryY());
         segment.getTrajectoryZ().setConstant(segment.getInitialTime(Axis.X), segment.getFinalTime(Axis.X), 0.0);
      }
   }
   
   public void scale(double scalar)
   {
      for(int i = 0; i < getNumberOfSegments(); i++)
         TrajectoryMathTools.scale(scalar, segments.get(i));
   }
}
