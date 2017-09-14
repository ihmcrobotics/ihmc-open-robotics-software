package us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator;

import us.ihmc.robotics.geometry.Direction;
import us.ihmc.robotics.math.trajectories.SegmentedFrameTrajectory3D;
import us.ihmc.robotics.math.trajectories.TrajectoryMathTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class TorqueTrajectory extends SegmentedFrameTrajectory3D
{
   public TorqueTrajectory(int maxNumberOfSegments, int maxNumberOfCoefficients)
   {
      super(maxNumberOfSegments, maxNumberOfCoefficients);
   }
   
   public void setNext(AngularMomentumTrajectory angMomTraj)
   {
      this.reset();
      for(int i = 0; i < angMomTraj.getNumberOfSegments(); i++)
      {
         TrajectoryMathTools.getDerivative(segments.get(i).getTrajectoryY(), angMomTraj.getSegment(i).getTrajectoryX());
         TrajectoryMathTools.scale(segments.get(i).getTrajectoryY(), -1.0);
         TrajectoryMathTools.getDerivative(segments.get(i).getTrajectoryX(), angMomTraj.getSegment(i).getTrajectoryY());
         segments.get(i).getTrajectoryZ().setConstant(segments.get(i).getInitialTime(Direction.X), segments.get(i).getFinalTime(Direction.X), 0.0);
         numberOfSegments++;
      }
   }
   
   public void scale(double scalar)
   {
      for(int i = 0; i < getNumberOfSegments(); i++)
         TrajectoryMathTools.scale(scalar, segments.get(i));
   }
}
