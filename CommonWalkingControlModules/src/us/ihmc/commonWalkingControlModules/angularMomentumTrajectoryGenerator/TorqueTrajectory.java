package us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator;

import us.ihmc.robotics.math.trajectories.TrajectoryMathTools;
import us.ihmc.robotics.math.trajectories.YoSegmentedFrameTrajectory3D;
import us.ihmc.robotics.geometry.Direction;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class TorqueTrajectory extends YoSegmentedFrameTrajectory3D
{
   private final TrajectoryMathTools trajectoryMathTools;
   public TorqueTrajectory(String name, int maxNumberOfSegments, int maxNumberOfCoefficients, YoVariableRegistry registry)
   {
      super(name, maxNumberOfSegments, maxNumberOfCoefficients, registry);
      trajectoryMathTools = new TrajectoryMathTools(name + "TorqueTrajectory", maxNumberOfCoefficients, registry);
   }
   
   public void set(AngularMomentumTrajectory angMomTraj)
   {
      this.reset();
      for(int i = 0; i < angMomTraj.getNumberOfSegments(); i++)
      {
         trajectoryMathTools.getDerivative(segments.get(i).getYoTrajectoryY(), angMomTraj.getSegment(i).getYoTrajectoryX());
         trajectoryMathTools.scale(segments.get(i).getYoTrajectoryY(), segments.get(i).getYoTrajectoryY(), -1.0);
         trajectoryMathTools.getDerivative(segments.get(i).getYoTrajectoryX(), angMomTraj.getSegment(i).getYoTrajectoryY());
         segments.get(i).getYoTrajectoryZ().setConstant(segments.get(i).getInitialTime(Direction.X), segments.get(i).getFinalTime(Direction.X), 0.0);
         numberOfSegments.increment();
      }
   }
   
   public void scale(double scalar)
   {
      for(int i = 0; i < getNumberOfSegments(); i++)
         trajectoryMathTools.scale(segments.get(i), segments.get(i), scalar);
   }
}
