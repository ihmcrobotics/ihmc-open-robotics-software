package us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator;

import us.ihmc.robotics.math.trajectories.YoSegmentedFrameTrajectory3D;
import us.ihmc.robotics.geometry.Direction;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class TorqueTrajectory extends YoSegmentedFrameTrajectory3D
{

   public TorqueTrajectory(String name, int maxNumberOfSegments, int maxNumberOfCoefficients, YoVariableRegistry registry)
   {
      super(name, maxNumberOfSegments, maxNumberOfCoefficients, registry);
   }
   
   public void set(AngularMomentumTrajectory angMomTraj)
   {
      this.reset();
      for(int i = 0; i < angMomTraj.getNumberOfSegments(); i++)
      {
         angMomTraj.getSegment(i).getYoTrajectoryX().getDerivative(segments.get(i).getYoTrajectoryY());
         segments.get(i).getYoTrajectoryY().scale(-1.0);
         angMomTraj.getSegment(i).getYoTrajectoryY().getDerivative(segments.get(i).getYoTrajectoryX());
         segments.get(i).getYoTrajectoryZ().setConstant(segments.get(i).getInitialTime(Direction.X), segments.get(i).getFinalTime(Direction.X), 0.0);
         numberOfSegments.increment();
      }
   }
   
   public void scale(double scalar)
   {
      for(int i = 0; i < getNumberOfSegments(); i++)
         segments.get(i).scale(scalar);
   }
   
}
