package us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator;

import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP.YoSegmentedFrameTrajectory3D;
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
         angMomTraj.getSegment(i).getDerivative(segments.get(i));
         numberOfSegments.increment();
      }
   }
   
   public void scale(double scalar)
   {
      for(int i = 0; i < getNumberOfSegments(); i++)
         segments.get(i).scale(scalar);
   }
   
}
