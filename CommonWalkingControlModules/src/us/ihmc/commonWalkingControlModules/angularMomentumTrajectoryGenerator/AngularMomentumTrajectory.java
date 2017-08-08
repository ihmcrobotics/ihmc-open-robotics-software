package us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator;

import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP.WalkingTrajectoryType;
import us.ihmc.robotics.math.trajectories.YoSegmentedFrameTrajectory3D;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.math.trajectories.YoFrameTrajectory3D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class AngularMomentumTrajectory extends YoSegmentedFrameTrajectory3D implements AngularMomentumTrajectoryInterface
{
   private YoFrameVector momentum;
   private YoFrameVector torque;
   private YoFrameVector rotatum;
   private YoFrameTrajectory3D torqueTrajectory;

   public AngularMomentumTrajectory(String namePrefix, int stepNumber, WalkingTrajectoryType type, YoVariableRegistry registry, ReferenceFrame referenceFrame,
                                    int maxNumberOfSegments, int maxNumberOfCoefficients)
   {
      super(namePrefix + stepNumber + type.toString() + "AngularMomentum", maxNumberOfSegments, maxNumberOfCoefficients, registry);
      momentum = new YoFrameVector(namePrefix + stepNumber + type.toString() + "Position", referenceFrame, registry);
      torque = new YoFrameVector(namePrefix + stepNumber + type.toString() + "Velocity", referenceFrame, registry);
      rotatum = new YoFrameVector(namePrefix + stepNumber + type.toString() + "Acceleration", referenceFrame, registry);
      torqueTrajectory = new YoFrameTrajectory3D(namePrefix + stepNumber + type.toString() + "TorqueTrajectory", maxNumberOfCoefficients - 1, referenceFrame,
                                                 registry);
   }

   @Override
   public void reset()
   {
      super.reset();
      momentum.setToNaN();
      torque.setToNaN();
      rotatum.setToNaN();
   }

   @Override
   public void update(double timeInState, FrameVector desiredAngularMomentumToPack)
   {
      update(timeInState);
      desiredAngularMomentumToPack.setIncludingFrame(currentSegment.getFramePosition());
   }

   @Override
   public void update(double timeInState, FrameVector desiredAngularMomentumToPack, FrameVector desiredTorqueToPack)
   {
      update(timeInState, desiredAngularMomentumToPack);
      desiredTorqueToPack.setIncludingFrame(currentSegment.getFrameVelocity());
   }

   @Override
   public void update(double timeInState, FrameVector desiredAngularMomentumToPack, FrameVector desiredTorqueToPack, FrameVector desiredRotatumToPack)
   {
      update(timeInState, desiredAngularMomentumToPack, desiredTorqueToPack);
      desiredRotatumToPack.setIncludingFrame(currentSegment.getFrameAcceleration());
   }

   @Override
   public void set(YoFrameTrajectory3D computedAngularMomentumTrajectory)
   {
      segments.get(getNumberOfSegments()).set(computedAngularMomentumTrajectory);
      numberOfSegments.increment();
   }

   public void set(double t0, double tFinal, FramePoint z0, FramePoint zf)
   {
      segments.get(getNumberOfSegments()).setLinear(t0, tFinal, z0, zf);
      numberOfSegments.increment();
   }

   public YoFrameTrajectory3D getTorqueTrajectory(int segmentIndex)
   {
      segments.get(segmentIndex).getDerivative(torqueTrajectory);
      return torqueTrajectory;
   }
}
