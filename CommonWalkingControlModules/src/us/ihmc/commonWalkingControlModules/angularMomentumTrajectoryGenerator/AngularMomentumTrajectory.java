package us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator;

import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP.WalkingTrajectoryType;
import us.ihmc.robotics.math.trajectories.YoSegmentedFrameTrajectory3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.math.frames.YoFrameVector;
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
   public void update(double timeInState, FrameVector3D desiredAngularMomentumToPack)
   {
      update(timeInState);
      desiredAngularMomentumToPack.setIncludingFrame(currentSegment.getFramePosition());
   }

   @Override
   public void update(double timeInState, FrameVector3D desiredAngularMomentumToPack, FrameVector3D desiredTorqueToPack)
   {
      update(timeInState, desiredAngularMomentumToPack);
      desiredTorqueToPack.setIncludingFrame(currentSegment.getFrameVelocity());
   }

   @Override
   public void update(double timeInState, FrameVector3D desiredAngularMomentumToPack, FrameVector3D desiredTorqueToPack, FrameVector3D desiredRotatumToPack)
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

   public void set(double t0, double tFinal, FramePoint3D z0, FramePoint3D zf)
   {
      segments.get(getNumberOfSegments()).setLinear(t0, tFinal, z0, zf);
      numberOfSegments.increment();
   }
}
