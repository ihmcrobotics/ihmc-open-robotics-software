package us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator;

import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP.WalkingTrajectoryType;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP.YoSegmentedFrameTrajectory3D;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class AngularMomentumTrajectory extends YoSegmentedFrameTrajectory3D implements AngularMomentumTrajectoryInterface
{
   private YoFrameVector momentum;
   private YoFrameVector torque;
   private YoFrameVector rotatum;

   public AngularMomentumTrajectory(String namePrefix, int stepNumber, WalkingTrajectoryType type, YoVariableRegistry registry, ReferenceFrame referenceFrame,
                                    int maxNumberOfSegments, int maxNumberOfCoefficients)
   {
      super(namePrefix + stepNumber + type.toString(), maxNumberOfSegments, maxNumberOfCoefficients, registry);
      momentum = new YoFrameVector(namePrefix + stepNumber + type.toString() + "Positon", referenceFrame, registry);
      torque = new YoFrameVector(namePrefix + stepNumber + type.toString() + "Velocity", referenceFrame, registry);
      rotatum = new YoFrameVector(namePrefix + stepNumber + type.toString() + "Acceleration", referenceFrame, registry);
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

   public void set(YoFrameTrajectory3D computedAngularMomentumTrajectory)
   {
      segments.get(getCurrentSegmentIndex()).set(computedAngularMomentumTrajectory);
      currentSegmentIndex.increment();
   }
}
