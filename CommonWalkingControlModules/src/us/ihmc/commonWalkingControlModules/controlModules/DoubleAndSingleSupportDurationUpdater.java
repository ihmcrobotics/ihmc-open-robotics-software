package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class DoubleAndSingleSupportDurationUpdater
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final CommonWalkingReferenceFrames referenceFrames;

   private final DoubleYoVariable doubleSupportFraction = new DoubleYoVariable("doubleSupportFraction", registry);
   private final DoubleYoVariable stepInPlaceCycleDuration = new DoubleYoVariable("stepInPlaceCycleDuration", registry);
   private final DoubleYoVariable doubleSupportDuration = new DoubleYoVariable("doubleSupportDuration", registry);
   private final DoubleYoVariable singleSupportDuration = new DoubleYoVariable("singleSupportDuration", registry);

   public DoubleAndSingleSupportDurationUpdater(CommonWalkingReferenceFrames referenceFrames, YoVariableRegistry parentRegistry)
   {
      this.referenceFrames = referenceFrames;
      parentRegistry.addChild(registry);
      setParameters();
   }

   public void update(Footstep desiredFootstep, RobotSide supportLeg, FrameVector2d desiredVelocity)
   {
      ReferenceFrame supportAnkleZUpFrame = referenceFrames.getAnkleZUpFrame(supportLeg);
      FramePoint desiredFootstepPosition = new FramePoint();
      desiredFootstep.getPositionIncludingFrame(desiredFootstepPosition);
      desiredFootstepPosition.changeFrame(supportAnkleZUpFrame);
      FrameVector2d desiredStepVector = new FrameVector2d(desiredFootstepPosition.toFramePoint2d());
      desiredVelocity.changeFrame(supportAnkleZUpFrame);
      double desiredVelocityMagnitude = desiredVelocity.length();
      double stepDistanceAlongDesiredVelocity = desiredStepVector.dot(desiredVelocity) / desiredVelocityMagnitude;
      double bodyDisplacement = stepDistanceAlongDesiredVelocity;
      double cycleDuration;
      double epsilon = 1e-7;
      if (desiredVelocityMagnitude < epsilon)
      {
         cycleDuration = stepInPlaceCycleDuration.getDoubleValue();
      }
      else
      {
         cycleDuration = Math.abs(bodyDisplacement / desiredVelocityMagnitude);
      }
      doubleSupportDuration.set(doubleSupportFraction.getDoubleValue() * cycleDuration);
      singleSupportDuration.set((1.0 - doubleSupportFraction.getDoubleValue()) * cycleDuration);
   }

   public double getDoubleSupportDuration()
   {
      return doubleSupportDuration.getDoubleValue();
   }

   public double getSingleSupportDuration()
   {
      return singleSupportDuration.getDoubleValue();
   }

   private void setParameters()
   {
      doubleSupportFraction.set(0.4);
      stepInPlaceCycleDuration.set(1.1);
   }
}
