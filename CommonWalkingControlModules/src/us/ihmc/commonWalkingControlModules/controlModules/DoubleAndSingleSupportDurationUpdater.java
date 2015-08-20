package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.humanoidRobotics.footstep.Footstep;


public class DoubleAndSingleSupportDurationUpdater
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final CommonHumanoidReferenceFrames referenceFrames;

   private final DoubleYoVariable doubleSupportFraction = new DoubleYoVariable("doubleSupportFraction", registry);
   private final DoubleYoVariable stepInPlaceCycleDuration = new DoubleYoVariable("stepInPlaceCycleDuration", registry);
   private final DoubleYoVariable doubleSupportDuration = new DoubleYoVariable("doubleSupportDuration", registry);
   private final DoubleYoVariable singleSupportDuration = new DoubleYoVariable("singleSupportDuration", registry);

   public DoubleAndSingleSupportDurationUpdater(CommonHumanoidReferenceFrames referenceFrames, YoVariableRegistry parentRegistry)
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
