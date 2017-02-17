package us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class RateBasedDesiredHeadingControlModule implements DesiredHeadingControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final DoubleYoVariable desiredHeading = new DoubleYoVariable("desiredHeading", registry);
   private final DoubleYoVariable desiredHeadingDot = new DoubleYoVariable("desiredHeadingDot", registry);

   private final DesiredHeadingFrame desiredHeadingFrame = new DesiredHeadingFrame();
   private final DesiredHeadingFrame predictedHeadingFrame = new DesiredHeadingFrame();

   private final double controlDT;

   public RateBasedDesiredHeadingControlModule(double initialDesiredHeading, double controlDT, YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
      this.controlDT = controlDT;
      desiredHeading.set(initialDesiredHeading);

      updateDesiredHeadingFrame();
   }

   @Override
   public void updateDesiredHeadingFrame()
   {
      updateDesiredHeading();
      desiredHeadingFrame.setHeadingAngleAndUpdate(desiredHeading.getDoubleValue());
   }

   @Override
   public double getFinalHeadingTargetAngle()
   {
      throw new RuntimeException("Don't use this. It should be removed from the interface");
   }

   @Override
   public FrameVector2d getFinalHeadingTarget()
   {
      throw new RuntimeException("Don't use this. It should be removed from the interface");
   }

   @Override
   public ReferenceFrame getDesiredHeadingFrame()
   {
      return desiredHeadingFrame;
   }

   @Override
   public ReferenceFrame getPredictedHeadingFrame(double timeFromNow)
   {
      double predictedHeading = predictHeading(timeFromNow);
      predictedHeadingFrame.setHeadingAngleAndUpdate(predictedHeading);
      return predictedHeadingFrame;
   }

   private double predictHeading(double timeFromNow)
   {
      double deltaHeading = desiredHeadingDot.getDoubleValue() * timeFromNow;
      return desiredHeading.getDoubleValue() + deltaHeading;
   }

   @Override
   public void setFinalHeadingTarget(FrameVector2d finalHeadingTarget)
   {
      finalHeadingTarget.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      setFinalHeadingTargetAngle(Math.atan2(finalHeadingTarget.getY(), finalHeadingTarget.getX()));
   }

   @Override
   public double getDesiredHeadingAngle()
   {
      return desiredHeading.getDoubleValue();
   }

   @Override
   public void getDesiredHeading(FrameVector2d desiredHeadingToPack, double timeFromNow)
   {
      double heading = predictHeading(timeFromNow);
      desiredHeadingToPack.setIncludingFrame(ReferenceFrame.getWorldFrame(), Math.cos(heading), Math.sin(heading));
   }

   @Override
   public void resetHeadingAngle(double newHeading)
   {
      desiredHeading.set(newHeading);
   }

   private void updateDesiredHeading()
   {
      double deltaHeading = desiredHeadingDot.getDoubleValue() * controlDT;

      desiredHeading.set(desiredHeading.getDoubleValue() + deltaHeading);
   }

   public static class DesiredHeadingFrame extends ReferenceFrame
   {
      private static final long serialVersionUID = 4657294310129415811L;

      public DesiredHeadingFrame()
      {
         super("DesiredHeadingFrame", ReferenceFrame.getWorldFrame(), false, false, true);
      }

      private final RotationMatrix rotation = new RotationMatrix();

      @Override
      protected void updateTransformToParent(RigidBodyTransform transformToParent)
      {

         transformToParent.setRotationAndZeroTranslation(rotation);
      }

      public void setHeadingAngleAndUpdate(double headingAngle)
      {
         rotation.setToYawMatrix(headingAngle);
         update();
      }
   }

   @Override
   public void setFinalHeadingTargetAngle(double finalHeadingTargetAngle)
   {
      throw new RuntimeException("Don't use this. It should be removed from the interface");
   }
}
