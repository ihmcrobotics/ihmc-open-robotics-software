package us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity;

import javax.vecmath.Matrix3d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;


public class RateBasedDesiredHeadingControlModule implements DesiredHeadingControlModule
{  
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final DoubleYoVariable desiredHeading = new DoubleYoVariable("desiredHeading", registry);
   private final DoubleYoVariable desiredHeadingDot = new DoubleYoVariable("desiredHeadingDot", registry);

   private final DesiredHeadingFrame desiredHeadingFrame = new DesiredHeadingFrame();

   private final double controlDT;
   
   public RateBasedDesiredHeadingControlModule(double initialDesiredHeading, double controlDT,
           YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
      this.controlDT = controlDT;
      this.desiredHeading.set(initialDesiredHeading);
      
      updateDesiredHeadingFrame();
   }

   public void updateDesiredHeadingFrame()
   {
      updateDesiredHeading();
      desiredHeadingFrame.update();
   }

   public double getFinalHeadingTargetAngle()
   {
      throw new RuntimeException("Don't use this. It should be removed from the interface");
   }
   
   public FrameVector2d getFinalHeadingTarget()
   {
      throw new RuntimeException("Don't use this. It should be removed from the interface");
   }

   public ReferenceFrame getDesiredHeadingFrame()
   {
      return desiredHeadingFrame;
   }

   public void setFinalHeadingTarget(FrameVector2d finalHeadingTarget)
   {
      finalHeadingTarget.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      setFinalHeadingTargetAngle(Math.atan2(finalHeadingTarget.getY(), finalHeadingTarget.getX()));
   }

   public double getDesiredHeadingAngle()
   {      
      return desiredHeading.getDoubleValue();
   }
   
   public void getDesiredHeading(FrameVector2d desiredHeadingToPack)
   {
      desiredHeadingToPack.setIncludingFrame(ReferenceFrame.getWorldFrame(), Math.cos(desiredHeading.getDoubleValue()),
                                    Math.sin(desiredHeading.getDoubleValue()));
   }

   public void resetHeadingAngle(double newHeading)
   {
      this.desiredHeading.set(newHeading);
   }

   private void updateDesiredHeading()
   {
      double deltaHeading = desiredHeadingDot.getDoubleValue() * controlDT;

      desiredHeading.set(desiredHeading.getDoubleValue() + deltaHeading);
   }

   private class DesiredHeadingFrame extends ReferenceFrame
   {
      private static final long serialVersionUID = 4657294310129415811L;

      public DesiredHeadingFrame()
      {
         super("DesiredHeadingFrame", ReferenceFrame.getWorldFrame(), false, false, true);
      }

      protected void updateTransformToParent(RigidBodyTransform transformToParent)
      {
         Matrix3d rotation = new Matrix3d();
         rotation.rotZ(desiredHeading.getDoubleValue());

         transformToParent.setRotationAndZeroTranslation(rotation);
      }
   }

   public void setFinalHeadingTargetAngle(double finalHeadingTargetAngle)
   {
      throw new RuntimeException("Don't use this. It should be removed from the interface");      
   }
}
