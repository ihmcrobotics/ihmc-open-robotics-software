package us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;


public class ManualDesiredVelocityControlModule implements DesiredVelocityControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoFrameVector2D desiredVelocity;

   public ManualDesiredVelocityControlModule(ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry)
   {
      desiredVelocity = new YoFrameVector2D("desiredVelocity", "", referenceFrame, registry);
      parentRegistry.addChild(registry);
   }

   public void setDesiredVelocity(FrameVector2D newDesiredVelocity)
   {
      newDesiredVelocity.changeFrame(desiredVelocity.getReferenceFrame());
      desiredVelocity.set(newDesiredVelocity);
   }

   @Override
   public void getDesiredVelocity(FrameVector2D desiredVelocityToPack)
   {
      desiredVelocityToPack.setIncludingFrame(desiredVelocity);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return desiredVelocity.getReferenceFrame();
   }
}
