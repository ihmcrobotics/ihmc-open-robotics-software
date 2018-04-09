package us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFrameVector2D;

public class ManualDesiredVelocityControlModule implements DesiredVelocityControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoFrameVector2D desiredVelocity;

   public ManualDesiredVelocityControlModule(ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry)
   {
      desiredVelocity = new YoFrameVector2D("desiredVelocity", "", referenceFrame, registry);
      parentRegistry.addChild(registry);
   }

   public void setDesiredVelocity(FrameVector2DReadOnly newDesiredVelocity)
   {
      desiredVelocity.setMatchingFrame(newDesiredVelocity);
   }

   public FrameVector2DReadOnly getDesiredVelocity()
   {
      return desiredVelocity;
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return desiredVelocity.getReferenceFrame();
   }
}
