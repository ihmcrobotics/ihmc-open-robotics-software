package us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity;

import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.math.frames.YoFrameVector2d;


public class ManualDesiredVelocityControlModule implements DesiredVelocityControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoFrameVector2d desiredVelocity;
   
   public ManualDesiredVelocityControlModule(ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry)
   {
      desiredVelocity = new YoFrameVector2d("desiredVelocity", "", referenceFrame, registry);
      parentRegistry.addChild(registry);
   }
   
   public void setDesiredVelocity(FrameVector2d newDesiredVelocity)
   {
      newDesiredVelocity.changeFrame(desiredVelocity.getReferenceFrame());
      desiredVelocity.set(newDesiredVelocity);  
   }

   public FrameVector2d getDesiredVelocity()
   {
      return desiredVelocity.getFrameVector2dCopy();
   }

   public void updateDesiredVelocity()
   {
//      throw new RuntimeException("Set velocity manually, don't call this method.");
   }
}
