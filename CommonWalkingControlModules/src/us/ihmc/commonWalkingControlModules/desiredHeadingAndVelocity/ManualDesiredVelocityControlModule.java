package us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector2d;

import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public class ManualDesiredVelocityControlModule implements DesiredVelocityControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry("SimpleDesiredVelocityControlModule");
   private final YoFrameVector2d desiredVelocity = new YoFrameVector2d("desiredVelocity", "", ReferenceFrame.getWorldFrame(), registry);
   
   public ManualDesiredVelocityControlModule(YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
   }
   
   public void setDesiredVelocity(FrameVector2d newDesiredVelocity)
   {
      desiredVelocity.set(newDesiredVelocity);
   }

   public FrameVector2d getDesiredVelocity()
   {
      return desiredVelocity.getFrameVector2dCopy();
   }

   public void updateDesiredVelocity()
   {
      throw new RuntimeException("Set velocity manually, don't call this method.");
   }
}
