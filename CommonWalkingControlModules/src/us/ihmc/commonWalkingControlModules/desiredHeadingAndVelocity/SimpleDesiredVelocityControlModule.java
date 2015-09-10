package us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoUtilities.math.frames.YoFrameVector2d;


public class SimpleDesiredVelocityControlModule implements DesiredVelocityControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry("SimpleDesiredVelocityControlModule");
   private final BooleanYoVariable desiredVelocityAlwaysFacesHeading = new BooleanYoVariable("desiredVelocityAlwaysFacesHeading", registry);
   private final DoubleYoVariable desiredVelocityNorm = new DoubleYoVariable("desiredVelocityNorm", registry);
   private final YoFrameVector2d desiredVelocity = new YoFrameVector2d("desiredVelocity", "", ReferenceFrame.getWorldFrame(), registry);

   private final DesiredHeadingControlModule desiredHeadingControlModule;

   public SimpleDesiredVelocityControlModule(DesiredHeadingControlModule desiredHeadingControlModule, double initialDesiredVelocity,
           YoVariableRegistry parentRegistry)
   {
      this.desiredHeadingControlModule = desiredHeadingControlModule;

      desiredVelocityNorm.set(initialDesiredVelocity);
      desiredVelocityAlwaysFacesHeading.set(true);

      parentRegistry.addChild(registry);

      updateDesiredVelocity();
   }

   public void getDesiredVelocity(FrameVector2d desiredVelocityToPack)
   {
      desiredVelocityToPack.setIncludingFrame(desiredVelocity.getReferenceFrame(), desiredVelocity.getX(), desiredVelocity.getY());
   }

   public void updateDesiredVelocity()
   {
      if (desiredVelocityAlwaysFacesHeading.getBooleanValue())
      {
         double desiredHeading = this.desiredHeadingControlModule.getDesiredHeadingAngle();
         this.desiredVelocity.set(desiredVelocityNorm.getDoubleValue() * Math.cos(desiredHeading),
                                  desiredVelocityNorm.getDoubleValue() * Math.sin(desiredHeading));
      }
   }

}
