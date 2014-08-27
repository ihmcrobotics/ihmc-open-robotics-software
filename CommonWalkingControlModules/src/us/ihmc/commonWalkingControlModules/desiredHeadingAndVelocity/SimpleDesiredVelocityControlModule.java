package us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity;

import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector2d;

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

   public FrameVector2d getDesiredVelocity()
   {
      return new FrameVector2d(desiredVelocity.getReferenceFrame(), desiredVelocity.getX(), desiredVelocity.getY());
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
