package us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity;

import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector2d;

public class SimpleDesiredVelocityControlModule implements DesiredVelocityControlModule
{
   private final BooleanYoVariable velocityAlwaysFacesHeading;
   private final DoubleYoVariable desiredVelocityNorm;
   private final YoFrameVector2d desiredVelocity;

   private final DesiredHeadingControlModule desiredHeadingControlModule;

   public SimpleDesiredVelocityControlModule(DesiredHeadingControlModule desiredHeadingControlModule, double initialDesiredVelocity,
           YoVariableRegistry yoVariableRegistry)
   {
      this.desiredHeadingControlModule = desiredHeadingControlModule;

      this.velocityAlwaysFacesHeading = new BooleanYoVariable("velocityAlwaysFacesHeading", yoVariableRegistry);

      this.desiredVelocityNorm = new DoubleYoVariable("desiredVelocityNorm", yoVariableRegistry);
      this.desiredVelocityNorm.set(initialDesiredVelocity);

      this.desiredVelocity = new YoFrameVector2d("desiredVelocity", "", ReferenceFrame.getWorldFrame(), yoVariableRegistry);
      velocityAlwaysFacesHeading.set(true);

      updateDesiredVelocity();
   }

   public void setDesiredVelocity(FrameVector2d newDesiredVelocity)
   {
      desiredVelocity.set(newDesiredVelocity);
   }

   public FrameVector2d getDesiredVelocity()
   {
      return new FrameVector2d(desiredVelocity.getReferenceFrame(), desiredVelocity.getX(), desiredVelocity.getY());
   }
   
   public void setVelocityAlwaysFacesHeading(boolean velocityAlwaysFacesHeading)
   {
      this.velocityAlwaysFacesHeading.set(velocityAlwaysFacesHeading);
   }
   
   public boolean getVelocityAlwaysFacesHeading()
   {
      return this.velocityAlwaysFacesHeading.getBooleanValue();
   }

   public void updateDesiredVelocity()
   {
      if (velocityAlwaysFacesHeading.getBooleanValue())
      {
         double desiredHeading = this.desiredHeadingControlModule.getDesiredHeadingAngle();
         this.desiredVelocity.set(desiredVelocityNorm.getDoubleValue() * Math.cos(desiredHeading),
                                  desiredVelocityNorm.getDoubleValue() * Math.sin(desiredHeading));
      }
   }

}
