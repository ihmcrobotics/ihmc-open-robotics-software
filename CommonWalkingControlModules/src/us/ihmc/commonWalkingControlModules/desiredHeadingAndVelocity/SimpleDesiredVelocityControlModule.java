package us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity;

import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;

public class SimpleDesiredVelocityControlModule implements DesiredVelocityControlModule
{
   private final boolean VELOCITY_AND_HEADING_ARE_DEPENDENT = true;

   private final DoubleYoVariable desiredVelocityNorm;
   private final YoFrameVector desiredVelocity;

   private final ProcessedSensorsInterface processedSensors;

   private final DesiredHeadingControlModule desiredHeadingControlModule;

   public SimpleDesiredVelocityControlModule(ProcessedSensorsInterface processedSensors, DesiredHeadingControlModule desiredHeadingControlModule,
           double initialDesiredVelocity, YoVariableRegistry yoVariableRegistry)

// public SimpleDesiredVelocityControlModule(ProcessedSensors processedSensors, DesiredHeadingControlModule desiredHeadingControlModule,
//         YoFrameVector initialDesiredVelocity, YoVariableRegistry yoVariableRegistry)
   {
      this.processedSensors = processedSensors;
      this.desiredHeadingControlModule = desiredHeadingControlModule;

//    this.desiredVelocity = new DoubleYoVariable("desiredVelocity", yoVariableRegistry);
//    this.desiredVelocity.set(initialDesiredVelocity);

      this.desiredVelocityNorm = new DoubleYoVariable("desiredVelocityNorm", yoVariableRegistry);
      this.desiredVelocityNorm.set(initialDesiredVelocity);


      if (VELOCITY_AND_HEADING_ARE_DEPENDENT)

         // Desired Velocity and Desired Heading are dependent (Desired Velocity is expressed in the desired heading frame)
         this.desiredVelocity = new YoFrameVector("desiredVelocity", "", desiredHeadingControlModule.getDesiredHeadingFrame(), yoVariableRegistry);


      else

         // Desired Velocity and Desired Heading are independent
         this.desiredVelocity = new YoFrameVector("desiredVelocity", "", ReferenceFrame.getWorldFrame(), yoVariableRegistry);

      updateDesiredVelocity();


   }

   public FrameVector2d getDesiredVelocity()
   {
      return new FrameVector2d(desiredVelocity.getReferenceFrame(), desiredVelocity.getX(), desiredVelocity.getY());

//    return new FrameVector2d(desiredHeadingControlModule.getDesiredHeadingFrame(), desiredVelocityNorm.getDoubleValue(), 0.0);
   }

   public FrameVector2d getVelocityErrorInFrame(ReferenceFrame referenceFrame)
   {
      if (!referenceFrame.isZupFrame())
      {
         throw new RuntimeException("Must be a ZUp frame!");
      }

      FrameVector centerOfMassVelocity = processedSensors.getCenterOfMassVelocityInFrame(referenceFrame);
      FrameVector2d centerOfMassVelocity2d = centerOfMassVelocity.toFrameVector2d();

      FrameVector2d desiredCenterOfMassVelocity = getDesiredVelocity();
      desiredCenterOfMassVelocity = desiredCenterOfMassVelocity.changeFrameCopy(referenceFrame);

      FrameVector2d ret = new FrameVector2d(referenceFrame);
      ret.sub(desiredCenterOfMassVelocity, centerOfMassVelocity2d);

      return ret;
   }

   public void updateDesiredVelocity()
   {
      if (VELOCITY_AND_HEADING_ARE_DEPENDENT)
      {
         this.desiredVelocity.set(desiredVelocityNorm.getDoubleValue(), 0.0, 0.0);
      }
      else
      {
         double desiredHeading = this.desiredHeadingControlModule.getDesiredHeading().getDoubleValue();
         this.desiredVelocity.set(desiredVelocityNorm.getDoubleValue() * Math.cos(desiredHeading),
                                  desiredVelocityNorm.getDoubleValue() * Math.sin(desiredHeading), 0.0);
      }

   }

}
