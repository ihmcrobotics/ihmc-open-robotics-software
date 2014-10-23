package us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity;

import us.ihmc.sensorProcessing.ProcessedSensorsInterface;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;

public class VelocityErrorCalculator
{
   private final DesiredVelocityControlModule desiredVelocityControlModule;
   private final ProcessedSensorsInterface processedSensors;
   
   public VelocityErrorCalculator(ProcessedSensorsInterface processedSensors, DesiredVelocityControlModule desiredVelocityControlModule)
   {
      this.processedSensors = processedSensors;
      this.desiredVelocityControlModule = desiredVelocityControlModule;
      
   }
   
   public FrameVector2d getVelocityErrorInFrame(ReferenceFrame referenceFrame, RobotSide legToTrustForCoMVelocity)
   {
      if (!referenceFrame.isZupFrame())
      {
         throw new RuntimeException("Must be a ZUp frame!");
      }

      FrameVector centerOfMassVelocity = processedSensors.getCenterOfMassVelocityInFrame(referenceFrame);
      FrameVector2d centerOfMassVelocity2d = centerOfMassVelocity.toFrameVector2d();

      FrameVector2d desiredCenterOfMassVelocity = desiredVelocityControlModule.getDesiredVelocity();
      desiredCenterOfMassVelocity.changeFrame(referenceFrame);

      FrameVector2d ret = new FrameVector2d(referenceFrame);
      ret.sub(desiredCenterOfMassVelocity, centerOfMassVelocity2d);

      return ret;
   }

}
