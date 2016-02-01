package us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity;

import us.ihmc.sensorProcessing.ProcessedSensorsInterface;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class VelocityErrorCalculator
{
   private final DesiredVelocityControlModule desiredVelocityControlModule;
   private final ProcessedSensorsInterface processedSensors;
   
   public VelocityErrorCalculator(ProcessedSensorsInterface processedSensors, DesiredVelocityControlModule desiredVelocityControlModule)
   {
      this.processedSensors = processedSensors;
      this.desiredVelocityControlModule = desiredVelocityControlModule;
      
   }
   
   private final FrameVector2d desiredCenterOfMassVelocity = new FrameVector2d();
   
   public FrameVector2d getVelocityErrorInFrame(ReferenceFrame referenceFrame, RobotSide legToTrustForCoMVelocity)
   {
      if (!referenceFrame.isZupFrame())
      {
         throw new RuntimeException("Must be a ZUp frame!");
      }

      FrameVector centerOfMassVelocity = processedSensors.getCenterOfMassVelocityInFrame(referenceFrame);
      FrameVector2d centerOfMassVelocity2d = centerOfMassVelocity.toFrameVector2d();

      desiredVelocityControlModule.getDesiredVelocity(desiredCenterOfMassVelocity);
      desiredCenterOfMassVelocity.changeFrame(referenceFrame);

      FrameVector2d ret = new FrameVector2d(referenceFrame);
      ret.sub(desiredCenterOfMassVelocity, centerOfMassVelocity2d);

      return ret;
   }

}
