package us.ihmc.commonWalkingControlModules.controlModules.leapOfFaith;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class PelvisLeapOfFaithModule
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoFrameOrientation orientationOffset = new YoFrameOrientation("leapOfFaithPelvisOrientationOffset", worldFrame, registry);

   private boolean isInTransfer = true;
   private double swingDuration;

   private final double yawGain = 10.0;
   private final double rollGain = 10.0;

   public PelvisLeapOfFaithModule(YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
   }

   public void initializeTransfer()
   {
      isInTransfer = true;
   }

   public void initializeSwing(double swingDuration)
   {
      this.swingDuration = swingDuration;
      isInTransfer = false;
   }

   public void computeAndAddAngularOffset(double currentTimeInState, FrameOrientation orientationToPack)
   {
      //// TODO: 6/8/17 make this side dependent and step length dependent 
      orientationOffset.setToZero();

      if (!isInTransfer)
      {
         double exceededTime = Math.max(currentTimeInState - swingDuration, 0.0);

         double yawAngleOffset = yawGain * exceededTime;
         double rollAngleOffset = rollGain * exceededTime;

         orientationOffset.setRoll(rollAngleOffset);
         orientationOffset.setYaw(yawAngleOffset);
      }

      orientationToPack.preMultiply(orientationOffset.getFrameOrientation());
   }
}
