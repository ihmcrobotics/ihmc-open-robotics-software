package us.ihmc.commonWalkingControlModules.controllerCore.command;

import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;

public class ControllerCoreOutput implements ControllerCoreOutputReadOnly
{
   private final CenterOfPressureDataHolder centerOfPressureDataHolder;
   private final FrameVector linearMomentumRate = new FrameVector(ReferenceFrame.getWorldFrame());

   public ControllerCoreOutput(CenterOfPressureDataHolder centerOfPressureDataHolder)
   {
      this.centerOfPressureDataHolder = centerOfPressureDataHolder;
      linearMomentumRate.setToNaN(ReferenceFrame.getWorldFrame());
   }

   public void setDesiredCenterOfPressure(FramePoint2d cop, RigidBody rigidBody)
   {
      centerOfPressureDataHolder.setCenterOfPressure(cop, rigidBody);
   }

   @Override
   public void getDesiredCenterOfPressure(FramePoint2d copToPack, RigidBody rigidBody)
   {
      centerOfPressureDataHolder.getCenterOfPressure(copToPack, rigidBody);
   }

   public void setLinearMomentumRate(FrameVector linearMomentumRate)
   {
      this.linearMomentumRate.set(linearMomentumRate);
   }

   public void setAndMatchFrameLinearMomentumRate(FrameVector linearMomentumRate)
   {
      this.linearMomentumRate.setIncludingFrame(linearMomentumRate);
      this.linearMomentumRate.changeFrame(ReferenceFrame.getWorldFrame());
   }

   @Override
   public void getLinearMomentumRate(FrameVector linearMomentumRateToPack)
   {
      linearMomentumRateToPack.setIncludingFrame(linearMomentumRate);
   }
}
