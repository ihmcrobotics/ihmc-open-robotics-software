package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.robotSide.RobotSide;

public class NewTransferToAndNextFootstepsData
{
   private final FramePoint3D transferFromPosition = new FramePoint3D();
   private final FramePoint3D transferToPosition = new FramePoint3D();
   private RobotSide transferToSide;

   private final FramePoint3D nextFootstepPosition = new FramePoint3D();

   public NewTransferToAndNextFootstepsData()
   {
      nextFootstepPosition.setToNaN();
   }

   public FramePoint3DReadOnly getTransferToPosition()
   {
      return transferToPosition;
   }

   public FramePoint3DReadOnly getTransferFromPosition()
   {
      return transferFromPosition;
   }

   public void setTransferFromPosition(FramePoint3DReadOnly transferFromPosition)
   {
      this.transferFromPosition.set(transferFromPosition);
   }

   public void setTransferToPosition(FramePoint3DReadOnly transferToPosition)
   {
      this.transferToPosition.set(transferToPosition);
   }

   public void setTransferFromPosition(ReferenceFrame referenceFrame)
   {
      transferFromPosition.setToZero(referenceFrame);
      transferFromPosition.changeFrame(ReferenceFrame.getWorldFrame());
   }

   public void setTransferToPosition(ReferenceFrame referenceFrame)
   {
      transferToPosition.setToZero(referenceFrame);
      transferToPosition.changeFrame(ReferenceFrame.getWorldFrame());
   }

   public RobotSide getTransferToSide()
   {
      return transferToSide;
   }

   public void setTransferToSide(RobotSide transferToSide)
   {
      this.transferToSide = transferToSide;
   }

   public FramePoint3DReadOnly getNextFootstepPosition()
   {
      return nextFootstepPosition;
   }

   public void setNextFootstepPosition(FramePoint3DReadOnly nextFootstepPosition)
   {
      if (nextFootstepPosition == null)
      {
         this.nextFootstepPosition.setToNaN();
         return;
      }

      this.nextFootstepPosition.set(nextFootstepPosition);
   }
}
