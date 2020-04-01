package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.robotSide.RobotSide;

public class NewTransferToAndNextFootstepsData
{
   private final FramePoint3D transferToPosition = new FramePoint3D();
   private final FramePoint3D comAtEndOfState = new FramePoint3D();
   private RobotSide transferToSide;

   public FramePoint3DReadOnly getTransferToPosition()
   {
      return transferToPosition;
   }

   public void setComAtEndOfState(FramePoint3DReadOnly comAtEndOfState)
   {
      this.comAtEndOfState.set(comAtEndOfState);
   }

   public FramePoint3DReadOnly getCoMAtEndOfState()
   {
      return comAtEndOfState;
   }

   public void setTransferToPosition(FramePoint3DReadOnly transferToPosition)
   {
      this.transferToPosition.set(transferToPosition);
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
}


