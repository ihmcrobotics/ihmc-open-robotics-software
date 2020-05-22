package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.robotSide.RobotSide;

public class NewTransferToAndNextFootstepsData
{
   private final FramePoint3D transferToPosition = new FramePoint3D();
   private final FramePoint2D comAtEndOfState = new FramePoint2D();
   private RobotSide transferToSide;

   public NewTransferToAndNextFootstepsData()
   {
      comAtEndOfState.setToNaN();
   }

   public FramePoint3DReadOnly getTransferToPosition()
   {
      return transferToPosition;
   }

   public void setComAtEndOfState(FramePoint3DReadOnly comAtEndOfState)
   {
      this.comAtEndOfState.set(comAtEndOfState);
   }

   public FramePoint2DReadOnly getCoMAtEndOfState()
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


