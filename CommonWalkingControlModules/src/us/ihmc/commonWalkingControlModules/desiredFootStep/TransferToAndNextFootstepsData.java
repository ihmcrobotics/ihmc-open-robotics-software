package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.robotSide.RobotSide;

public class TransferToAndNextFootstepsData
{
   private Footstep transferFromFootstep;
   private Footstep transferFromDesiredFootstep;
   private Footstep transferToFootstep;
   private RobotSide transferToSide;

   private Footstep nextFootstep;

   public Footstep getTransferToFootstep()
   {
      return transferToFootstep;
   }

   public Footstep getTransferFromFootstep()
   {
      return transferFromFootstep;
   }

   public void setTransferFromFootstep(Footstep transferFromFootstep)
   {
      this.transferFromFootstep = transferFromFootstep;
   }

   public void setTransferToFootstep(Footstep transferToFootstep)
   {
      this.transferToFootstep = transferToFootstep;
   }

   public Footstep getTransferFromDesiredFootstep()
   {
      return transferFromDesiredFootstep;
   }

   public void setTransferFromDesiredFootstep(Footstep previousDesiredFootstep)
   {
      this.transferFromDesiredFootstep = previousDesiredFootstep;
   }

   public RobotSide getTransferToSide()
   {
      return transferToSide;
   }

   public void setTransferToSide(RobotSide transferToSide)
   {
      this.transferToSide = transferToSide;
   }

   public Footstep getNextFootstep()
   {
      return nextFootstep;
   }

   public void setNextFootstep(Footstep nextFootstep)
   {
      this.nextFootstep = nextFootstep;
   }
}
