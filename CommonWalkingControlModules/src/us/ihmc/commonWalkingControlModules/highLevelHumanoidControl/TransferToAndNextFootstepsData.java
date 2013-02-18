package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePose;

public class TransferToAndNextFootstepsData
{
   private FrameConvexPolygon2d transferToFootPolygonInSoleFrame;
   
   private Footstep transferToFootstep;
   private RobotSide transferToSide;
   
   private Footstep nextFootstep;
   private Footstep nextNextFootstep;
   
   public ContactablePlaneBody getTransferToFootContactablePlaneBody()
   {
      return transferToFootstep.getBody();
   }
   
   public Footstep getTransferToFootstep()
   {
      return transferToFootstep;
   }
   
   public void setTransferToFootstep(Footstep transferToFootstep)
   {
      this.transferToFootstep = transferToFootstep;
   }
   
   public FrameConvexPolygon2d getTransferToFootPolygonInSoleFrame()
   {
      return transferToFootPolygonInSoleFrame;
   }
   
   public void setTransferToFootPolygonInSoleFrame(FrameConvexPolygon2d transferToFootPolygonInSoleFrame)
   {
      this.transferToFootPolygonInSoleFrame = transferToFootPolygonInSoleFrame;
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
   
   public Footstep getNextNextFootstep()
   {
      return nextNextFootstep;
   }
   
   public void setNextNextFootstep(Footstep nextNextFootstep)
   {
      this.nextNextFootstep = nextNextFootstep;
   }
   
}
