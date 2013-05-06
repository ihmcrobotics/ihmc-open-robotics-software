package us.ihmc.commonWalkingControlModules.desiredFootStep;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;

public class TransferToAndNextFootstepsData
{
   private FrameConvexPolygon2d transferToFootPolygonInSoleFrame;
   
   private Footstep transferToFootstep;
   private RobotSide transferToSide;
   
   private Footstep nextFootstep;
   private Footstep nextNextFootstep;

   private double w0;
   private double estimatedStepTime;
   
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
   
   public double getW0()
   {
      return w0;
   }

   public void setW0(double w0)
   {
      this.w0 = w0;
   }

   public double getEstimatedStepTime()
   {
      return estimatedStepTime;
   }

   public void setEstimatedStepTime(double estimatedStepTime)
   {
      this.estimatedStepTime = estimatedStepTime;
   }
}
