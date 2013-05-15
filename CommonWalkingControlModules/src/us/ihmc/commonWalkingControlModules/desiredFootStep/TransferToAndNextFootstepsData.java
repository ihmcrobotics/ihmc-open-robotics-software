package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public class TransferToAndNextFootstepsData
{
   private FrameConvexPolygon2d transferToFootPolygonInSoleFrame;
   
   private Footstep transferFromFootstep;
   private Footstep transferToFootstep;
   private RobotSide transferToSide;
   
   private Footstep nextFootstep;
   private Footstep nextNextFootstep;

   private double w0;
   private double estimatedStepTime;
   
   private double singleSupportDuration;
   private double doubleSupportDuration;
   private double doubleSupportInitialTransferDuration;
   
   private boolean stopIfReachedEnd;
   
   public ContactablePlaneBody getTransferToFootContactablePlaneBody()
   {
      return transferToFootstep.getBody();
   }
   
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

   public void getFootLocationList(ArrayList<FramePoint> footLocationListToPack, double centimetersForwardFromCenter)
   {
      if (transferFromFootstep != null)
      {
         getFootLocationFromFootstepInWorldFrame(footLocationListToPack, transferFromFootstep, centimetersForwardFromCenter);
      }
      else return;
      
      if (transferToFootstep != null)
      {
         getFootLocationFromFootstepInWorldFrame(footLocationListToPack, transferToFootstep, centimetersForwardFromCenter);
      }
      else return;

      if (nextFootstep != null)
      {
         getFootLocationFromFootstepInWorldFrame(footLocationListToPack, nextFootstep, centimetersForwardFromCenter);
      }
      else return;

      if (nextNextFootstep != null)
      {
         getFootLocationFromFootstepInWorldFrame(footLocationListToPack, nextNextFootstep, centimetersForwardFromCenter);
      }
      else return;
   }

   private static void getFootLocationFromFootstepInWorldFrame(ArrayList<FramePoint> footLocationListToPack, Footstep footstep, double centimetersForwardFromCenter)
   {
      FramePoint centerOfFootstep = FootstepUtils.getCenterOfFootstep(footstep, centimetersForwardFromCenter);
      footLocationListToPack.add(centerOfFootstep);
   }

   public double getSingleSupportDuration()
   {
      return singleSupportDuration;
   }

   public double getDoubleSupportDuration()
   {
      return doubleSupportDuration;
   }

   public boolean getStopIfReachedEnd()
   {
      return stopIfReachedEnd;
   }

   public double getDoubleSupportInitialTransferDuration()
   {
      return doubleSupportInitialTransferDuration;
   }
   
   public void setSingleSupportDuration(double singleSupportDuration)
   {
      this.singleSupportDuration = singleSupportDuration;
   }

   public void setDoubleSupportDuration(double doubleSupportDuration)
   {
      this.doubleSupportDuration = doubleSupportDuration;
   }

   public void setStopIfReachedEnd(boolean stopIfReachedEnd)
   {
      this.stopIfReachedEnd = stopIfReachedEnd;
   }

   public void setDoubleSupportInitialTransferDuration(double doubleSupportInitialTransferDuration)
   {
      this.doubleSupportInitialTransferDuration = doubleSupportInitialTransferDuration;
   }
}
