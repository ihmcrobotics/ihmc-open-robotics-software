package us.ihmc.commonWalkingControlModules.desiredFootStep;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.humanoidRobot.footstep.Footstep;
import us.ihmc.robotics.humanoidRobot.footstep.FootstepUtils;

public class TransferToAndNextFootstepsData
{
   private FrameConvexPolygon2d transferToFootPolygonInSoleFrame;
   
   private Footstep transferFromFootstep;
   private Footstep transferFromDesiredFootstep;
   private Footstep transferToFootstep;
   private RobotSide transferToSide;
   
   private Footstep nextFootstep;
   private Footstep nextNextFootstep;
  
   private Point3d currentDesiredICP;
   private Vector3d currentDesiredICPVelocity;

   private double w0;
   private double estimatedStepTime;
   
   private double singleSupportDuration;
   private double doubleSupportDuration;
   private double doubleSupportInitialTransferDuration;
   
   private boolean stopIfReachedEnd;
   
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

   public void setCurrentDesiredICP(FramePoint2d currentICP, FrameVector2d currentICPVelocity) 
   {
      currentDesiredICP = new Point3d(currentICP.getX(), currentICP.getY(), 0.0);
      currentICP.getReferenceFrame().getTransformToDesiredFrame(ReferenceFrame.getWorldFrame()).transform(currentDesiredICP);
      
      currentDesiredICPVelocity = new Vector3d(currentICPVelocity.getX(), currentICPVelocity.getY(), 0.0);
      currentICPVelocity.getReferenceFrame().getTransformToDesiredFrame(ReferenceFrame.getWorldFrame()).transform(currentDesiredICPVelocity);
   }

   public Point3d getCurrentDesiredICP() 
   {
      return currentDesiredICP;
   }
   
   public Vector3d getCurrentDesiredICPVelocity() 
   {
      return currentDesiredICPVelocity;
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
      return  3.4; // w0; // NOTE: Attention, magic number!!!
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

   public void getFootLocationList(ArrayList<FramePoint> footLocationListToPack, ArrayList<ReferenceFrame> soleFrameListToPack, double centimetersForwardFromCenter, double centimetersInFromCenter)
   {
      if (transferFromFootstep != null)
      {
         getFootLocationFromFootstepInWorldFramePlusSoleframe(footLocationListToPack, soleFrameListToPack, transferFromFootstep, transferToSide.getOppositeSide(), centimetersForwardFromCenter, centimetersInFromCenter);
      }
      else return;
      
      if (transferToFootstep != null)
      {
         getFootLocationFromFootstepInWorldFramePlusSoleframe(footLocationListToPack, soleFrameListToPack, transferToFootstep, transferToSide, centimetersForwardFromCenter, centimetersInFromCenter);
      }
      else return;

      if (nextFootstep != null)
      {
         getFootLocationFromFootstepInWorldFramePlusSoleframe(footLocationListToPack, soleFrameListToPack, nextFootstep, transferToSide.getOppositeSide(), centimetersForwardFromCenter, centimetersInFromCenter);
      }
      else return;

      if (nextNextFootstep != null)
      {
         getFootLocationFromFootstepInWorldFramePlusSoleframe(footLocationListToPack, soleFrameListToPack, nextNextFootstep, transferToSide, centimetersForwardFromCenter, centimetersInFromCenter);
      }
      else return;
   }

   private static void getFootLocationFromFootstepInWorldFramePlusSoleframe(ArrayList<FramePoint> footLocationListToPack, 
         ArrayList<ReferenceFrame> soleFrameListToPack, Footstep footstep, RobotSide side, double centimetersForwardFromCenter, double centimetersInFromCenter)
   {
      FramePoint centerOfFootstep = FootstepUtils.getCenterOfPredictedContactPointsInFootstep(footstep, side, centimetersForwardFromCenter, centimetersInFromCenter);
      ReferenceFrame soleFrame = footstep.getSoleReferenceFrame(); 
      footLocationListToPack.add(centerOfFootstep);
      soleFrameListToPack.add(soleFrame);
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
