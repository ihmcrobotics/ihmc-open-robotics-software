package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.robotSide.RobotSide;

public class FootstepData
{
   public Footstep footstep;
   public FootstepTiming timing;

   /**
    * Initialize the foot point storage object
    */
   public FootstepData(Footstep footstep, FootstepTiming timing)
   {
      this.footstep = footstep;
      this.timing = timing;      
   }

   public FootstepData()
   {}

   public void setFootstep(Footstep footstep)
   {
      this.footstep = footstep;
   }

   public void setTiming(FootstepTiming timing)
   {
      this.timing = timing;
   }

   public void set(FootstepData other)
   {
      this.footstep = other.footstep;
      this.timing = other.timing;
   }

   public void set(Footstep footstep, FootstepTiming timing)
   {
      this.footstep = footstep;
      this.timing = timing;
   }

   public void setSwingTime(double swingTime)
   {
      this.timing.setTimings(swingTime, timing.getTransferTime());
   }
   
   public Footstep getFootstep()
   {
      return footstep;
   }

   public ReferenceFrame getFootstepPoseReferenceFrame()
   {
      return footstep.getFootstepPose().getReferenceFrame();
   }
   
   public RobotSide getSwingSide()
   {
      return footstep.getRobotSide();
   }
   
   public RobotSide getSupportSide()
   {
      return footstep.getRobotSide().getOppositeSide();
   }
   
   public double getTransferStartTime()
   {
      return timing.getExecutionStartTime();
   }
   
   public double getSwingStartTime()
   {
      return timing.getSwingStartTime();
   }
   
   public double getSwingTime()
   {
      return timing.getSwingTime();
   }
   
   public double getTransferTime()
   {
      return timing.getTransferTime();
   }
   
   public double getStepTime()
   {
      return timing.getStepTime();
   }

   public FramePose3D getFramePose()
   {
      return footstep.getFootstepPose();
   }   
}
