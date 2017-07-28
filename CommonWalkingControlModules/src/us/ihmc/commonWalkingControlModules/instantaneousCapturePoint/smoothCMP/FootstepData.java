package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothCMP;

import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

/**
 * Object that stores points in the specified feet frame
 * @author Apoorv S
 */

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
   
   /**
    * Add a foot frame point to the list of points. 
    * Converts a point specified to the required reference frame if required
    * @param newPoint Point to be added
    */
   public void addUnsafeFootstepPoint(FramePoint2d newPoint)
   {
      if(newPoint == null)
      {
         return;
      }
      else 
      {
         if(newPoint.getReferenceFrame() != footstep.getSoleReferenceFrame())
            newPoint.changeFrame(footstep.getSoleReferenceFrame());
      }
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

   public FramePose getFramePose()
   {
      return footstep.getFootstepPose();
   }   
}
