package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

/**
 * Object that stores points in the specified feet frame
 * @author Apoorv S
 */

public class FootstepTrajectoryPoint
{
   private Footstep footstep;   
   private FootstepTiming timing;
   private ReferenceFrame footFrame;
      
   /**
    * Initialize the foot point storage object
    * @param side Specify foot side
    * @param refFrame Provide corresponding foot reference frame. This will be stored and accessed to get updated values
    */
   
   public FootstepTrajectoryPoint(Footstep footstep, FootstepTiming timing)
   {
      this(footstep, timing, 0);     
   }
   
   /**
    * Initialize the foot point storage object
    * @param side Specify foot side
    * @param refFrame Provide corresponding foot reference frame. This will be stored and accessed to get updated values
    * @param estimatedSize Initial estimate of the number of points to be stored
    */
   
   public FootstepTrajectoryPoint(Footstep footstep, FootstepTiming timing, int estimatedSize)
   {
      this.footstep = footstep;
      this.timing = timing;      
      this.footFrame = footstep.getSoleReferenceFrame();
   }
   
   public void addFootstepPoint(FramePoint2d newPoint)
   {
      //TODO 
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
         if(newPoint.getReferenceFrame() != footFrame)
            newPoint.changeFrame(footFrame);
      }
   }

   public FramePoint getCentroidOfExpectedFootPolygonLocation()
   {
      return footstep.getFootstepPose().getFramePointCopy();
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
}
