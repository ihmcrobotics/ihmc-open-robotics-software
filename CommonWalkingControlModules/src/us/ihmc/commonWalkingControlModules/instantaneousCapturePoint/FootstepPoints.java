package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

/**
 * Object that stores points in the specified feet frame
 * @author Apoorv S
 */

public class FootstepPoints
{
   private ReferenceFrame footFrame;
   private List<FramePoint2d> footstepPoints;
   private RobotSide side;
   
   /**
    * Initialize the foot point storage object
    * @param side Specify foot side
    * @param refFrame Provide corresponding foot reference frame. This will be stored and accessed to get updated values
    */
   
   public FootstepPoints(RobotSide side, ReferenceFrame refFrame)
   {
      this(side, refFrame, 0);
   }
   
   /**
    * Initialize the foot point storage object
    * @param side Specify foot side
    * @param refFrame Provide corresponding foot reference frame. This will be stored and accessed to get updated values
    * @param estimatedSize Initial estimate of the number of points to be stored
    */
   
   public FootstepPoints(RobotSide side, ReferenceFrame refFrame, int estimatedSize)
   {
      this.side = side;
      this.footFrame = refFrame;
      this.footstepPoints = new ArrayList<>(estimatedSize);
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
         
         footstepPoints.add(newPoint);
      }
   }
      
   public void clearFootstepPoints()
   {
      footstepPoints.clear();
   }
   
   public int getNumberOfPoints()
   {
      return footstepPoints.size();      
   }
   
   public FramePoint2d getFootStepPointInFeetFrame(int index)
   {
      return footstepPoints.get(index);
   }   
}
