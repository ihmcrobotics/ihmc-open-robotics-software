package us.ihmc.commonWalkingControlModules.configurations;

import java.util.List;

import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.robotSide.RobotSide;

public abstract class ExtendedCapturePointPlannerParameters
{  
   /**
    * Provides the CoP way point list for generating CoP trajectories
    * @param side
    * @return
    */
   public abstract List<FrameVector2d> getCoPWayPointLocationsFootFrame(RobotSide side);
   /**
    * Provides a list of the alphas that denote the percentage of time taken to transition from one CoP way point to another.
    * Summation of the list must be equal to 1. 
    * @param side
    * @return
    */
   public abstract List<Double> getCoPWayPointAlpha(RobotSide side);
   public abstract FrameVector2d getFinalTransferCoPOffset();
   public abstract double getDefaultFinalTransferDuration();
   
   public int getNumberOfFootstepsToConsider()
   {
      return 3;
   }
   
   public int getNumberOfWayPointsPerFoot()
   {
      return 2;
   }
   
   public CoPSplineType getOrderOfCoPInterpolation()
   {
      return CoPSplineType.CUBIC;
   }
}
