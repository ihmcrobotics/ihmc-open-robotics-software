package us.ihmc.commonWalkingControlModules.configurations;

import java.util.List;

import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.robotics.robotSide.RobotSide;

public abstract class CenterOfPressurePlannerParameters
{  
   /**
    * Provides the CoP way point list for generating CoP trajectories
    * @param side
    * @return
    */
   public abstract List<Vector2D> getCoPWayPointLocationsFootFrame(RobotSide side);
   /**
    * Provides a list of the alphas that denote the percentage of time taken to transition from one CoP way point to another.
    * Summation of the list must be equal to 1. 
    * @param side
    * @return
    */
   public abstract List<Double> getCoPWayPointAlpha(RobotSide side);   
   public abstract Vector2D getFinalTransferCoPOffset();
   public abstract double getDefaultFinalTransferDuration();
   public abstract int getNumberOfFootstepsToConsider();
   public abstract int getNumberOfWayPointsPerFoot();
   public abstract CoPSplineType getOrderOfCoPInterpolation();   
   public abstract double getDefaultStationaryTransferTime();
}
