package us.ihmc.commonWalkingControlModules.configurations;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class DummyExtendedCapturePointPlannerParameters extends ExtendedCapturePointPlannerParameters
{   
   ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   int numberOfCoPWayPointsPerFoot = 3;
   FrameVector2d finalTransferCoPOffset = new FrameVector2d(worldFrame, 0.1, 0.1); 
   List<FrameVector2d> leftCoPLocations = new ArrayList<>(numberOfCoPWayPointsPerFoot);
   List<FrameVector2d> rightCoPLocations = new ArrayList<>(numberOfCoPWayPointsPerFoot);
   List<Double> coPWayPointAlphas= new ArrayList<>(numberOfCoPWayPointsPerFoot);
   
   public DummyExtendedCapturePointPlannerParameters()
   {
      
   }
   
   @Override
   public List<FrameVector2d> getCoPWayPointLocationsFootFrame(RobotSide side)
   {      
      if(side == RobotSide.LEFT)         
         return leftCoPLocations;
      else
         return rightCoPLocations;
   }

   @Override
   public List<Double> getCoPWayPointAlpha(RobotSide side)
   {
      return coPWayPointAlphas;
   }

   @Override
   public FrameVector2d getFinalTransferCoPOffset()
   {
      return finalTransferCoPOffset;
   }

   @Override
   public double getDefaultFinalTransferDuration()
   {
      return 0.2;
   }
   
   @Override
   public int getNumberOfFootstepsToConsider()
   {
      return 3;
   }
   
   @Override
   public int getNumberOfWayPointsPerFoot()
   {
      return numberOfCoPWayPointsPerFoot;
   }
   
   @Override
   public CoPSplineType getOrderOfCoPInterpolation()
   {
      return CoPSplineType.LINEAR;
   }
}
