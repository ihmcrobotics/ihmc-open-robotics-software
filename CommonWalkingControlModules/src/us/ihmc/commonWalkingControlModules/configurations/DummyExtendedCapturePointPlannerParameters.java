package us.ihmc.commonWalkingControlModules.configurations;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

public class DummyExtendedCapturePointPlannerParameters extends ExtendedCapturePointPlannerParameters
{   
   int numberOfCoPWayPointsPerFoot = 3;
   int numberOfFootstepsToConsider = 3;
   double finalTransferDuration = 0.1;
   Vector2D finalTransferCoPOffset = new Vector2D(0.1, 0.1); 
   List<Vector2D> leftCoPLocations = new ArrayList<>(numberOfCoPWayPointsPerFoot);
   List<Vector2D> rightCoPLocations = new ArrayList<>(numberOfCoPWayPointsPerFoot);
   List<Double> coPWayPointAlphas= new ArrayList<>(numberOfCoPWayPointsPerFoot-1);
   
   public DummyExtendedCapturePointPlannerParameters()
   {
      leftCoPLocations.add(new Vector2D(-0.05, 0.0));
      leftCoPLocations.add(new Vector2D(0.0, 0.0));
      leftCoPLocations.add(new Vector2D(0.06, 0.0));
      
      rightCoPLocations.add(new Vector2D(-0.05, 0.0));
      rightCoPLocations.add(new Vector2D(0.0, 0.0));
      rightCoPLocations.add(new Vector2D(0.06, 0.0));
      
      coPWayPointAlphas.add(0.5);
      coPWayPointAlphas.add(0.5);
   }
   
   @Override
   public List<Vector2D> getCoPWayPointLocationsFootFrame(RobotSide side)
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
   public Vector2D getFinalTransferCoPOffset()
   {
      return finalTransferCoPOffset;
   }

   @Override
   public double getDefaultFinalTransferDuration()
   {
      return finalTransferDuration;
   }
   
   @Override
   public int getNumberOfFootstepsToConsider()
   {
      return numberOfFootstepsToConsider;
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
   
   public void setNumberOfWaypointsPerFoot(int number)
   {
      numberOfCoPWayPointsPerFoot = number;
   }
   
   public void setNumberOfFootstepsToConsider(int number)
   {
      numberOfFootstepsToConsider = number;
   }
   
   public void setSymmetricCoPWayPointOffsets(List<Vector2D> coPOffsets, List<Double> coPAlphas)
   {
      rightCoPLocations = coPOffsets;
      leftCoPLocations = coPOffsets;
      coPWayPointAlphas = coPAlphas;
   }  
   
   public void setFinalTransferDuration(double finalTransferDuration)
   {
      this.finalTransferDuration = finalTransferDuration;
   }
   
   public double getDefaultStationaryTransferTime()
   {
      return getDefaultFinalTransferDuration();
   }
}
