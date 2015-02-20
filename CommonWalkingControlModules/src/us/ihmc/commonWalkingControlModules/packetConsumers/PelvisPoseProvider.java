package us.ihmc.commonWalkingControlModules.packetConsumers;

import us.ihmc.utilities.math.geometry.FrameOrientationWaypoint;
import us.ihmc.utilities.math.geometry.FramePointWaypoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public abstract class PelvisPoseProvider
{
   public abstract boolean checkForNewPosition();

   public abstract boolean checkForNewOrientation();

   public abstract boolean checkForHomePosition();

   public abstract boolean checkForHomeOrientation();
   
   public abstract Double getTrajectoryTimeToHome();
      
   public abstract FramePointWaypoint[] getDesiredPelvisPosition(ReferenceFrame desiredReferenceFrame);

   public abstract FrameOrientationWaypoint[] getDesiredPelvisOrientation(ReferenceFrame desiredReferenceFrame);
   
   
   // to be used by those interpolators which expect only one desired position
   public FramePointWaypoint getFinalPelvisPosition(ReferenceFrame desiredReferenceFrame)
   {
      FramePointWaypoint[] waypoints = getDesiredPelvisPosition(desiredReferenceFrame);
      if(waypoints == null ) return null;
      
      int last = waypoints.length -1;
      for (int i=0; i< last; i++ ) waypoints[last].timeSincePreviousWaypoint += waypoints[i].timeSincePreviousWaypoint;
      return waypoints[last];
   }
   
   // to be used by those interpolators which expect only one desired position
   public FrameOrientationWaypoint getFinalPelvisOrientation(ReferenceFrame desiredReferenceFrame)
   {
      FrameOrientationWaypoint[] waypoints = getDesiredPelvisOrientation(desiredReferenceFrame);
      if(waypoints == null ) return null;
      
      int last = waypoints.length -1;
      for (int i=0; i< last; i++ ) waypoints[last].timeSincePreviousWaypoint += waypoints[i].timeSincePreviousWaypoint;
      return waypoints[last];
   }
   
}