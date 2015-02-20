package us.ihmc.commonWalkingControlModules.packetConsumers;

import us.ihmc.utilities.math.geometry.FrameOrientationWaypoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public abstract class ChestOrientationProvider 
{
   public abstract ReferenceFrame getChestOrientationExpressedInFrame();

   public abstract boolean checkForNewChestOrientation();
   
   public abstract boolean checkForHomeOrientation();

   public abstract FrameOrientationWaypoint[] getDesiredChestOrientations();
   
   // this method can be used by old trajectory generators that expect a single desiredChestOrientation.
   // if the number of waypoints is > 1, the last one is sent.
   // timeSincePreviousWaypoint will be the total time of the whole trajectory.
   public FrameOrientationWaypoint getFinalChestOrientation()
   {
      FrameOrientationWaypoint[] waypoints = getDesiredChestOrientations();
     
      if(waypoints == null) return null;
      
      int last = waypoints.length -1;
      for (int i=0; i<last; i++)
      {
         waypoints[last].timeSincePreviousWaypoint += waypoints[i].timeSincePreviousWaypoint;
      }
      return waypoints[last];
   }
}