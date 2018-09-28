package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.math.trajectories.waypoints.YoFrameEuclideanTrajectoryPoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

/**
 * Adds some visualization methods to the YoFrameEuclideanTrajectoryPoint class 
 */
public class CoPTrajectoryPoint extends YoFrameEuclideanTrajectoryPoint
{
   public CoPTrajectoryPoint(String namePrefix, String nameSuffix, YoVariableRegistry registry, ReferenceFrame[] referenceFrames)
   {
      super(namePrefix, nameSuffix, registry, referenceFrames);
   }

   public void setIncludingFrame(CoPTrajectoryPoint other)
   {
      registerReferenceFrame(other.getReferenceFrame());
      switchCurrentReferenceFrame(other.getReferenceFrame());
      set(other.getTime(), other.getPosition(), other.getLinearVelocity());
      putYoValuesIntoFrameWaypoint();
   }
}
