package us.ihmc.quadrupedRobotics.planning.bodyPath;

import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.referenceFrame.FramePose2D;

public interface QuadrupedPlanarBodyPathProvider
{
   /**
    * Packs the planar pose of the robot at the given time
    * @param time relative to the start of the trajectory
    * @param poseToPack
    */
   void getPlanarPose(double time, FramePose2D poseToPack);
}
