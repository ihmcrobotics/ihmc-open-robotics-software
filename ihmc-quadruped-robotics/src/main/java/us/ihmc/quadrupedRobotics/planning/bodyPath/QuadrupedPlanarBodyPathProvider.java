package us.ihmc.quadrupedRobotics.planning.bodyPath;

import us.ihmc.euclid.referenceFrame.FramePose2D;

public interface QuadrupedPlanarBodyPathProvider
{
   void initialize();

   /**
    * Packs the planar pose of the robot at the given time
    * @param time absolute time based on robot timestamp
    * @param poseToPack
    */
   void getPlanarPose(double time, FramePose2D poseToPack);
}
