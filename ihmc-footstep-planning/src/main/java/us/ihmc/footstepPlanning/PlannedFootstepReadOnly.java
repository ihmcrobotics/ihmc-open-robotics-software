package us.ihmc.footstepPlanning;

import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.robotics.robotSide.RobotSide;

public interface PlannedFootstepReadOnly
{
   RobotSide getRobotSide();

   void getFootstepPose(FramePose3D footstepPoseToPack);

   ConvexPolygon2DReadOnly getFoothold();

   boolean hasFoothold();
}
