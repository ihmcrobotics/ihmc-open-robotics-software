package us.ihmc.footstepPlanning;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;

import java.util.List;

public interface PlannedFootstepReadOnly
{
   RobotSide getRobotSide();

   void getFootstepPose(FramePose3D footstepPoseToPack);

   ConvexPolygon2DReadOnly getFoothold();

   boolean hasFoothold();

   TrajectoryType getTrajectoryType();

   double getSwingHeight();

   TDoubleArrayList getCustomWaypointProportions();

   List<Point3D> getCustomWaypointPositions();

   double getSwingDuration();

   double getTransferDuration();

   List<FrameSE3TrajectoryPoint> getSwingTrajectory();

   long getSequenceId();
}
