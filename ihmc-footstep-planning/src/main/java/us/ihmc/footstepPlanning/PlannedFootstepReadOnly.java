package us.ihmc.footstepPlanning;

import controller_msgs.msg.dds.FootstepDataMessage;
import gnu.trove.list.array.TDoubleArrayList;
import ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintListConverter;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintMessageConverter;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintRegion;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;

import java.util.Arrays;
import java.util.List;

public interface PlannedFootstepReadOnly
{
   RobotSide getRobotSide();

   void getFootstepPose(FramePose3DBasics footstepPoseToPack);

   FramePose3DReadOnly getFootstepPose();

   ConvexPolygon2DReadOnly getFoothold();

   boolean hasFoothold();

   TrajectoryType getTrajectoryType();

   double getSwingHeight();

   TDoubleArrayList getCustomWaypointProportions();

   List<Point3D> getCustomWaypointPositions();

   double getSwingDuration();

   double getTransferDuration();

   List<FrameSE3TrajectoryPoint> getSwingTrajectory();

   PlanarRegion getRegionSnappedTo();

   long getSequenceId();

   default FootstepDataMessage getAsMessage()
   {
      FootstepDataMessage footstepDataMessage = new FootstepDataMessage();
      footstepDataMessage.setSequenceId(getSequenceId());
      footstepDataMessage.setRobotSide(getRobotSide().toByte());
      footstepDataMessage.getLocation().set(getFootstepPose().getPosition());
      footstepDataMessage.getOrientation().set(getFootstepPose().getOrientation());

      for (int i = 0; i < getFoothold().getNumberOfVertices(); i++)
      {
         footstepDataMessage.getPredictedContactPoints2d().add().set(getFoothold().getVertex(i), 0.0);
      }

      if (getTrajectoryType() != null)
      {
         footstepDataMessage.setTrajectoryType(getTrajectoryType().toByte());
      }

      footstepDataMessage.setSwingHeight(getSwingHeight());

      for (int i = 0; i < getCustomWaypointProportions().size(); i++)
      {
         footstepDataMessage.getCustomWaypointProportions().add(getCustomWaypointProportions().get(i));
      }
      for (int i = 0; i < getCustomWaypointPositions().size(); i++)
      {
         footstepDataMessage.getCustomPositionWaypoints().add().set(getCustomWaypointPositions().get(i));
      }

      for (int i = 0; i < getSwingTrajectory().size(); i++)
      {
         SE3TrajectoryPointMessage swingTrajectoryPointToSet = footstepDataMessage.getSwingTrajectory().add();
         swingTrajectoryPointToSet.setTime(getSwingTrajectory().get(i).getTime());
         swingTrajectoryPointToSet.getPosition().set(getSwingTrajectory().get(i).getPosition());
         swingTrajectoryPointToSet.getOrientation().set(getSwingTrajectory().get(i).getOrientation());
         swingTrajectoryPointToSet.getLinearVelocity().set(getSwingTrajectory().get(i).getLinearVelocity());
         swingTrajectoryPointToSet.getAngularVelocity().set(getSwingTrajectory().get(i).getAngularVelocity());
      }

      footstepDataMessage.setSwingDuration(getSwingDuration());
      footstepDataMessage.setTransferDuration(getTransferDuration());

      if (getRegionSnappedTo() != null)
      {
         footstepDataMessage.getStepConstraints().set(StepConstraintMessageConverter.convertToStepConstraintsListMessageFromPlanarRegions(Arrays.asList(getRegionSnappedTo())));
      }

      return footstepDataMessage;
   }
}
