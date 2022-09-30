package us.ihmc.footstepPlanning;

import controller_msgs.msg.dds.FootstepDataMessage;
import ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage;
import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;

import java.util.ArrayList;
import java.util.List;
import java.util.UUID;

/**
 * Object holding all information planned for a given step by {@link FootstepPlanningModule}
 */
public class PlannedFootstep implements PlannedFootstepReadOnly
{
   private final long sequenceId;
   private final RobotSide robotSide;
   private final FramePose3D footstepPose = new FramePose3D();
   private final ConvexPolygon2D foothold = new ConvexPolygon2D();

   private TrajectoryType trajectoryType = null;
   private double swingHeight = -1.0;
   private final TDoubleArrayList customWaypointProportions = new TDoubleArrayList();
   private final List<Point3D> customWaypointPositions = new ArrayList<>();
   private final List<FrameSE3TrajectoryPoint> swingTrajectory = new ArrayList<>();

   private double swingDuration = -1.0;
   private double transferDuration = -1.0;

   public PlannedFootstep(RobotSide robotSide)
   {
      this(robotSide, null, null);
   }

   public PlannedFootstep(RobotSide robotSide, Pose3DReadOnly footstepPose)
   {
      this(robotSide, footstepPose, null);
   }

   public PlannedFootstep(RobotSide robotSide, Pose3DReadOnly footstepPose, ConvexPolygon2D foothold)
   {
      this.sequenceId = (UUID.randomUUID().getLeastSignificantBits() % Integer.MAX_VALUE) + Integer.MAX_VALUE;
      this.robotSide = robotSide;

      if (footstepPose != null)
      {
         this.footstepPose.set(footstepPose);
      }
      if (foothold != null)
      {
         this.foothold.set(foothold);
      }
   }

   public PlannedFootstep(PlannedFootstepReadOnly other)
   {
      this.sequenceId = other.getSequenceId();
      this.robotSide = other.getRobotSide();
      other.getFootstepPose(this.footstepPose);
      this.foothold.set(other.getFoothold());

      this.trajectoryType = other.getTrajectoryType();
      this.swingHeight = other.getSwingHeight();

      for (int i = 0; i < other.getCustomWaypointProportions().size(); i++)
      {
         customWaypointProportions.add(other.getCustomWaypointProportions().get(i));
      }
      for (int i = 0; i < other.getCustomWaypointPositions().size(); i++)
      {
         customWaypointPositions.add(new Point3D(other.getCustomWaypointPositions().get(i)));
      }

      this.swingDuration = other.getSwingDuration();
      this.transferDuration = other.getTransferDuration();
   }

   @Override
   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public FramePose3D getFootstepPose()
   {
      return footstepPose;
   }

   @Override
   public void getFootstepPose(FramePose3D footstepPoseToPack)
   {
      footstepPoseToPack.set(footstepPose);
   }

   @Override
   public ConvexPolygon2D getFoothold()
   {
      return foothold;
   }

   @Override
   public boolean hasFoothold()
   {
      return !foothold.isEmpty();
   }

   @Override
   public TrajectoryType getTrajectoryType()
   {
      return trajectoryType;
   }

   @Override
   public double getSwingHeight()
   {
      return swingHeight;
   }

   @Override
   public TDoubleArrayList getCustomWaypointProportions()
   {
      return customWaypointProportions;
   }

   @Override
   public List<Point3D> getCustomWaypointPositions()
   {
      return customWaypointPositions;
   }

   @Override
   public double getSwingDuration()
   {
      return swingDuration;
   }

   @Override
   public double getTransferDuration()
   {
      return transferDuration;
   }

   public void setTrajectoryType(TrajectoryType trajectoryType)
   {
      this.trajectoryType = trajectoryType;
   }

   public void setSwingHeight(double swingHeight)
   {
      this.swingHeight = swingHeight;
   }

   public void setCustomWaypointProportions(double proportion0, double proportion1)
   {
      this.customWaypointProportions.clear();
      this.customWaypointProportions.add(proportion0);
      this.customWaypointProportions.add(proportion1);
   }

   public void setCustomWaypointPositions(Point3D position0, Point3D position1)
   {
      this.customWaypointPositions.clear();
      this.customWaypointPositions.add(position0);
      this.customWaypointPositions.add(position1);
   }

   public void setSwingDuration(double swingDuration)
   {
      this.swingDuration = swingDuration;
   }

   public void setTransferDuration(double transferDuration)
   {
      this.transferDuration = transferDuration;
   }

   public void limitFootholdVertices()
   {
      if (!foothold.isEmpty() && foothold.getNumberOfVertices() != 4)
      {
         ConvexPolygonTools.limitVerticesConservative(foothold, 4);
      }
   }

   @Override
   public List<FrameSE3TrajectoryPoint> getSwingTrajectory()
   {
      return swingTrajectory;
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }

   public FootstepDataMessage getAsMessage()
   {
      FootstepDataMessage footstepDataMessage = new FootstepDataMessage();
      footstepDataMessage.setSequenceId(sequenceId);
      footstepDataMessage.setRobotSide(robotSide.toByte());
      footstepDataMessage.getLocation().set(footstepPose.getPosition());
      footstepDataMessage.getOrientation().set(footstepPose.getOrientation());

      for (int i = 0; i < foothold.getNumberOfVertices(); i++)
      {
         footstepDataMessage.getPredictedContactPoints2d().add().set(foothold.getVertex(i), 0.0);
      }

      if (trajectoryType != null)
      {
         footstepDataMessage.setTrajectoryType(trajectoryType.toByte());
      }

      footstepDataMessage.setSwingHeight(swingHeight);

      for (int i = 0; i < customWaypointProportions.size(); i++)
      {
         footstepDataMessage.getCustomWaypointProportions().add(customWaypointProportions.get(i));
      }
      for (int i = 0; i < customWaypointPositions.size(); i++)
      {
         footstepDataMessage.getCustomPositionWaypoints().add().set(customWaypointPositions.get(i));
      }

      for (int i = 0; i < swingTrajectory.size(); i++)
      {
         SE3TrajectoryPointMessage swingTrajectoryPointToSet = footstepDataMessage.getSwingTrajectory().add();
         swingTrajectoryPointToSet.setTime(swingTrajectory.get(i).getTime());
         swingTrajectoryPointToSet.getPosition().set(swingTrajectory.get(i).getPosition());
         swingTrajectoryPointToSet.getOrientation().set(swingTrajectory.get(i).getOrientation());
         swingTrajectoryPointToSet.getLinearVelocity().set(swingTrajectory.get(i).getLinearVelocity());
         swingTrajectoryPointToSet.getAngularVelocity().set(swingTrajectory.get(i).getAngularVelocity());
      }

      footstepDataMessage.setSwingDuration(swingDuration);
      footstepDataMessage.setTransferDuration(transferDuration);

      return footstepDataMessage;
   }

   public static PlannedFootstep getFromMessage(FootstepDataMessage footstepDataMessage)
   {
      RobotSide robotSide = RobotSide.fromByte(footstepDataMessage.getRobotSide());
      FramePose3D footstepPose = new FramePose3D(ReferenceFrame.getWorldFrame(), footstepDataMessage.getLocation(), footstepDataMessage.getOrientation());

      PlannedFootstep plannedFootstep = new PlannedFootstep(robotSide, footstepPose);

      footstepDataMessage.getPredictedContactPoints2d().forEach(plannedFootstep.getFoothold()::addVertex);
      plannedFootstep.getFoothold().update();

      plannedFootstep.setTrajectoryType(TrajectoryType.fromByte(footstepDataMessage.getTrajectoryType()));
      plannedFootstep.setSwingHeight(footstepDataMessage.getSwingHeight());

      if (footstepDataMessage.getCustomWaypointProportions().size() == 2)
      {
         double proportion0 = footstepDataMessage.getCustomWaypointProportions().get(0);
         double proportion1 = footstepDataMessage.getCustomWaypointProportions().get(1);
         plannedFootstep.setCustomWaypointProportions(proportion0, proportion1);
      }
      if (footstepDataMessage.getCustomPositionWaypoints().size() == 2)
      {
         Point3D waypoint0 = footstepDataMessage.getCustomPositionWaypoints().get(0);
         Point3D waypoint1 = footstepDataMessage.getCustomPositionWaypoints().get(1);
         plannedFootstep.setCustomWaypointPositions(waypoint0, waypoint1);
      }
      for (int i = 0; i < footstepDataMessage.getSwingTrajectory().size(); i++)
      {
         SE3TrajectoryPointMessage trajectoryPoint = footstepDataMessage.getSwingTrajectory().get(i);
         FrameSE3TrajectoryPoint trajectoryPointToSet = new FrameSE3TrajectoryPoint();
         trajectoryPointToSet.set(trajectoryPoint.getTime(),
                                  trajectoryPoint.getPosition(),
                                  trajectoryPoint.getOrientation(),
                                  trajectoryPoint.getLinearVelocity(),
                                  trajectoryPoint.getAngularVelocity());
         plannedFootstep.getSwingTrajectory().add(trajectoryPointToSet);
      }

      plannedFootstep.setSwingDuration(footstepDataMessage.getSwingDuration());
      plannedFootstep.setTransferDuration(footstepDataMessage.getTransferDuration());

      return plannedFootstep;
   }
}
