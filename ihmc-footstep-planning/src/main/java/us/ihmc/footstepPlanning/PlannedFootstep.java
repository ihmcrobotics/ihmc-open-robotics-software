package us.ihmc.footstepPlanning;

import controller_msgs.msg.dds.FootstepDataMessage;
import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
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

   public PlannedFootstep(PlannedFootstep other)
   {
      this.sequenceId = other.sequenceId;
      this.robotSide = other.robotSide;
      this.footstepPose.set(other.footstepPose);
      this.foothold.set(other.foothold);

      this.trajectoryType = other.trajectoryType;
      this.swingHeight = other.swingHeight;

      for (int i = 0; i < other.customWaypointProportions.size(); i++)
      {
         customWaypointProportions.add(other.customWaypointProportions.get(i));
      }
      for (int i = 0; i < other.customWaypointPositions.size(); i++)
      {
         customWaypointPositions.add(new Point3D(other.customWaypointPositions.get(i)));
      }

      this.swingDuration = other.swingDuration;
      this.transferDuration = other.transferDuration;
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

   public TrajectoryType getTrajectoryType()
   {
      return trajectoryType;
   }

   public double getSwingHeight()
   {
      return swingHeight;
   }

   public TDoubleArrayList getCustomWaypointProportions()
   {
      return customWaypointProportions;
   }

   public List<Point3D> getCustomWaypointPositions()
   {
      return customWaypointPositions;
   }

   public double getSwingDuration()
   {
      return swingDuration;
   }

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

      plannedFootstep.setSwingDuration(footstepDataMessage.getSwingDuration());
      plannedFootstep.setTransferDuration(footstepDataMessage.getTransferDuration());

      return plannedFootstep;
   }
}
