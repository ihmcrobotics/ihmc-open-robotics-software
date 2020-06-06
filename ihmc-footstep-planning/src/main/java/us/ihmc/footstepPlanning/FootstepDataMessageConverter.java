package us.ihmc.footstepPlanning;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.SE3TrajectoryPointMessage;
import org.apache.commons.lang3.tuple.MutablePair;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;

import java.util.ArrayList;
import java.util.List;

public class FootstepDataMessageConverter
{
   public static FootstepDataListMessage createFootstepDataListFromPlan(FootstepPlan footstepPlan,
                                                                        double defaultSwingTime,
                                                                        double defaultTransferTime)
   {
      if (footstepPlan == null)
      {
         return null;
      }

      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      footstepDataListMessage.setDefaultSwingDuration(defaultSwingTime);
      footstepDataListMessage.setDefaultTransferDuration(defaultTransferTime);
      footstepDataListMessage.setOffsetFootstepsHeightWithExecutionError(true);

      appendPlanToMessage(footstepPlan, footstepDataListMessage);

      return footstepDataListMessage;
   }

   public static void appendPlanToMessage(FootstepPlan footstepPlan, FootstepDataListMessage footstepDataListMessage)
   {
      for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)
      {
         Footstep footstep = footstepPlan.getFootstep(i);
         FootstepDataMessage footstepData = HumanoidMessageTools.createFootstepDataMessage(footstep);

         if (footstep.hasPredictedContactPoints())
         {
            ConvexPolygon2D foothold = new ConvexPolygon2D();
            footstep.getPredictedContactPoints().forEach(foothold::addVertex);
            foothold.update();

            if (foothold.getNumberOfVertices() != 4)
            {
               ConvexPolygonTools.limitVerticesConservative(foothold, 4);
            }

            footstepData.getPredictedContactPoints2d().clear();
            for (int j = 0; j < foothold.getNumberOfVertices(); j++)
            {
               footstepData.getPredictedContactPoints2d().add().set(foothold.getVertex(j));
            }
         }

         footstepDataListMessage.getFootstepDataList().add().set(footstepData);
      }
   }

   public static FootstepPlan convertToFootstepPlan(FootstepDataListMessage footstepDataListMessage)
   {
      FootstepPlan footstepPlan = new FootstepPlan();

      for (FootstepDataMessage footstepMessage : footstepDataListMessage.getFootstepDataList())
      {
         Footstep footstep = new Footstep();
         setFootstepFromMessage(footstep, footstepMessage);
         footstepPlan.addFootstep(footstep);
      }

      return footstepPlan;
   }

   public static void setFootstepFromMessage(Footstep footstep, FootstepDataMessage footstepDataMessage)
   {
      footstep.setRobotSide(RobotSide.fromByte(footstepDataMessage.getRobotSide()));
      footstep.getFootstepPose().set(footstepDataMessage.getLocation(), footstepDataMessage.getOrientation());

      List<Point2D> predictedContactPointsToPack = new ArrayList<>();
      Object<Point3D> predictedContactPoints = footstepDataMessage.getPredictedContactPoints2d();
      for (int i = 0; i < predictedContactPoints.size(); i++)
      {
         predictedContactPointsToPack.add(new Point2D(predictedContactPoints.get(i)));
      }
      footstep.setPredictedContactPoints(predictedContactPointsToPack);

      footstep.setTrajectoryType(TrajectoryType.fromByte(footstepDataMessage.getTrajectoryType()));
      footstep.setSwingHeight(footstepDataMessage.getSwingHeight());
      footstep.setSwingTrajectoryBlendDuration(footstepDataMessage.getSwingTrajectoryBlendDuration());

      if (footstepDataMessage.getCustomPositionWaypoints().size() == 2)
      {
         footstep.setCustomPositionWaypoints(footstepDataMessage.getCustomPositionWaypoints());
      }
      else
      {
         footstep.getCustomPositionWaypoints().clear();
      }

      if (footstepDataMessage.getCustomWaypointProportions().size() == 2)
      {
         footstep.setCustomWaypointProportions(i -> footstepDataMessage.getCustomWaypointProportions().get(i));
      }
      else
      {
         footstep.getCustomWaypointProportions().clear();
      }

      footstep.getSwingTrajectory().clear();
      for (int i = 0; i < footstepDataMessage.getSwingTrajectory().size(); i++)
      {
         FrameSE3TrajectoryPoint trajectoryPointToSet = footstep.getSwingTrajectory().add();
         SE3TrajectoryPointMessage trajectoryPoint = footstepDataMessage.getSwingTrajectory().get(i);

         trajectoryPointToSet.setPosition(trajectoryPoint.getPosition());
         trajectoryPointToSet.setOrientation(trajectoryPoint.getOrientation());
         trajectoryPointToSet.setLinearVelocity(trajectoryPoint.getLinearVelocity());
         trajectoryPointToSet.setAngularVelocity(trajectoryPoint.getAngularVelocity());
         trajectoryPointToSet.setTime(trajectoryPoint.getTime());
      }
   }

   public static ArrayList<Pair<RobotSide, Pose3D>> reduceFootstepPlanForUIMessager(FootstepDataListMessage footstepDataListMessage)
   {
      return reduceFootstepPlanForUIMessager(convertToFootstepPlan(footstepDataListMessage));
   }

   public static ArrayList<Pair<RobotSide, Pose3D>> reduceFootstepPlanForUIMessager(FootstepPlan footstepPlan)
   {
      ArrayList<Pair<RobotSide, Pose3D>> footstepLocations = new ArrayList<>();
      for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)  // this code makes the message smaller to send over the network, TODO investigate
      {
         FramePose3D soleFramePoseToPack = new FramePose3D();
         footstepPlan.getFootstep(i).getPose(soleFramePoseToPack);
         footstepLocations.add(new MutablePair<>(footstepPlan.getFootstep(i).getRobotSide(), new Pose3D(soleFramePoseToPack)));
      }
      return footstepLocations;
   }
}
