package us.ihmc.footstepPlanning;

import java.util.ArrayList;

import org.apache.commons.lang3.tuple.MutablePair;
import org.apache.commons.lang3.tuple.Pair;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.footstep.SimpleFootstep;
import us.ihmc.idl.IDLSequence;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.robotSide.RobotSide;

public class FootstepDataMessageConverter
{
   public static FootstepDataListMessage createFootstepDataListFromPlan(FootstepPlan footstepPlan, double swingTime,
         double transferTime, ExecutionMode executionMode)
   {
      if (footstepPlan == null)
         return null;

      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      footstepDataListMessage.setDefaultSwingDuration(swingTime);
      footstepDataListMessage.setDefaultTransferDuration(transferTime);
      footstepDataListMessage.setOffsetFootstepsHeightWithExecutionError(true);

      for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)
      {
         SimpleFootstep footstep = footstepPlan.getFootstep(i);

         FramePose3D footstepPose = new FramePose3D();
         footstep.getSoleFramePose(footstepPose);
         Point3D location = new Point3D(footstepPose.getPosition());
         Quaternion orientation = new Quaternion(footstepPose.getOrientation());

         FootstepDataMessage footstepData = HumanoidMessageTools.createFootstepDataMessage(footstep.getRobotSide(), location, orientation);

         if (footstep.hasFoothold())
         {
            ConvexPolygon2D foothold = new ConvexPolygon2D();
            footstep.getFoothold(foothold);

            if (foothold.getNumberOfVertices() != 4)
               ConvexPolygonTools.limitVerticesConservative(foothold, 4);

            ArrayList<Point2D> contactPoints = new ArrayList<>();
            for (int contactPointIdx = 0; contactPointIdx < 4; contactPointIdx++)
               contactPoints.add(new Point2D(foothold.getVertex(contactPointIdx)));
            HumanoidMessageTools.packPredictedContactPoints(contactPoints, footstepData);
         }

         footstepDataListMessage.getFootstepDataList().add().set(footstepData);
      }

      return footstepDataListMessage;
   }
   
   public static void appendPlanToMessage(FootstepPlan footstepPlan, FootstepDataListMessage footstepDataListMessage)
   {
      for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)
      {
         SimpleFootstep footstep = footstepPlan.getFootstep(i);

         FramePose3D footstepPose = new FramePose3D();
         footstep.getSoleFramePose(footstepPose);
         Point3D location = new Point3D(footstepPose.getPosition());
         Quaternion orientation = new Quaternion(footstepPose.getOrientation());

         FootstepDataMessage footstepData = HumanoidMessageTools.createFootstepDataMessage(footstep.getRobotSide(), location, orientation);

         if (footstep.hasFoothold())
         {
            ConvexPolygon2D foothold = new ConvexPolygon2D();
            footstep.getFoothold(foothold);

            if (foothold.getNumberOfVertices() != 4)
               ConvexPolygonTools.limitVerticesConservative(foothold, 4);

            ArrayList<Point2D> contactPoints = new ArrayList<>();
            for (int contactPointIdx = 0; contactPointIdx < 4; contactPointIdx++)
               contactPoints.add(new Point2D(foothold.getVertex(contactPointIdx)));
            HumanoidMessageTools.packPredictedContactPoints(contactPoints, footstepData);
         }

         footstepDataListMessage.getFootstepDataList().add().set(footstepData);
      }
   }

   public static FootstepPlan convertToFootstepPlan(FootstepDataListMessage footstepDataListMessage)
   {
      FootstepPlan footstepPlan = new FootstepPlan();

      for (FootstepDataMessage footstepMessage : footstepDataListMessage.getFootstepDataList())
      {
         FramePose3D stepPose = new FramePose3D();
         stepPose.getPosition().set(footstepMessage.getLocation());
         stepPose.getOrientation().set(footstepMessage.getOrientation());
         SimpleFootstep simpleFootstep = footstepPlan.addFootstep(RobotSide.fromByte(footstepMessage.getRobotSide()), stepPose);

         IDLSequence.Object<Point3D> predictedContactPoints = footstepMessage.getPredictedContactPoints2d();
         if (!predictedContactPoints.isEmpty())
         {
            ConvexPolygon2D foothold = new ConvexPolygon2D();
            for (int i = 0; i < predictedContactPoints.size(); i++)
            {
               foothold.addVertex(predictedContactPoints.get(i));
            }
            foothold.update();
            simpleFootstep.setFoothold(foothold);
         }
      }

      return footstepPlan;
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
         footstepPlan.getFootstep(i).getSoleFramePose(soleFramePoseToPack);
         footstepLocations.add(new MutablePair<>(footstepPlan.getFootstep(i).getRobotSide(), new Pose3D(soleFramePoseToPack)));
      }
      return footstepLocations;
   }
}
