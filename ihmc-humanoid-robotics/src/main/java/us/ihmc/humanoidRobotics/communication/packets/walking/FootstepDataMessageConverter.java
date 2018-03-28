package us.ihmc.humanoidRobotics.communication.packets.walking;

import java.util.ArrayList;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.SimpleFootstep;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.robotics.geometry.ConvexPolygonTools;

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
            MessageTools.copyData(contactPoints, footstepData.predictedContactPoints);
         }

         footstepDataListMessage.footstepDataList.add().set(footstepData);
      }

      footstepDataListMessage.queueingProperties.setExecutionMode(executionMode.toByte());
      footstepDataListMessage.queueingProperties.setPreviousMessageId(FootstepDataListMessage.VALID_MESSAGE_DEFAULT_ID);
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
            MessageTools.copyData(contactPoints, footstepData.predictedContactPoints);
         }

         footstepDataListMessage.footstepDataList.add().set(footstepData);
      }
   }
}
