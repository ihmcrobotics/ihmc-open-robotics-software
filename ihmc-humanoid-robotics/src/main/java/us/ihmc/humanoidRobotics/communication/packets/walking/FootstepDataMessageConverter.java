package us.ihmc.humanoidRobotics.communication.packets.walking;

import java.util.ArrayList;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.SimpleFootstep;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.geometry.FramePose;

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

         FramePose footstepPose = new FramePose();
         footstep.getSoleFramePose(footstepPose);
         Point3D location = new Point3D();
         Quaternion orientation = new Quaternion();
         footstepPose.getPosition(location);
         footstepPose.getOrientation(orientation);

         FootstepDataMessage footstepData = new FootstepDataMessage(footstep.getRobotSide(), location, orientation);

         if (footstep.hasFoothold())
         {
            ConvexPolygon2D foothold = new ConvexPolygon2D();
            footstep.getFoothold(foothold);

            if (foothold.getNumberOfVertices() != 4)
               ConvexPolygonTools.limitVerticesConservative(foothold, 4);

            ArrayList<Point2D> contactPoints = new ArrayList<>();
            for (int contactPointIdx = 0; contactPointIdx < 4; contactPointIdx++)
               contactPoints.add(new Point2D(foothold.getVertex(contactPointIdx)));
            footstepData.setPredictedContactPoints(contactPoints);
         }

         footstepDataListMessage.add(footstepData);
      }

      footstepDataListMessage.setExecutionMode(executionMode);
      return footstepDataListMessage;
   }
   
   public static void appendPlanToMessage(FootstepPlan footstepPlan, FootstepDataListMessage footstepDataListMessage)
   {
      for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)
      {
         SimpleFootstep footstep = footstepPlan.getFootstep(i);

         FramePose footstepPose = new FramePose();
         footstep.getSoleFramePose(footstepPose);
         Point3D location = new Point3D();
         Quaternion orientation = new Quaternion();
         footstepPose.getPosition(location);
         footstepPose.getOrientation(orientation);

         FootstepDataMessage footstepData = new FootstepDataMessage(footstep.getRobotSide(), location, orientation);

         if (footstep.hasFoothold())
         {
            ConvexPolygon2D foothold = new ConvexPolygon2D();
            footstep.getFoothold(foothold);

            if (foothold.getNumberOfVertices() != 4)
               ConvexPolygonTools.limitVerticesConservative(foothold, 4);

            ArrayList<Point2D> contactPoints = new ArrayList<>();
            for (int contactPointIdx = 0; contactPointIdx < 4; contactPointIdx++)
               contactPoints.add(new Point2D(foothold.getVertex(contactPointIdx)));
            footstepData.setPredictedContactPoints(contactPoints);
         }

         footstepDataListMessage.add(footstepData);
      }
   }
}
