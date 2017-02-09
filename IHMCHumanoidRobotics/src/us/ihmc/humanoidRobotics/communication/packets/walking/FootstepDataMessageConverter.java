package us.ihmc.humanoidRobotics.communication.packets.walking;

import java.util.ArrayList;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.SimpleFootstep;
import us.ihmc.humanoidRobotics.communication.packets.ExecutionMode;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage.FootstepOrigin;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
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
      footstepDataListMessage.setDefaultSwingTime(swingTime);
      footstepDataListMessage.setDefaultTransferTime(transferTime);

      for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)
      {
         SimpleFootstep footstep = footstepPlan.getFootstep(i);

         FramePose footstepPose = new FramePose();
         footstep.getSoleFramePose(footstepPose);
         Point3d location = new Point3d();
         Quat4d orientation = new Quat4d();
         footstepPose.getPosition(location);
         footstepPose.getOrientation(orientation);

         FootstepDataMessage footstepData = new FootstepDataMessage(footstep.getRobotSide(), location, orientation);
         footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);

         if (footstep.hasFoothold())
         {
            ConvexPolygon2d foothold = new ConvexPolygon2d();
            footstep.getFoothold(foothold);

            if (foothold.getNumberOfVertices() != 4)
               ConvexPolygonTools.limitVerticesConservative(foothold, 4);

            ArrayList<Point2d> contactPoints = new ArrayList<>();
            for (int contactPointIdx = 0; contactPointIdx < 4; contactPointIdx++)
               contactPoints.add(new Point2d(foothold.getVertex(contactPointIdx)));
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
         Point3d location = new Point3d();
         Quat4d orientation = new Quat4d();
         footstepPose.getPosition(location);
         footstepPose.getOrientation(orientation);

         FootstepDataMessage footstepData = new FootstepDataMessage(footstep.getRobotSide(), location, orientation);
         footstepData.setOrigin(FootstepOrigin.AT_SOLE_FRAME);

         if (footstep.hasFoothold())
         {
            ConvexPolygon2d foothold = new ConvexPolygon2d();
            footstep.getFoothold(foothold);

            if (foothold.getNumberOfVertices() != 4)
               ConvexPolygonTools.limitVerticesConservative(foothold, 4);

            ArrayList<Point2d> contactPoints = new ArrayList<>();
            for (int contactPointIdx = 0; contactPointIdx < 4; contactPointIdx++)
               contactPoints.add(new Point2d(foothold.getVertex(contactPointIdx)));
            footstepData.setPredictedContactPoints(contactPoints);
         }

         footstepDataListMessage.add(footstepData);
      }
   }
}
