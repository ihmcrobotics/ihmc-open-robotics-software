package us.ihmc.footstepPlanning;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import org.apache.commons.lang3.tuple.MutablePair;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;

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
         PlannedFootstep footstep = footstepPlan.getFootstep(i);
         footstep.limitFootholdVertices();
         footstepDataListMessage.getFootstepDataList().add().set(footstep.getAsMessage());
      }
   }

   public static FootstepPlan convertToFootstepPlan(FootstepDataListMessage footstepDataListMessage)
   {
      FootstepPlan footstepPlan = new FootstepPlan();

      for (FootstepDataMessage footstepMessage : footstepDataListMessage.getFootstepDataList())
      {
         PlannedFootstep plannedFootstep = PlannedFootstep.getFromMessage(footstepMessage);
         footstepPlan.addFootstep(plannedFootstep);
         LogTools.warn("Custom Waypoints: Before: [{}], After: [{}]", footstepMessage.getCustomPositionWaypoints().size(),
                       plannedFootstep.getCustomWaypointPositions().size());
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
         footstepPlan.getFootstep(i).getFootstepPose(soleFramePoseToPack);
         footstepLocations.add(new MutablePair<>(footstepPlan.getFootstep(i).getRobotSide(), new Pose3D(soleFramePoseToPack)));
      }
      return footstepLocations;
   }

   public static FootstepDataListMessage convertFeetLocationsToMessage(ArrayList<Point3D> locations)
   {
      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      footstepDataListMessage.setDefaultSwingDuration(0.5);
      footstepDataListMessage.setDefaultTransferDuration(0.1);
      footstepDataListMessage.setOffsetFootstepsHeightWithExecutionError(true);

      // alternate left and right
      for (int i = 0; i < locations.size(); i++)
      {
         FootstepDataMessage footstepDataMessage = new FootstepDataMessage();
         footstepDataMessage.getLocation().set(locations.get(i));
         footstepDataMessage.getOrientation().set(new Quaternion(0.0, 0.0, 0.0, 1.0));
         footstepDataMessage.setRobotSide((i % 2 == 0 ? RobotSide.LEFT : RobotSide.RIGHT).toByte());

         footstepDataListMessage.getFootstepDataList().add().set(footstepDataMessage);
      }

      return footstepDataListMessage;
   }
}
