package us.ihmc.avatar.ros;

import java.util.ArrayList;
import java.util.List;

import geometry_msgs.Quaternion;
import geometry_msgs.Vector3;
import ihmc_msgs.FootstepDataRosMessage;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.robotSide.RobotSide;

public class FootStepDataMessageConverter
{
   public static us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage convertFootStepData(FootstepDataRosMessage msg)
   {
      RobotSide robotSide = RobotSide.values[(int) msg.getRobotSide()];

      Vector3 loc = msg.getLocation();
      Quaternion quat = msg.getOrientation();

      Point3D location = new Point3D(loc.getX(), loc.getY(), loc.getZ());
      us.ihmc.euclid.tuple4D.Quaternion orientation = new us.ihmc.euclid.tuple4D.Quaternion(quat.getX(), quat.getY(), quat.getZ(), quat.getW());

      return new us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage(robotSide, location, orientation);
   }

   public static void convertFootStepDataList(List<FootstepDataRosMessage> footStepDataList, ArrayList<us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage> footStepDataArrayList)
   {
      for (int i = 0; i < footStepDataList.size(); i++)
      {
         us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage footStep = convertFootStepData(footStepDataList.get(i));
         footStepDataArrayList.add(footStep);
      }
   }
}
