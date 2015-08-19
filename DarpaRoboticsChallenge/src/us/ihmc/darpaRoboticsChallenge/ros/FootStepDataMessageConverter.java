package us.ihmc.darpaRoboticsChallenge.ros;

import geometry_msgs.Quaternion;
import geometry_msgs.Vector3;
import ihmc_msgs.FootstepDataMessage;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.communication.packets.walking.FootstepData;
import us.ihmc.robotics.robotSide.RobotSide;

public class FootStepDataMessageConverter
{
   public static FootstepData convertFootStepData(FootstepDataMessage msg)
   {
      RobotSide robotSide = RobotSide.values[(int) msg.getRobotSide()];

      Vector3 loc = msg.getLocation();
      Quaternion quat = msg.getOrientation();

      Point3d location = new Point3d(loc.getX(), loc.getY(), loc.getZ());
      Quat4d orientation = new Quat4d(quat.getX(), quat.getY(), quat.getZ(), quat.getW());

      return new FootstepData(robotSide, location, orientation);
   }

   public static void convertFootStepDataList(List<FootstepDataMessage> footStepDataList, ArrayList<FootstepData> footStepDataArrayList)
   {
      for (int i = 0; i < footStepDataList.size(); i++)
      {
         FootstepData footStep = convertFootStepData(footStepDataList.get(i));
         footStepDataArrayList.add(footStep);
      }
   }
}
