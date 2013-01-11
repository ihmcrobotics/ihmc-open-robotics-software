package us.ihmc.darpaRoboticsChallenge;

import java.util.ArrayList;

import javax.vecmath.Vector3d;

public class DRCConfigParameters
{
   public static final boolean USE_GFE_ROBOT_MODEL = false;
   public static final boolean USE_R2_ROBOT_MODEL = !USE_GFE_ROBOT_MODEL;

   public static final double DRC_ROBOT_ANKLE_HEIGHT = 0.084;
   public static final double DRC_ROBOT_FOOT_WIDTH = 0.12;
   public static final double DRC_ROBOT_FOOT_FORWARD = 0.18;
   public static final double DRC_ROBOT_FOOT_BACK = 0.07;

   @SuppressWarnings("serial")
   public static final ArrayList<Vector3d> DRC_ROBOT_GROUND_CONTACT_POINT_OFFSET_FROM_FOOT = new ArrayList<Vector3d>()
   {
      {
         add(new Vector3d(-DRC_ROBOT_FOOT_BACK, -(DRC_ROBOT_FOOT_WIDTH / 2.0), -DRC_ROBOT_ANKLE_HEIGHT));
         add(new Vector3d(-DRC_ROBOT_FOOT_BACK, DRC_ROBOT_FOOT_WIDTH / 2.0, -DRC_ROBOT_ANKLE_HEIGHT));
         add(new Vector3d(DRC_ROBOT_FOOT_FORWARD, -(DRC_ROBOT_FOOT_WIDTH / 2.0), -DRC_ROBOT_ANKLE_HEIGHT));
         add(new Vector3d(DRC_ROBOT_FOOT_FORWARD, DRC_ROBOT_FOOT_WIDTH / 2.0, -DRC_ROBOT_ANKLE_HEIGHT));
      }
   };

   // Networking
   public static final String OPERATOR_INTERFACE_BG_VIDEO_SERVER_IP_ADDRESS = "localhost";
   public static final int OPERATOR_INTERFACE_BG_VIDEO_SERVER_PORT_NUMBER = 2099;
   public static final String ROBOT_DATA_RECEIVER_IP_ADDRESS = "localhost";
   public static final int ROBOT_DATA_RECEIVER_PORT_NUMBER = 7777;

}
