package us.ihmc.darpaRoboticsChallenge.ros;

import java.util.Map;

import us.ihmc.utilities.screwTheory.OneDoFJoint;

public class ROSAtlasJointMap
{
   /*
    * Copied from
    *    https://bitbucket.org/osrf/drcsim/raw/default/ros/atlas_msgs/msg/AtlasState.msg
    */
   
   
   public final static int back_lbz  = 0;
   public final static int back_mby  = 1;
   public final static int back_ubx  = 2;
   public final static int neck_ay   = 3;
   public final static int l_leg_uhz = 4;
   public final static int l_leg_mhx = 5;
   public final static int l_leg_lhy = 6;
   public final static int l_leg_kny = 7;
   public final static int l_leg_uay = 8;
   public final static int l_leg_lax = 9;
   public final static int r_leg_uhz = 10;
   public final static int r_leg_mhx = 11;
   public final static int r_leg_lhy = 12;
   public final static int r_leg_kny = 13;
   public final static int r_leg_uay = 14;
   public final static int r_leg_lax = 15;
   public final static int l_arm_usy = 16;
   public final static int l_arm_shx = 17;
   public final static int l_arm_ely = 18;
   public final static int l_arm_elx = 19;
   public final static int l_arm_uwy = 20;
   public final static int l_arm_mwx = 21;
   public final static int r_arm_usy = 22;
   public final static int r_arm_shx = 23;
   public final static int r_arm_ely = 24;
   public final static int r_arm_elx = 25;
   public final static int r_arm_uwy = 26;
   public final static int r_arm_mwx = 27;
   
   
   public final static int numberOfJoints = r_arm_mwx + 1; 
   
   public static OneDoFJoint[] getJointMap(Map<String, OneDoFJoint> jointsByName)
   {
      OneDoFJoint[] joints = new OneDoFJoint[atlas_msgs.AtlasState.r_arm_mwx + 1];
      joints[back_lbz]  = jointsByName.get("back_lbz");
      joints[back_mby]  = jointsByName.get("back_mby");
      joints[back_ubx]  = jointsByName.get("back_ubx");
      joints[neck_ay]   = jointsByName.get("neck_ay");
      joints[l_leg_uhz] = jointsByName.get("l_leg_uhz");
      joints[l_leg_mhx] = jointsByName.get("l_leg_mhx");
      joints[l_leg_lhy] = jointsByName.get("l_leg_lhy");
      joints[l_leg_kny] = jointsByName.get("l_leg_kny");
      joints[l_leg_uay] = jointsByName.get("l_leg_uay");
      joints[l_leg_lax] = jointsByName.get("l_leg_lax");
      joints[r_leg_uhz] = jointsByName.get("r_leg_uhz");
      joints[r_leg_mhx] = jointsByName.get("r_leg_mhx");
      joints[r_leg_lhy] = jointsByName.get("r_leg_lhy");
      joints[r_leg_kny] = jointsByName.get("r_leg_kny");
      joints[r_leg_uay] = jointsByName.get("r_leg_uay");
      joints[r_leg_lax] = jointsByName.get("r_leg_lax");
      joints[l_arm_usy] = jointsByName.get("l_arm_usy");
      joints[l_arm_shx] = jointsByName.get("l_arm_shx");
      joints[l_arm_ely] = jointsByName.get("l_arm_ely");
      joints[l_arm_elx] = jointsByName.get("l_arm_elx");
      joints[l_arm_uwy] = jointsByName.get("l_arm_uwy");
      joints[l_arm_mwx] = jointsByName.get("l_arm_mwx");
      joints[r_arm_usy] = jointsByName.get("r_arm_usy");
      joints[r_arm_shx] = jointsByName.get("r_arm_shx");
      joints[r_arm_ely] = jointsByName.get("r_arm_ely");
      joints[r_arm_elx] = jointsByName.get("r_arm_elx");
      joints[r_arm_uwy] = jointsByName.get("r_arm_uwy");
      joints[r_arm_mwx] = jointsByName.get("r_arm_mwx");

      return joints;
   }

   public static String getBodyIMUName()
   {
      return "imu_sensor";
   }

   public static String getLeftFootForceSensorName()
   {
      return "l_leg_lax";
   }

   public static String getRightFootForceSensorName()
   {
      return "r_leg_lax";
   }

   public static String getLeftHandForceSensorName()
   {
      return "l_arm_mwx";
   }

   public static String getRightHandForceSensorName()
   {
      return "r_arm_mwx";
   }

   public static String[] getForceSensors()
   {
      return new String[] { getLeftFootForceSensorName(), getRightFootForceSensorName(), getLeftHandForceSensorName(), getRightHandForceSensorName() };
   }
}
