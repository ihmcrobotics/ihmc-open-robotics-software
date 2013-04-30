package us.ihmc.darpaRoboticsChallenge.ros;

import static atlas_msgs.AtlasState.back_lbz;
import static atlas_msgs.AtlasState.back_mby;
import static atlas_msgs.AtlasState.back_ubx;
import static atlas_msgs.AtlasState.l_arm_elx;
import static atlas_msgs.AtlasState.l_arm_ely;
import static atlas_msgs.AtlasState.l_arm_mwx;
import static atlas_msgs.AtlasState.l_arm_shx;
import static atlas_msgs.AtlasState.l_arm_usy;
import static atlas_msgs.AtlasState.l_arm_uwy;
import static atlas_msgs.AtlasState.l_leg_kny;
import static atlas_msgs.AtlasState.l_leg_lax;
import static atlas_msgs.AtlasState.l_leg_lhy;
import static atlas_msgs.AtlasState.l_leg_mhx;
import static atlas_msgs.AtlasState.l_leg_uay;
import static atlas_msgs.AtlasState.l_leg_uhz;
import static atlas_msgs.AtlasState.neck_ay;
import static atlas_msgs.AtlasState.r_arm_elx;
import static atlas_msgs.AtlasState.r_arm_ely;
import static atlas_msgs.AtlasState.r_arm_mwx;
import static atlas_msgs.AtlasState.r_arm_shx;
import static atlas_msgs.AtlasState.r_arm_usy;
import static atlas_msgs.AtlasState.r_arm_uwy;
import static atlas_msgs.AtlasState.r_leg_kny;
import static atlas_msgs.AtlasState.r_leg_lax;
import static atlas_msgs.AtlasState.r_leg_lhy;
import static atlas_msgs.AtlasState.r_leg_mhx;
import static atlas_msgs.AtlasState.r_leg_uay;
import static atlas_msgs.AtlasState.r_leg_uhz;

import java.util.Map;

import us.ihmc.utilities.screwTheory.OneDoFJoint;

public class ROSAtlasJointMap
{
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
