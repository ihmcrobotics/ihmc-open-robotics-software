package us.ihmc.darpaRoboticsChallenge.ros;

import java.util.LinkedHashMap;
import java.util.Map;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

public class ROSAtlasJointMap
{
   /*
    * Copied from
    *    https://bitbucket.org/osrf/drcsim/raw/default/ros/atlas_msgs/msg/AtlasState.msg
    */
	public final static boolean ISATLASV3NAMING = false;
   
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
   
   public static String[] jointNames = new String[numberOfJoints];
   static
   {	
		if (ISATLASV3NAMING) {
			jointNames[back_lbz] = "back_bkz";
			jointNames[back_mby] = "back_bky";
			jointNames[back_ubx] = "back_bkx";
			jointNames[neck_ay] = "neck_ry";
			jointNames[l_leg_uhz] = "l_leg_hpz";
			jointNames[l_leg_mhx] = "l_leg_hpx";
			jointNames[l_leg_lhy] = "l_leg_hpy";
			jointNames[l_leg_kny] = "l_leg_kny";
			jointNames[l_leg_uay] = "l_leg_aky";
			jointNames[l_leg_lax] = "l_leg_akx";
			jointNames[r_leg_uhz] = "r_leg_hpz";
			jointNames[r_leg_mhx] = "r_leg_hpx";
			jointNames[r_leg_lhy] = "r_leg_hpy";
			jointNames[r_leg_kny] = "r_leg_kny";
			jointNames[r_leg_uay] = "r_leg_aky";
			jointNames[r_leg_lax] = "r_leg_akx";
			jointNames[l_arm_usy] = "l_arm_shy";
			jointNames[l_arm_shx] = "l_arm_shx";
			jointNames[l_arm_ely] = "l_arm_ely";
			jointNames[l_arm_elx] = "l_arm_elx";
			jointNames[l_arm_uwy] = "l_arm_wry";
			jointNames[l_arm_mwx] = "l_arm_wrx";
			jointNames[r_arm_usy] = "r_arm_shy";
			jointNames[r_arm_shx] = "r_arm_shx";
			jointNames[r_arm_ely] = "r_arm_ely";
			jointNames[r_arm_elx] = "r_arm_elx";
			jointNames[r_arm_uwy] = "r_arm_wry";
			jointNames[r_arm_mwx] = "r_arm_wrx";
		} else {

			jointNames[back_lbz] = "back_lbz";
			jointNames[back_mby] = "back_mby";
			jointNames[back_ubx] = "back_ubx";
			jointNames[neck_ay] = "neck_ay";
			jointNames[l_leg_uhz] = "l_leg_uhz";
			jointNames[l_leg_mhx] = "l_leg_mhx";
			jointNames[l_leg_lhy] = "l_leg_lhy";
			jointNames[l_leg_kny] = "l_leg_kny";
			jointNames[l_leg_uay] = "l_leg_uay";
			jointNames[l_leg_lax] = "l_leg_lax";
			jointNames[r_leg_uhz] = "r_leg_uhz";
			jointNames[r_leg_mhx] = "r_leg_mhx";
			jointNames[r_leg_lhy] = "r_leg_lhy";
			jointNames[r_leg_kny] = "r_leg_kny";
			jointNames[r_leg_uay] = "r_leg_uay";
			jointNames[r_leg_lax] = "r_leg_lax";
			jointNames[l_arm_usy] = "l_arm_usy";
			jointNames[l_arm_shx] = "l_arm_shx";
			jointNames[l_arm_ely] = "l_arm_ely";
			jointNames[l_arm_elx] = "l_arm_elx";
			jointNames[l_arm_uwy] = "l_arm_uwy";
			jointNames[l_arm_mwx] = "l_arm_mwx";
			jointNames[r_arm_usy] = "r_arm_usy";
			jointNames[r_arm_shx] = "r_arm_shx";
			jointNames[r_arm_ely] = "r_arm_ely";
			jointNames[r_arm_elx] = "r_arm_elx";
			jointNames[r_arm_uwy] = "r_arm_uwy";
			jointNames[r_arm_mwx] = "r_arm_mwx";
		}
   }
   
   public static LinkedHashMap<String, Integer> nameToIndexMap = new LinkedHashMap<String, Integer>();
   static
   {
      for (int i=0; i<jointNames.length; i++)
      {
         nameToIndexMap.put(jointNames[i], i);
      }
   }
   
   public static final SideDependentList<String[]> forcedSideDependentJointNames = new SideDependentList<String[]>();
   static
   {
	  String[] jointNamesRight = new String[numberOfJoints];
	  jointNamesRight[back_lbz] = jointNames[back_lbz];
	  jointNamesRight[back_mby] = jointNames[back_mby];
	  jointNamesRight[back_ubx] = jointNames[back_ubx];
	  jointNamesRight[neck_ay] = jointNames[neck_ay];
	  jointNamesRight[l_leg_uhz] = jointNames[r_leg_uhz];
	  jointNamesRight[l_leg_mhx] = jointNames[r_leg_mhx];
	  jointNamesRight[l_leg_lhy] = jointNames[r_leg_lhy];
	  jointNamesRight[l_leg_kny] = jointNames[r_leg_kny];
	  jointNamesRight[l_leg_uay] = jointNames[r_leg_uay];
	  jointNamesRight[l_leg_lax] = jointNames[r_leg_lax];
	  jointNamesRight[r_leg_uhz] = jointNames[r_leg_uhz];
	  jointNamesRight[r_leg_mhx] = jointNames[r_leg_mhx];
	  jointNamesRight[r_leg_lhy] = jointNames[r_leg_lhy];
	  jointNamesRight[r_leg_kny] = jointNames[r_leg_kny];
	  jointNamesRight[r_leg_uay] = jointNames[r_leg_uay];
	  jointNamesRight[r_leg_lax] = jointNames[r_leg_lax];
	  jointNamesRight[l_arm_usy] = jointNames[r_arm_usy];
	  jointNamesRight[l_arm_shx] = jointNames[r_arm_shx];
	  jointNamesRight[l_arm_ely] = jointNames[r_arm_ely];
	  jointNamesRight[l_arm_elx] = jointNames[r_arm_elx];
	  jointNamesRight[l_arm_uwy] = jointNames[r_arm_uwy];
	  jointNamesRight[l_arm_mwx] = jointNames[r_arm_mwx];
	  jointNamesRight[r_arm_usy] = jointNames[r_arm_usy];
	  jointNamesRight[r_arm_shx] = jointNames[r_arm_shx];
	  jointNamesRight[r_arm_ely] = jointNames[r_arm_ely];
	  jointNamesRight[r_arm_elx] = jointNames[r_arm_elx];
	  jointNamesRight[r_arm_uwy] = jointNames[r_arm_uwy];
	  jointNamesRight[r_arm_mwx] = jointNames[r_arm_mwx];
	  
	  forcedSideDependentJointNames.put(RobotSide.RIGHT, jointNamesRight);
	
	  String[] jointNamesLeft = new String[numberOfJoints];
	  jointNamesLeft[back_lbz] = jointNames[back_lbz];
	  jointNamesLeft[back_mby] = jointNames[back_mby];
	  jointNamesLeft[back_ubx] = jointNames[back_ubx];
	  jointNamesLeft[neck_ay] = jointNames[neck_ay];
	  jointNamesLeft[l_leg_uhz] = jointNames[l_leg_uhz];
	  jointNamesLeft[l_leg_mhx] = jointNames[l_leg_mhx];
	  jointNamesLeft[l_leg_lhy] = jointNames[l_leg_lhy];
	  jointNamesLeft[l_leg_kny] = jointNames[l_leg_kny];
	  jointNamesLeft[l_leg_uay] = jointNames[l_leg_uay];
	  jointNamesLeft[l_leg_lax] = jointNames[l_leg_lax];
	  jointNamesLeft[r_leg_uhz] = jointNames[l_leg_uhz];
	  jointNamesLeft[r_leg_mhx] = jointNames[l_leg_mhx];
	  jointNamesLeft[r_leg_lhy] = jointNames[l_leg_lhy];
	  jointNamesLeft[r_leg_kny] = jointNames[l_leg_kny];
	  jointNamesLeft[r_leg_uay] = jointNames[l_leg_uay];
	  jointNamesLeft[r_leg_lax] = jointNames[l_leg_lax];
	  jointNamesLeft[l_arm_usy] = jointNames[l_arm_usy];
	  jointNamesLeft[l_arm_shx] = jointNames[l_arm_shx];
	  jointNamesLeft[l_arm_ely] = jointNames[l_arm_ely];
	  jointNamesLeft[l_arm_elx] = jointNames[l_arm_elx];
	  jointNamesLeft[l_arm_uwy] = jointNames[l_arm_uwy];
	  jointNamesLeft[l_arm_mwx] = jointNames[l_arm_mwx];
	  jointNamesLeft[r_arm_usy] = jointNames[l_arm_usy];
	  jointNamesLeft[r_arm_shx] = jointNames[l_arm_shx];
	  jointNamesLeft[r_arm_ely] = jointNames[l_arm_ely];
	  jointNamesLeft[r_arm_elx] = jointNames[l_arm_elx];
	  jointNamesLeft[r_arm_uwy] = jointNames[l_arm_uwy];
	  jointNamesLeft[r_arm_mwx] = jointNames[l_arm_mwx];
	  
	  forcedSideDependentJointNames.put(RobotSide.LEFT, jointNamesLeft);
   }
   
   
   public static OneDoFJoint[] getJointMap(Map<String, OneDoFJoint> jointsByName)
   {
      OneDoFJoint[] joints = new OneDoFJoint[numberOfJoints];
      for(int i = 0; i < numberOfJoints; i++)
      {
         joints[i] = jointsByName.get(jointNames[i]);
      }
      
      return joints;
   }

   public static String getBodyIMUName()
   {
      return DRCRobotJointMap.bodyIMUSensor;
   }

   public static String getLeftFootForceSensorName()
   {
      return jointNames[l_leg_lax];
   }

   public static String getRightFootForceSensorName()
   {
      return jointNames[r_leg_lax];
   }

   public static String getLeftHandForceSensorName()
   {
      return jointNames[l_arm_mwx];
   }

   public static String getRightHandForceSensorName()
   {
      return jointNames[r_arm_mwx];
   }

   public static String getHokuyoJointName()
   {
      return "hokuyo_joint";
   }

   public static String[] getForceSensors()
   {
      return new String[] { getLeftFootForceSensorName(), getRightFootForceSensorName(), getLeftHandForceSensorName(), getRightHandForceSensorName() };
   }
}
