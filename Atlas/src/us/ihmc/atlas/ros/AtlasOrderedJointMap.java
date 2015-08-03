package us.ihmc.atlas.ros;

import java.util.LinkedHashMap;
import java.util.Map;

import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class AtlasOrderedJointMap
{
   /*
    * Copied from
    * https://bitbucket.org/osrf/drcsim/raw/default/ros/atlas_msgs/msg
    * /AtlasState.msg
    */

   public final static int back_bkz = 0;
   public final static int back_bky = 1;
   public final static int back_bkx = 2;
   public final static int neck_ry = 3;
   public final static int l_leg_hpz = 4;
   public final static int l_leg_hpx = 5;
   public final static int l_leg_hpy = 6;
   public final static int l_leg_kny = 7;
   public final static int l_leg_aky = 8;
   public final static int l_leg_akx = 9;
   public final static int r_leg_hpz = 10;
   public final static int r_leg_hpx = 11;
   public final static int r_leg_hpy = 12;
   public final static int r_leg_kny = 13;
   public final static int r_leg_aky = 14;
   public final static int r_leg_akx = 15;
   public final static int l_arm_shz = 16;
   public final static int l_arm_shx = 17;
   public final static int l_arm_ely = 18;
   public final static int l_arm_elx = 19;
   public final static int l_arm_wry = 20;
   public final static int l_arm_wrx = 21;
   public final static int l_arm_wry2 = 22;
   public final static int r_arm_shz = 23;
   public final static int r_arm_shx = 24;
   public final static int r_arm_ely = 25;
   public final static int r_arm_elx = 26;
   public final static int r_arm_wry = 27;
   public final static int r_arm_wrx = 28;
   public final static int r_arm_wry2 = 29;

   public final static int numberOfJoints = r_arm_wry2 + 1;

   public static final String[] jointNames = new String[numberOfJoints];
   static
   {
      jointNames[back_bkz] = "back_bkz";
      jointNames[back_bky] = "back_bky";
      jointNames[back_bkx] = "back_bkx";
      jointNames[neck_ry] = "neck_ry";
      jointNames[l_leg_hpz] = "l_leg_hpz";
      jointNames[l_leg_hpx] = "l_leg_hpx";
      jointNames[l_leg_hpy] = "l_leg_hpy";
      jointNames[l_leg_kny] = "l_leg_kny";
      jointNames[l_leg_aky] = "l_leg_aky";
      jointNames[l_leg_akx] = "l_leg_akx";
      jointNames[r_leg_hpz] = "r_leg_hpz";
      jointNames[r_leg_hpx] = "r_leg_hpx";
      jointNames[r_leg_hpy] = "r_leg_hpy";
      jointNames[r_leg_kny] = "r_leg_kny";
      jointNames[r_leg_aky] = "r_leg_aky";
      jointNames[r_leg_akx] = "r_leg_akx";
      jointNames[l_arm_shz] = "l_arm_shz";
      jointNames[l_arm_shx] = "l_arm_shx";
      jointNames[l_arm_ely] = "l_arm_ely";
      jointNames[l_arm_elx] = "l_arm_elx";
      jointNames[l_arm_wry] = "l_arm_wry";
      jointNames[l_arm_wrx] = "l_arm_wrx";
      jointNames[l_arm_wry2] = "l_arm_wry2";
      jointNames[r_arm_shz] = "r_arm_shz";
      jointNames[r_arm_shx] = "r_arm_shx";
      jointNames[r_arm_ely] = "r_arm_ely";
      jointNames[r_arm_elx] = "r_arm_elx";
      jointNames[r_arm_wry] = "r_arm_wry";
      jointNames[r_arm_wrx] = "r_arm_wrx";
      jointNames[r_arm_wry2] = "r_arm_wry2";
   }

   public static final LinkedHashMap<String, Integer> nameToIndexMap = new LinkedHashMap<String, Integer>();
   static
   {
      for (int i = 0; i < jointNames.length; i++)
      {
         nameToIndexMap.put(jointNames[i], i);
      }
   }

   public static final SideDependentList<String[]> forcedSideDependentJointNames = new SideDependentList<String[]>();
   static
   {
      String[] jointNamesRight = new String[numberOfJoints];
      jointNamesRight[back_bkz] = jointNames[back_bkz];
      jointNamesRight[back_bky] = jointNames[back_bky];
      jointNamesRight[back_bkx] = jointNames[back_bkx];
      jointNamesRight[neck_ry] = jointNames[neck_ry];
      jointNamesRight[l_leg_hpz] = jointNames[r_leg_hpz];
      jointNamesRight[l_leg_hpx] = jointNames[r_leg_hpx];
      jointNamesRight[l_leg_hpy] = jointNames[r_leg_hpy];
      jointNamesRight[l_leg_kny] = jointNames[r_leg_kny];
      jointNamesRight[l_leg_aky] = jointNames[r_leg_aky];
      jointNamesRight[l_leg_akx] = jointNames[r_leg_akx];
      jointNamesRight[r_leg_hpz] = jointNames[r_leg_hpz];
      jointNamesRight[r_leg_hpx] = jointNames[r_leg_hpx];
      jointNamesRight[r_leg_hpy] = jointNames[r_leg_hpy];
      jointNamesRight[r_leg_kny] = jointNames[r_leg_kny];
      jointNamesRight[r_leg_aky] = jointNames[r_leg_aky];
      jointNamesRight[r_leg_akx] = jointNames[r_leg_akx];
      jointNamesRight[l_arm_shz] = jointNames[r_arm_shz];
      jointNamesRight[l_arm_shx] = jointNames[r_arm_shx];
      jointNamesRight[l_arm_ely] = jointNames[r_arm_ely];
      jointNamesRight[l_arm_elx] = jointNames[r_arm_elx];
      jointNamesRight[l_arm_wry] = jointNames[r_arm_wry];
      jointNamesRight[l_arm_wrx] = jointNames[r_arm_wrx];
      jointNamesRight[l_arm_wry2] = jointNames[r_arm_wry2];
      jointNamesRight[r_arm_shz] = jointNames[r_arm_shz];
      jointNamesRight[r_arm_shx] = jointNames[r_arm_shx];
      jointNamesRight[r_arm_ely] = jointNames[r_arm_ely];
      jointNamesRight[r_arm_elx] = jointNames[r_arm_elx];
      jointNamesRight[r_arm_wry] = jointNames[r_arm_wry];
      jointNamesRight[r_arm_wrx] = jointNames[r_arm_wrx];
      jointNamesRight[r_arm_wry2] = jointNames[r_arm_wry2];

      forcedSideDependentJointNames.put(RobotSide.RIGHT, jointNamesRight);

      String[] jointNamesLeft = new String[numberOfJoints];
      jointNamesLeft[back_bkz] = jointNames[back_bkz];
      jointNamesLeft[back_bky] = jointNames[back_bky];
      jointNamesLeft[back_bkx] = jointNames[back_bkx];
      jointNamesLeft[neck_ry] = jointNames[neck_ry];
      jointNamesLeft[l_leg_hpz] = jointNames[l_leg_hpz];
      jointNamesLeft[l_leg_hpx] = jointNames[l_leg_hpx];
      jointNamesLeft[l_leg_hpy] = jointNames[l_leg_hpy];
      jointNamesLeft[l_leg_kny] = jointNames[l_leg_kny];
      jointNamesLeft[l_leg_aky] = jointNames[l_leg_aky];
      jointNamesLeft[l_leg_akx] = jointNames[l_leg_akx];
      jointNamesLeft[r_leg_hpz] = jointNames[l_leg_hpz];
      jointNamesLeft[r_leg_hpx] = jointNames[l_leg_hpx];
      jointNamesLeft[r_leg_hpy] = jointNames[l_leg_hpy];
      jointNamesLeft[r_leg_kny] = jointNames[l_leg_kny];
      jointNamesLeft[r_leg_aky] = jointNames[l_leg_aky];
      jointNamesLeft[r_leg_akx] = jointNames[l_leg_akx];
      jointNamesLeft[l_arm_shz] = jointNames[l_arm_shz];
      jointNamesLeft[l_arm_shx] = jointNames[l_arm_shx];
      jointNamesLeft[l_arm_ely] = jointNames[l_arm_ely];
      jointNamesLeft[l_arm_elx] = jointNames[l_arm_elx];
      jointNamesLeft[l_arm_wry] = jointNames[l_arm_wry];
      jointNamesLeft[l_arm_wrx] = jointNames[l_arm_wrx];
      jointNamesLeft[l_arm_wry2] = jointNames[l_arm_wry2];
      jointNamesLeft[r_arm_shz] = jointNames[l_arm_shz];
      jointNamesLeft[r_arm_shx] = jointNames[l_arm_shx];
      jointNamesLeft[r_arm_ely] = jointNames[l_arm_ely];
      jointNamesLeft[r_arm_elx] = jointNames[l_arm_elx];
      jointNamesLeft[r_arm_wry] = jointNames[l_arm_wry];
      jointNamesLeft[r_arm_wrx] = jointNames[l_arm_wrx];
      jointNamesLeft[r_arm_wry2] = jointNames[l_arm_wry2];

      forcedSideDependentJointNames.put(RobotSide.LEFT, jointNamesLeft);
   }

   public static OneDoFJoint[] getJointMap(Map<String, OneDoFJoint> jointsByName)
   {
      OneDoFJoint[] joints = new OneDoFJoint[numberOfJoints];
      for (int i = 0; i < numberOfJoints; i++)
      {
         joints[i] = jointsByName.get(jointNames[i]);
      }

      return joints;
   }

   public static String getLeftFootForceSensorName()
   {
      return jointNames[l_leg_akx];
   }

   public static String getRightFootForceSensorName()
   {
      return jointNames[r_leg_akx];
   }

   public static String getLeftHandForceSensorName()
   {
      return jointNames[l_arm_wry2];
   }

   public static String getRightHandForceSensorName()
   {
      return jointNames[r_arm_wry2];
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
