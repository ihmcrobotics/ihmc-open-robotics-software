package us.ihmc.wanderer.parameters;

import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;


public class WandererOrderedJointNames
{
   /*
    * Copied from
    *    https://bitbucket.org/osrf/drcsim/raw/default/ros/atlas_msgs/msg/AtlasState.msg
    */

   public final static int back_lbx = 0;
   public final static int back_mby = 1;
   public final static int back_ubz = 2;
   public final static int l_leg_mhx = 3;
   public final static int l_leg_uhz = 4;
   public final static int l_leg_lhy = 5;
   public final static int l_leg_kny = 6;
   public final static int l_leg_uay = 7;
   public final static int l_leg_lax = 8;
   public final static int r_leg_mhx = 9;
   public final static int r_leg_uhz = 10;
   public final static int r_leg_lhy = 11;
   public final static int r_leg_kny = 12;
   public final static int r_leg_uay = 13;
   public final static int r_leg_lax = 14;
   
   public final static int numberOfJoints = r_leg_lax + 1; 
   
   public static String[] jointNames = new String[numberOfJoints];
   static
   {
      jointNames[back_lbx] = "back_lbx";
      jointNames[back_mby] = "back_mby";
      jointNames[back_ubz] = "back_ubz";
      jointNames[l_leg_mhx] = "l_leg_mhx";
      jointNames[l_leg_uhz] = "l_leg_uhz";
      jointNames[l_leg_lhy] = "l_leg_lhy";
      jointNames[l_leg_kny] = "l_leg_kny";
      jointNames[l_leg_uay] = "l_leg_uay";
      jointNames[l_leg_lax] = "l_leg_lax";
      jointNames[r_leg_mhx] = "r_leg_mhx";
      jointNames[r_leg_uhz] = "r_leg_uhz";
      jointNames[r_leg_lhy] = "r_leg_lhy";
      jointNames[r_leg_kny] = "r_leg_kny";
      jointNames[r_leg_uay] = "r_leg_uay";
      jointNames[r_leg_lax] = "r_leg_lax";
   }

   public static final SideDependentList<String[]> forcedSideDependentJointNames = new SideDependentList<String[]>();
   static
   {
     String[] jointNamesRight = new String[numberOfJoints];
     jointNamesRight[back_ubz] = jointNames[back_ubz];
     jointNamesRight[back_mby] = jointNames[back_mby];
     jointNamesRight[back_lbx] = jointNames[back_lbx];
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
     
     forcedSideDependentJointNames.put(RobotSide.RIGHT, jointNamesRight);
   
     String[] jointNamesLeft = new String[numberOfJoints];
     jointNamesLeft[back_ubz] = jointNames[back_ubz];
     jointNamesLeft[back_mby] = jointNames[back_mby];
     jointNamesLeft[back_lbx] = jointNames[back_lbx];
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
     
     forcedSideDependentJointNames.put(RobotSide.LEFT, jointNamesLeft);
   }
}
