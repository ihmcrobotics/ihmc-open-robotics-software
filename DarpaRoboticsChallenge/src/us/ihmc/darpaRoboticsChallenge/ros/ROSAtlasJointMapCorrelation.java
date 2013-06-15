package us.ihmc.darpaRoboticsChallenge.ros;

import javax.swing.JPanel;

public class ROSAtlasJointMapCorrelation extends ROSAtlasJointMap 
{
   //This class identifies joint angle correlation indices between left and right side joint angles
   // It also identifies whether same angles between different sides must change the sign.
   // Indices are based on ROSAtlasJointMap
   public final static int[] oppositeSideIndex = new int[ROSAtlasJointMap.numberOfJoints];
   static
   {
      oppositeSideIndex[back_lbz] = back_lbz;
      oppositeSideIndex[back_mby] = back_mby;
      oppositeSideIndex[back_ubx] = back_ubx;
      oppositeSideIndex[neck_ay]  = neck_ay;
      oppositeSideIndex[l_leg_uhz] = r_leg_uhz;
      oppositeSideIndex[l_leg_mhx] = r_leg_mhx;
      oppositeSideIndex[l_leg_lhy] = r_leg_lhy;
      oppositeSideIndex[l_leg_kny] = r_leg_kny;
      oppositeSideIndex[l_leg_uay] = r_leg_uay;
      oppositeSideIndex[l_leg_lax] = r_leg_lax;
      oppositeSideIndex[r_leg_uhz] = l_leg_uhz;
      oppositeSideIndex[r_leg_mhx] = l_leg_mhx;
      oppositeSideIndex[r_leg_lhy] = l_leg_lhy;
      oppositeSideIndex[r_leg_kny] = l_leg_kny;
      oppositeSideIndex[r_leg_uay] = l_leg_uay;
      oppositeSideIndex[r_leg_lax] = l_leg_lax;
      oppositeSideIndex[l_arm_usy] = r_arm_usy;
      oppositeSideIndex[l_arm_shx] = r_arm_shx;
      oppositeSideIndex[l_arm_ely] = r_arm_ely;
      oppositeSideIndex[l_arm_elx] = r_arm_elx;
      oppositeSideIndex[l_arm_uwy] = r_arm_uwy;
      oppositeSideIndex[l_arm_mwx] = r_arm_mwx;
      oppositeSideIndex[r_arm_usy] = l_arm_usy;
      oppositeSideIndex[r_arm_shx] = l_arm_shx;
      oppositeSideIndex[r_arm_ely] = l_arm_ely;
      oppositeSideIndex[r_arm_elx] = l_arm_elx;
      oppositeSideIndex[r_arm_uwy] = l_arm_uwy;
      oppositeSideIndex[r_arm_mwx] = l_arm_mwx;

   }

   public final static boolean[] symetricSignChange = new boolean[ROSAtlasJointMap.numberOfJoints];
   //If symetricSignChange[i]==true, then symmetric angles are opposite signs.
   static
   {
      symetricSignChange[back_lbz] = true;
      symetricSignChange[back_mby] = false;
      symetricSignChange[back_ubx] = true;
      symetricSignChange[neck_ay]  = false;
      symetricSignChange[l_leg_uhz] = symetricSignChange[r_leg_uhz] = true;
      symetricSignChange[l_leg_mhx] = symetricSignChange[r_leg_mhx] = true;
      symetricSignChange[l_leg_lhy] = symetricSignChange[r_leg_lhy] = false;
      symetricSignChange[l_leg_kny] = symetricSignChange[r_leg_kny] = false;
      symetricSignChange[l_leg_uay] = symetricSignChange[r_leg_uay] = false;
      symetricSignChange[l_leg_lax] = symetricSignChange[r_leg_lax] = true;
      symetricSignChange[l_arm_usy] = symetricSignChange[r_arm_usy] = false;
      symetricSignChange[l_arm_shx] = symetricSignChange[r_arm_shx] = true;
      symetricSignChange[l_arm_ely] = symetricSignChange[r_arm_ely] = false;
      symetricSignChange[l_arm_elx] = symetricSignChange[r_arm_elx] = true;
      symetricSignChange[l_arm_uwy] = symetricSignChange[r_arm_uwy] = false;
      symetricSignChange[l_arm_mwx] = symetricSignChange[r_arm_mwx] = true;
   }
}
