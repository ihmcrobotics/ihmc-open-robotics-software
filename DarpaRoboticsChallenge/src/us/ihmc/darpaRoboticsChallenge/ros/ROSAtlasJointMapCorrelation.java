package us.ihmc.darpaRoboticsChallenge.ros;

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

   public final static boolean[] symmetricSignChange = new boolean[ROSAtlasJointMap.numberOfJoints];
   //If symetricSignChange[i]==true, then symmetric angles are opposite signs.
   static
   {
      symmetricSignChange[back_lbz] = true;
      symmetricSignChange[back_mby] = false;
      symmetricSignChange[back_ubx] = true;
      symmetricSignChange[neck_ay]  = false;
      symmetricSignChange[l_leg_uhz] = symmetricSignChange[r_leg_uhz] = true;
      symmetricSignChange[l_leg_mhx] = symmetricSignChange[r_leg_mhx] = true;
      symmetricSignChange[l_leg_lhy] = symmetricSignChange[r_leg_lhy] = false;
      symmetricSignChange[l_leg_kny] = symmetricSignChange[r_leg_kny] = false;
      symmetricSignChange[l_leg_uay] = symmetricSignChange[r_leg_uay] = false;
      symmetricSignChange[l_leg_lax] = symmetricSignChange[r_leg_lax] = true;
      symmetricSignChange[l_arm_usy] = symmetricSignChange[r_arm_usy] = false;
      symmetricSignChange[l_arm_shx] = symmetricSignChange[r_arm_shx] = true;
      symmetricSignChange[l_arm_ely] = symmetricSignChange[r_arm_ely] = false;
      symmetricSignChange[l_arm_elx] = symmetricSignChange[r_arm_elx] = true;
      symmetricSignChange[l_arm_uwy] = symmetricSignChange[r_arm_uwy] = false;
      symmetricSignChange[l_arm_mwx] = symmetricSignChange[r_arm_mwx] = true;
   }
}
