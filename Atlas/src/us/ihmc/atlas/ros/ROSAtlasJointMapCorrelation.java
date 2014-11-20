package us.ihmc.atlas.ros;

public class ROSAtlasJointMapCorrelation extends AtlasOrderedJointMap 
{
   //This class identifies joint angle correlation indices between left and right side joint angles
   // It also identifies whether same angles between different sides must change the sign.
   // Indices are based on ROSAtlasJointMap
   public final static int[] oppositeSideIndex = new int[AtlasOrderedJointMap.numberOfJoints];
   static
   {
      oppositeSideIndex[back_bkz] = back_bkz;
      oppositeSideIndex[back_bky] = back_bky;
      oppositeSideIndex[back_bkx] = back_bkx;
      oppositeSideIndex[neck_ry]  = neck_ry;
      oppositeSideIndex[l_leg_hpz] = r_leg_hpz;
      oppositeSideIndex[l_leg_hpx] = r_leg_hpx;
      oppositeSideIndex[l_leg_hpy] = r_leg_hpy;
      oppositeSideIndex[l_leg_kny] = r_leg_kny;
      oppositeSideIndex[l_leg_aky] = r_leg_aky;
      oppositeSideIndex[l_leg_akx] = r_leg_akx;
      oppositeSideIndex[r_leg_hpz] = l_leg_hpz;
      oppositeSideIndex[r_leg_hpx] = l_leg_hpx;
      oppositeSideIndex[r_leg_hpy] = l_leg_hpy;
      oppositeSideIndex[r_leg_kny] = l_leg_kny;
      oppositeSideIndex[r_leg_aky] = l_leg_aky;
      oppositeSideIndex[r_leg_akx] = l_leg_akx;
      oppositeSideIndex[l_arm_shz] = r_arm_shz;
      oppositeSideIndex[l_arm_shx] = r_arm_shx;
      oppositeSideIndex[l_arm_ely] = r_arm_ely;
      oppositeSideIndex[l_arm_elx] = r_arm_elx;
      oppositeSideIndex[l_arm_wry] = r_arm_wry;
      oppositeSideIndex[l_arm_wrx] = r_arm_wrx;
      oppositeSideIndex[r_arm_shz] = l_arm_shz;
      oppositeSideIndex[r_arm_shx] = l_arm_shx;
      oppositeSideIndex[r_arm_ely] = l_arm_ely;
      oppositeSideIndex[r_arm_elx] = l_arm_elx;
      oppositeSideIndex[r_arm_wry] = l_arm_wry;
      oppositeSideIndex[r_arm_wrx] = l_arm_wrx;

   }

   public final static boolean[] symmetricSignChange = new boolean[AtlasOrderedJointMap.numberOfJoints];
   //If symetricSignChange[i]==true, then symmetric angles are opposite signs.
   static
   {
      symmetricSignChange[back_bkz] = true;
      symmetricSignChange[back_bky] = false;
      symmetricSignChange[back_bkx] = true;
      symmetricSignChange[neck_ry]  = false;
      symmetricSignChange[l_leg_hpz] = symmetricSignChange[r_leg_hpz] = true;
      symmetricSignChange[l_leg_hpx] = symmetricSignChange[r_leg_hpx] = true;
      symmetricSignChange[l_leg_hpy] = symmetricSignChange[r_leg_hpy] = false;
      symmetricSignChange[l_leg_kny] = symmetricSignChange[r_leg_kny] = false;
      symmetricSignChange[l_leg_aky] = symmetricSignChange[r_leg_aky] = false;
      symmetricSignChange[l_leg_akx] = symmetricSignChange[r_leg_akx] = true;
      symmetricSignChange[l_arm_shz] = symmetricSignChange[r_arm_shz] = false;
      symmetricSignChange[l_arm_shx] = symmetricSignChange[r_arm_shx] = true;
      symmetricSignChange[l_arm_ely] = symmetricSignChange[r_arm_ely] = false;
      symmetricSignChange[l_arm_elx] = symmetricSignChange[r_arm_elx] = true;
      symmetricSignChange[l_arm_wry] = symmetricSignChange[r_arm_wry] = false;
      symmetricSignChange[l_arm_wrx] = symmetricSignChange[r_arm_wrx] = true;
   }
}
