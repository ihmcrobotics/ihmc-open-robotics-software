package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

public enum HumanoidArmPose
{   
   STAND_PREP, SMALL_CHICKEN_WINGS, LARGE_CHICKEN_WINGS, STRAIGHTEN_ELBOWS, SUPPINATE_ARMS_IN_A_LITTLE, ARMS_BACK, LARGER_CHICKEN_WINGS,
   ARMS_OUT_EXTENDED, SUPPINATE_ARMS_IN_MORE, SUPPINATE_ARMS_IN_A_LOT, SUPER_CHICKEN_WINGS, FLYING, FLYING_SUPPINATE_IN, FLYING_SUPPINATE_OUT,
   REACH_BACK, REACH_WAY_BACK, ARMS_03, REACH_FORWARD, REACH_WAY_FORWARD, ARM_STRAIGHT_DOWN, ARM_NINETY_ELBOW_DOWN, ARM_NINETY_ELBOW_FORWARD,
   ARM_NINETY_ELBOW_UP, ARM_FORTFIVE_ELBOW_UP, ARM_FORTFIVE_ELBOW_DOWN, ARM_OUT_TRICEP_EXERCISE, ARM_NINETY_ELBOW_DOWN2, ARM_NINETY_ELBOW_FORWARD2, ARM_NINETY_ELBOW_UP2, ARM_FORTFIVE_ELBOW_UP2, ARM_FORTFIVE_ELBOW_UP3, ARM_FORTFIVE_ELBOW_DOWN2, ARM_FORTFIVE_ELBOW_DOWN3;
 
   private static final double halfPi = Math.PI / 2.0;

   /**
    * Arm angles are as follows:
    * 
    * 1 - shoulder extensor (negative forward / positive back) min: 0.3 max: 0.6
    * 2 - shoulder adductor (negative out / positive in) min: -1.3 max: -0.4
    * 3 - elbow supinator (negative out / positive in) min: 0.05 max: 0.5
    * 4 - elbow extensor (negative up / positive down) min: -1.7 max: -0.9
    * 
    * @return armAngles
    */
   public double[] getArmJointAngles()
   {
      switch (this)
      {
         case STAND_PREP:
            return new double[]{0.3, -0.4, 0.05, -1.7};
         case SMALL_CHICKEN_WINGS:
            return new double[]{0.3, -0.6, 0.05, -1.7};
         case LARGE_CHICKEN_WINGS:
            return new double[]{0.3, -0.8, 0.05, -1.7};
         case STRAIGHTEN_ELBOWS:
            return new double[]{0.3, -0.6, 0.05, -1.0};
         case SUPPINATE_ARMS_IN_A_LITTLE:
            return new double[]{0.3, -0.6, 0.2, -1.7};
         case ARMS_BACK:
            return new double[]{0.6, -0.4, 0.05, -1.7};
         case LARGER_CHICKEN_WINGS:
            return new double[]{0.3, -1.0, 0.05, -1.7};
         case ARMS_OUT_EXTENDED:
            return new double[]{0.3, -1.0, 0.05, -0.9};
         case FLYING:
            return new double[]{0.3, -1.2, 0.05, -0.4};
         case FLYING_SUPPINATE_IN:
            return new double[]{0.3, -1.2, 1.0, -0.4};
         case FLYING_SUPPINATE_OUT:
            return new double[]{0.3, -1.2, -1.0, -0.4};
         case SUPPINATE_ARMS_IN_MORE:
            return new double[]{0.3, -0.4, 0.3, -1.7};
         case SUPPINATE_ARMS_IN_A_LOT:
            return new double[]{0.3, -0.4, 0.5, -1.7};
         case SUPER_CHICKEN_WINGS:
            return new double[]{0.3, -1.3, 0.05, -1.7};
            
         case REACH_BACK:
            return new double[]{1.0, -0.4, 0.05, -1.7};
         case REACH_WAY_BACK:
            return new double[]{1.0, -0.4, 0.05, -0.4};
         case ARMS_03:
            return new double[]{0.3, -0.4, 0.3, -1.7};
         case REACH_FORWARD:
            return new double[]{-0.6, -0.4, 0.05, -1.7};
         case REACH_WAY_FORWARD:
            return new double[]{-0.6, -0.4, 0.05, -0.4};
            
            
         case ARM_STRAIGHT_DOWN:
            return new double[]{0.0, -0.5, 1.45, -0.53};

         case ARM_NINETY_ELBOW_DOWN:
            return new double[]{0.0, -halfPi, halfPi, -halfPi};
         case ARM_NINETY_ELBOW_DOWN2:
            return new double[]{halfPi / 2.0, -halfPi, halfPi / 2.0, -halfPi};
         case ARM_NINETY_ELBOW_FORWARD:
            return new double[]{0.0, -halfPi, 0.0, -halfPi};
         case ARM_NINETY_ELBOW_FORWARD2:
            return new double[]{halfPi / 2.0, -halfPi, -halfPi / 2.0, -halfPi};
         case ARM_NINETY_ELBOW_UP:
            return new double[]{0.0, -halfPi, -halfPi, -halfPi};
         case ARM_NINETY_ELBOW_UP2:
            return new double[]{-halfPi / 2.0, -halfPi, -halfPi / 2.0, -halfPi};
         case ARM_FORTFIVE_ELBOW_UP:
            return new double[]{0.0, -halfPi, -halfPi / 2.0, -halfPi};
         case ARM_FORTFIVE_ELBOW_UP2:
            return new double[]{-halfPi /2.0, -halfPi, 0.0, -halfPi};
         case ARM_FORTFIVE_ELBOW_UP3:
            return new double[]{halfPi /2.0, -halfPi, -halfPi, -halfPi};
         case ARM_FORTFIVE_ELBOW_DOWN:
            return new double[]{0.0, -halfPi, 0.6, -halfPi};
         case ARM_FORTFIVE_ELBOW_DOWN2:
            return new double[]{halfPi/ 2.0, -halfPi, 0.0, -halfPi};
         case ARM_FORTFIVE_ELBOW_DOWN3:
            return new double[]{-halfPi/ 2.0, -halfPi, halfPi, -halfPi};
            
         case ARM_OUT_TRICEP_EXERCISE:
            return new double[]{0.0, -1.4, 1.4, 0.05};
            
                     
         default:
            throw new RuntimeException("Shouldn't get here!");
      }
   }
}
