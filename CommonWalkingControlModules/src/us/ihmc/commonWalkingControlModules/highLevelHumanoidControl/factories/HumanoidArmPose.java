package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

public enum HumanoidArmPose
{
   STAND_PREP, SMALL_CHICKEN_WINGS, LARGE_CHICKEN_WINGS, STRAIGHTEN_ELBOWS, SUPPINATE_ARMS, ARM_FOUR, ARM_FOUR_B, ARM_FOUR_C, ARM_FOUR_D, ARM_FIVE, ARM_SIX, ARM_NINE, ARM_TEN, ELEVEN, TWELVE, ARM_THIRTEEN;
 
   
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
         case SUPPINATE_ARMS:
            return new double[]{0.3, -0.6, 0.2, -1.7};
            
            
         case ARM_FOUR:
            return new double[]{0.6, -0.4, 0.05, -1.7};

         case ARM_FOUR_B:
            return new double[]{0.3, -0.4, 0.05, -1.7};

         case ARM_FOUR_C:
            return new double[]{0.3, -1.0, 0.05, -1.7};

            
         case ARM_FOUR_D:
            return new double[]{0.3, -1.0, 0.05, -0.5};

            
         case ARM_FIVE:
            return new double[]{0.3, -1.0, 0.05, -1.7};

         case ARM_SIX:
            return new double[]{0.3, -0.4, 0.3, -1.7};

            
         case ARM_NINE:
            return new double[]{0.3, -0.4, 0.05, -1.7};

         case ARM_TEN:
            return new double[]{0.3, -0.4, 0.5, -1.7};

            
         case ELEVEN:
            return new double[]{0.3, -1.3, 0.05, -1.7};

         case TWELVE:
            return new double[]{0.3, -0.4, 0.5, -1.7};

         case ARM_THIRTEEN:
           return new double[]{0.3, -0.4, 0.05, -1.7};

            
         default:
            throw new RuntimeException("Shouldn't get here!");
      }
   }
}
