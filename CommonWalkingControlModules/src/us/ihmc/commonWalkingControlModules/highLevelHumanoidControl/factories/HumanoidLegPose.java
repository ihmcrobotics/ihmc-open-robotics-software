package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

public enum HumanoidLegPose
{
   STAND_PREP, LEG_TWO, LEG_THREE, LEG_THREE_B, LEG_THREE_C, LEG_THREE_D, LEG_FOUR, LEG_FOUR_B, LEG_FOUR_C, LEG_FOUR_D, LEG_FIVE, LEG_SIX, 
   LEG_NINE, LEG_TEN, LEG_ELEVEN, LEG_TWELVE, LEG_THIRTEEN, LEG_FOURTEEN, LEG_FIFTEEN, LEG_SIXTEEN, LEG_SEVENTEEN, 
   LEG_EIGHTEEN, LEG_NINTEEN, LEG_TWENTY, LEG_TWENTY_B;
 
   
   public double[] getLegJointAngles()
   {
      switch (this)
      {
         case STAND_PREP:
            return new double[]{0.0, 0.0, -0.25, -0.7, 0.0, 0.0};
         case LEG_TWO:
            return new double[]{0.0, 0.0, 0.0, -0.7, 0.0, 0.0};
         case LEG_THREE:
            return new double[]{0.0, 0.0, 0.8, -0.7, 0.0, 0.0};
         case LEG_THREE_B:
            return new double[]{0.0, 0.0, 1.0, -0.5, 0.0, 0.0};
         case LEG_THREE_C:
            return new double[]{0.0, 0.0, 1.0, -1.0, 0.0, 0.0};
         case LEG_THREE_D:
            return new double[]{0.0, 0.0, 1.0, -1.5, 0.0, 0.0};
         case LEG_FOUR:
            return new double[]{0.0, 0.0, -0.5, -0.8, 0.0, 0.0};
         case LEG_FOUR_B:
            return new double[]{0.0, 0.0, -1.5, -0.3, 0.0, 0.0};
         case LEG_FOUR_C:
            return new double[]{0.0, 0.0, -1.5, -1.0, 0.0, 0.0};
         case LEG_FOUR_D:
            return new double[]{0.0, 0.0, -1.5, -1.3, 0.0, 0.0};
         case LEG_FIVE:
            return new double[]{0.0, -0.2, -0.5, -0.8, 0.0, 0.0};
         case LEG_SIX:
            return new double[]{-0.15, -0.2, -0.5, -0.8, 0.0, 0.0};
         case LEG_NINE:
           return new double[]{0.0, -0.2, -0.5, -1.7, 0.0, 0.0};
         case LEG_TEN:
           return new double[]{0.0, -0.2, -0.5, -0.7, 0.0, 0.0};
         case LEG_ELEVEN:
           return new double[]{0.0, -0.2, -0.5, -1.0, 0.0, 0.0};
         case LEG_TWELVE:
           return new double[]{0.0, -0.2, -0.5, -0.7, 0.0, 0.0};
         case LEG_THIRTEEN:
           return new double[]{0.0, -0.2, -0.5, -1.7, 0.0, 0.0};
         case LEG_FOURTEEN:
           return new double[]{0.0, -0.2, 0.0, -0.7, 0.0, 0.0};
         case LEG_FIFTEEN:
           return new double[]{0.0, -0.3, 0.0, -0.7, 0.0, 0.0};
         case LEG_SIXTEEN:
           return new double[]{0.0, -0.1, 0.0, -0.7, 0.0, 0.0};
         case LEG_SEVENTEEN:
            return new double[]{0.0, -0.1, 0.0, -0.7, 0.0, 0.0};
         case LEG_EIGHTEEN:
            return new double[]{0.0, -0.1, 0.0, -0.7, 0.0, 0.0};
         case LEG_NINTEEN:
            return new double[]{0.0, -0.1, 0.0, -0.7, 0.0, 0.0};
         case LEG_TWENTY:
            return new double[]{0.0, 0.3, 0.0, -0.7, 0.0, 0.0};
         case LEG_TWENTY_B:
            return new double[]{0.0, -0.3, 0.0, -0.7, 0.0, 0.0};


         default:
            throw new RuntimeException("Shouldn't get here!");
      }
   }
}

