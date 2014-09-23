package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

public enum HumanoidSpinePose
{
   STAND_PREP, FORWARD, BACKWARD, LEFT, RIGHT, SPINE_ONE, SPINE_TWO, SPINE_THREE;
 
   
   public double[] getSpineJointAngles()
   {
      switch (this)
      {
         case STAND_PREP:
            return new double[]{0.0, -0.06, 0.0};
         case FORWARD:
            return new double[]{0.0, -0.06, 0.0};
         case BACKWARD:
            return new double[]{0.0, -0.06, 0.0};
         case LEFT:
            return new double[]{0.0, -0.06, 0.0};
         case RIGHT:
            return new double[]{0.0, -0.06, 0.0};
            
         case SPINE_ONE:
            return new double[]{0.0, 0.1, 0.0};

            
         case SPINE_TWO:
            return new double[]{0.0, -0.06, 0.1};
            
         case SPINE_THREE:
            return new double[]{0.0, -0.06, -0.1};


         default:
            throw new RuntimeException("Shouldn't get here!");
      }
   }
}
