package us.ihmc.wholeBodyController.diagnostics;

public enum HumanoidSpinePose
{
   STAND_PREP, ZERO, TURN_LEFT, TURN_RIGHT, LEAN_FORWARD_A_LOT, LEAN_BACKWARD_A_LOT, LEAN_LEFT, LEAN_RIGHT, LEAN_FORWARD, LEAN_BACKWARD;

   /**
    * Spine angles are as follows:
    * 
    * 1 - waist rotator (negative torso right / positive torso left) min: 0.0 max: 0.0 
    * 2 - waist extensor (negative forward / positive back) min: -0.06 max: 0.1
    * 3 - waist lateral extensor (negative lean left / positive lean right) min: -0.1  max: 0.1
    * 
    * @return armAngles
    */
   public double[] getSpineJointAngles()
   {
      switch (this)
      {
      case STAND_PREP:
         return new double[] {0.0, -0.06, 0.0};
      case ZERO:
         return new double[] {0.0, 0.0, 0.0};
      case TURN_LEFT:
         return new double[] {0.1, -0.06, 0.0};
      case TURN_RIGHT:
         return new double[] {-0.1, -0.06, 0.0};
      case LEAN_FORWARD_A_LOT:
         return new double[] {0.0, -0.1, 0.0};
      case LEAN_BACKWARD_A_LOT:
         return new double[] {0.0, 0.1, 0.0};
      case LEAN_FORWARD:
         return new double[] {0.0, -0.05, 0.0};
      case LEAN_BACKWARD:
         return new double[] {0.0, 0.05, 0.0};
      case LEAN_LEFT:
         return new double[] {0.0, -0.06, 0.1};
      case LEAN_RIGHT:
         return new double[] {0.0, -0.06, -0.1};

      default:
         throw new RuntimeException("Shouldn't get here!");
      }
   }
}
