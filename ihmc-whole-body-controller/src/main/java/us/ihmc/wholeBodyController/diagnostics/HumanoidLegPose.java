package us.ihmc.wholeBodyController.diagnostics;

import us.ihmc.robotics.robotSide.RobotSide;

public enum HumanoidLegPose
{
   STAND_PREP,
   ZERO,
   THIGHS_BACK_AND_STRAIGHT_A_LITTLE,
   THIGHS_BACK_AND_STRAIGHT_MORE,
   THIGHS_BACK_AND_STRAIGHT_A_LOT,
   SUPERMAN,
   SUPERMAN_BENT_KNEES,
   THIGHS_FORWARD_A_LITTLE_SLIGHLY_BENT_KNEES,
   THIGHS_UP_STRAIGHT_KNEES,
   THIGHS_UP_BENT_KNEES,
   THIGHS_UP_BENT_KNEES_MORE,
   STAND_PREP_HIPS_OUT_A_BIT,
   HIPS_OUT_A_BIT_ROTATED_OUT,
   LEGS_STRAIGHT_KNEES_FULLY_BENT,
   STAND_PREP_LEGS_OUT_AND_FORWARD,
   LEGS_OUT_FORWARD_WITH_BENT_KNEES,
   RELAXED_SLIGHTLY_BENT_KNEES,
   HIPS_OUT_MORE_SLIGHTLY_BENT_KNEES,
   STAND_PREP_HIPS_OUT_A_LITTLE,
   HIPS_IN_A_LOT;

   /**
    * Leg angles are as follows:
    * 
    * 1 - hip rotator (negative out / positive in) min: -0.15 max: 0.0
    * 2 - hip adductor (negative out / positive in) min: -0.3 max: 0.06
    * 3 - hip extensor (negative forward / positive back) min: -1.5 max: 1.0
    * 4 - knee extensor (negative back / positive forward) min: -1.7 max: -0.3
    * 5 - ankle extensor (negative up / positive down) min: 0.0 max: 0.0
    * 6 - ankle adductor (negative in / positive out) min: 0.0 max: 0.0
    * 
    * @return legAngles
    */
   public double[] getLegJointAngles(RobotSide robotSide)
   {
      switch (this)
      {
      case STAND_PREP:
         return new double[] {0.0, 0.0, -0.25, 0.7, 0.0, 0.0};
      case ZERO:
         return new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      case THIGHS_BACK_AND_STRAIGHT_A_LITTLE:
         return new double[] {0.0, 0.0, 0.0, 0.7, 0.0, 0.0};
      case THIGHS_BACK_AND_STRAIGHT_MORE:
         return new double[] {0.0, 0.0, 0.8, 0.7, 0.0, 0.0};
      case THIGHS_BACK_AND_STRAIGHT_A_LOT:
         return new double[] {0.0, 0.0, 1.0, 0.5, 0.0, 0.0};
      case SUPERMAN:
         return new double[] {0.0, 0.0, 1.0, 1.0, 0.0, 0.0};
      case SUPERMAN_BENT_KNEES:
         return new double[] {0.0, 0.0, 1.0, 1.5, 0.0, 0.0};
      case THIGHS_FORWARD_A_LITTLE_SLIGHLY_BENT_KNEES:
         return new double[] {0.0, 0.0, -0.5, 0.8, 0.0, 0.0};
      case THIGHS_UP_STRAIGHT_KNEES:
         return new double[] {0.0, 0.0, -1.5, 0.3, 0.0, 0.0};
      case THIGHS_UP_BENT_KNEES:
         return new double[] {0.0, 0.0, -1.5, 1.0, 0.0, 0.0};
      case THIGHS_UP_BENT_KNEES_MORE:
         return new double[] {0.0, 0.0, -1.5, 1.3, 0.0, 0.0};
      case STAND_PREP_HIPS_OUT_A_BIT:
         return new double[] {0.0, robotSide.negateIfLeftSide(-0.2), -0.5, 0.8, 0.0, 0.0};
      case HIPS_OUT_A_BIT_ROTATED_OUT:
         return new double[] {-0.15, robotSide.negateIfLeftSide(-0.2), -0.5, 0.8, 0.0, 0.0};
      case LEGS_STRAIGHT_KNEES_FULLY_BENT:
         return new double[] {0.0, robotSide.negateIfLeftSide(-0.2), -0.5, 1.5, 0.0, 0.0};
      case STAND_PREP_LEGS_OUT_AND_FORWARD:
         return new double[] {0.0, robotSide.negateIfLeftSide(-0.2), -0.5, 0.7, 0.0, 0.0};
      case LEGS_OUT_FORWARD_WITH_BENT_KNEES:
         return new double[] {0.0, robotSide.negateIfLeftSide(-0.2), -0.5, 1.0, 0.0, 0.0};
      case RELAXED_SLIGHTLY_BENT_KNEES:
         return new double[] {0.0, robotSide.negateIfLeftSide(-0.2), 0.0, 0.7, 0.0, 0.0};
      case HIPS_OUT_MORE_SLIGHTLY_BENT_KNEES:
         return new double[] {0.0, robotSide.negateIfLeftSide(-0.3), 0.0, 0.7, 0.0, 0.0};
      case STAND_PREP_HIPS_OUT_A_LITTLE:
         return new double[] {0.0, robotSide.negateIfLeftSide(-0.1), 0.0, 0.7, 0.0, 0.0};
      case HIPS_IN_A_LOT:
         return new double[] {0.0, robotSide.negateIfLeftSide(0.06), 0.0, 0.7, 0.0, 0.0};

      default:
         throw new RuntimeException("Shouldn't get here!");
      }
   }
}
