package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.robotics.robotSide.RobotSide;

public enum HumanoidArmPose
{   
   STAND_PREP, SMALL_CHICKEN_WINGS, LARGE_CHICKEN_WINGS, STRAIGHTEN_ELBOWS, SUPPINATE_ARMS_IN_A_LITTLE, ARMS_BACK, LARGER_CHICKEN_WINGS, KARATE_KID_KRANE_KICK,
   ARMS_OUT_EXTENDED, SUPPINATE_ARMS_IN_MORE, SUPPINATE_ARMS_IN_A_LOT, SUPER_CHICKEN_WINGS, FLYING, FLYING_SUPPINATE_IN, FLYING_SUPPINATE_OUT, FLYING_PALMS_UP,
   FLEX_UP,FLEX_DOWN,REACH_BACK, REACH_WAY_BACK, ARMS_03, REACH_FORWARD, REACH_WAY_FORWARD, ARM_STRAIGHT_DOWN, ARM_NINETY_ELBOW_DOWN, ARM_NINETY_ELBOW_FORWARD,
   ARM_NINETY_ELBOW_UP, ARM_FORTFIVE_ELBOW_UP, ARM_FORTFIVE_ELBOW_DOWN, ARM_OUT_TRICEP_EXERCISE, ARM_NINETY_ELBOW_DOWN2, ARM_NINETY_ELBOW_FORWARD2, ARM_NINETY_ELBOW_UP2, ARM_FORTFIVE_ELBOW_UP2, ARM_FORTFIVE_ELBOW_UP3, ARM_FORTFIVE_ELBOW_DOWN2, ARM_FORTFIVE_ELBOW_DOWN3, REACH_FAR_FORWARD, REACH_FAR_BACK;
 
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
   public double[] getArmJointAngles(RobotSide robotSide)
   {
      switch (this)
      {
         case STAND_PREP:
            return symmetricArmPose(0.3, halfPi - 0.4, 0.05, 1.7, robotSide);
         case SMALL_CHICKEN_WINGS:
            return symmetricArmPose(0.3, halfPi - 0.6, 0.05, 1.7, robotSide);
         case LARGE_CHICKEN_WINGS:
            return symmetricArmPose(0.3, halfPi - 0.8, 0.05, 1.7, robotSide);
         case STRAIGHTEN_ELBOWS:
            return symmetricArmPose(0.3, halfPi - 0.6, 0.05, 1.0, robotSide);
         case SUPPINATE_ARMS_IN_A_LITTLE:
            return symmetricArmPose(0.3, halfPi - 0.6, 0.2, 1.7, robotSide);
         case ARMS_BACK:
            return symmetricArmPose(0.6, halfPi - 0.4, 0.05, 1.7, robotSide);
         case LARGER_CHICKEN_WINGS:
            return symmetricArmPose(0.3, halfPi - 1.0, 0.05, 1.7, robotSide);
         case ARMS_OUT_EXTENDED:
            return symmetricArmPose(0.3, halfPi - 1.0, 0.05, 0.9, robotSide);
         case FLYING:
            return symmetricArmPose(0.3, halfPi - 1.2, 0.05, 0.4, robotSide);
         case FLYING_PALMS_UP:
            return symmetricArmPose(0.3, halfPi - 1.2, -1.0, 0.4, robotSide);
         case FLEX_UP:
            return symmetricArmPose(0.0,0.0,0.0,-2, robotSide);
         case FLEX_DOWN:
            return symmetricArmPose(0.0,0.0,0.0,-1.4, robotSide);
         case FLYING_SUPPINATE_IN:
            return symmetricArmPose(0.3, halfPi - 1.2, 1.0, 0.4, robotSide);
         case FLYING_SUPPINATE_OUT:
            return symmetricArmPose(0.3, halfPi - 1.2, -1.0, 0.4, robotSide);
         case KARATE_KID_KRANE_KICK:
            return symmetricArmPose(0.0,0.0,0.0,0.0, robotSide);
         case SUPPINATE_ARMS_IN_MORE:
            return symmetricArmPose(0.3, halfPi - 0.4, 0.3, 1.7, robotSide);
         case SUPPINATE_ARMS_IN_A_LOT:
            return symmetricArmPose(0.3, halfPi - 0.4, 0.5, 1.7, robotSide);
         case SUPER_CHICKEN_WINGS:
            return symmetricArmPose(0.3, halfPi - 1.3, 0.05, 1.7, robotSide);
            
         case REACH_BACK:
            return symmetricArmPose(1.0, halfPi - 0.4, 0.05, 1.7, robotSide);
         case REACH_WAY_BACK:
            return symmetricArmPose(1.0, halfPi - 0.4, 0.05, 0.4, robotSide);
         case ARMS_03:
            return symmetricArmPose(0.3, halfPi - 0.4, 0.3, 1.7, robotSide);
         case REACH_FORWARD:
            return symmetricArmPose(-0.6, halfPi - 0.4, 0.05, 1.7, robotSide);
         case REACH_WAY_FORWARD:
            return symmetricArmPose(-0.6, halfPi - 0.4, 0.05, 0.4, robotSide);
         case REACH_FAR_FORWARD:
            return symmetricArmPose(-0.8 * halfPi, halfPi - 0.4, 0.0, 0.0, robotSide);
         case REACH_FAR_BACK:
            return symmetricArmPose( 0.8 * halfPi, halfPi - 0.4, 0.0, 0.0, robotSide);
            
            
         case ARM_STRAIGHT_DOWN:
            return symmetricArmPose(0.0, 1.4, 0.0, 0.0, robotSide);

         case ARM_NINETY_ELBOW_DOWN:
            return symmetricArmPose(0.0, 0.0, halfPi, halfPi, robotSide);
         case ARM_NINETY_ELBOW_DOWN2:
            return symmetricArmPose(halfPi / 2.0, 0.0, halfPi / 2.0, halfPi, robotSide);
         case ARM_NINETY_ELBOW_FORWARD:
            return symmetricArmPose(0.0, 0.0, 0.0, halfPi, robotSide);
         case ARM_NINETY_ELBOW_FORWARD2:
            return symmetricArmPose(halfPi / 2.0, 0.0, -halfPi / 2.0, halfPi, robotSide);
         case ARM_NINETY_ELBOW_UP:
            return symmetricArmPose(0.0, 0.0, -halfPi, halfPi, robotSide);
         case ARM_NINETY_ELBOW_UP2:
            return symmetricArmPose(-halfPi / 2.0, 0.0, -halfPi / 2.0, halfPi, robotSide);
         case ARM_FORTFIVE_ELBOW_UP:
            return symmetricArmPose(0.0, 0.0, -halfPi / 2.0, halfPi, robotSide);
         case ARM_FORTFIVE_ELBOW_UP2:
            return symmetricArmPose(-halfPi /2.0, 0.0, 0.0, halfPi, robotSide);
         case ARM_FORTFIVE_ELBOW_UP3:
            return symmetricArmPose(halfPi /2.0, 0.0, -halfPi, halfPi, robotSide);
         case ARM_FORTFIVE_ELBOW_DOWN:
            return symmetricArmPose(0.0, 0.0, 0.6, halfPi, robotSide);
         case ARM_FORTFIVE_ELBOW_DOWN2:
            return symmetricArmPose(halfPi/ 2.0, 0.0, 0.0, halfPi, robotSide);
         case ARM_FORTFIVE_ELBOW_DOWN3:
            return symmetricArmPose(-halfPi/ 2.0, 0.0, halfPi, halfPi, robotSide);
            
         case ARM_OUT_TRICEP_EXERCISE:
            return symmetricArmPose(0.0, 0.0, 1.4, 0.05, robotSide);
            
                     
         default:
            throw new RuntimeException("Shouldn't get here!");
      }
   }


   private double[] symmetricArmPose(double d0, double d1, double d2, double d3, RobotSide robotSide)
   {
      return new double[]{d0, robotSide.negateIfLeftSide(d1), d2, robotSide.negateIfLeftSide(d3)};
   }


   public double getDesiredElbowAngle(RobotSide robotSide)
   {
      return getArmJointAngles(robotSide)[3];
   }

   /**
    * Get the orientation of the upper-arm w.r.t. to the chest. By using IK solver as in DiagnosticBehavior this is robot agnostic whereas the getArmJointAngles() is Valkyrie specific. 
    * @return
    */
   public double[] getDesiredUpperArmYawPitchRoll()
   {
      switch (this)
      {
         case STAND_PREP: 
            return new double[]{-0.0485, 0.3191, -1.1856};
         case SMALL_CHICKEN_WINGS: 
            return new double[]{-0.0435, 0.3279, -0.9843};
         case LARGE_CHICKEN_WINGS: 
            return new double[]{-0.0368, 0.3356, -0.7824};
         case STRAIGHTEN_ELBOWS: 
            return new double[]{-0.0435, 0.3279, -0.9843};
         case SUPPINATE_ARMS_IN_A_LITTLE: 
            return new double[]{-0.1795, 0.4080, -1.0333};
         case ARMS_BACK: 
            return new double[]{-0.0565, 0.6187, -1.2032};
         case LARGER_CHICKEN_WINGS: 
            return new double[]{-0.0286, 0.3419, -0.5799};   
         case ARMS_OUT_EXTENDED: 
            return new double[]{-0.0286, 0.3419, -0.5799};   
         case FLYING: 
            return new double[]{-0.0192, 0.3465, -0.3770};
         case FLYING_PALMS_UP: 
            return new double[]{0.4636, -1.5708, -1.1071};
         case KARATE_KID_KRANE_KICK: 
            return new double[]{0.6154, 0.5235, 0.9553};
         case FLEX_UP: 
            return new double[]{0.4636, -1.5708, -1.1071};
         case FLEX_DOWN: 
            return new double[]{-0.8500, 0.0554, -0.5854};
         case FLYING_SUPPINATE_IN: 
            return new double[]{-0.8201, 1.1406, -0.9796};
         case FLYING_SUPPINATE_OUT: 
            return new double[]{0.3871, -0.6305, -0.4430};
         case SUPPINATE_ARMS_IN_MORE: 
            return new double[]{-0.3004, 0.4030, -1.2751};
         case SUPPINATE_ARMS_IN_A_LOT: 
            return new double[]{-0.5133, 0.4530, -1.3638};
         case SUPER_CHICKEN_WINGS: 
            return new double[]{-0.0142, 0.3481, -0.2754};
            
         case REACH_BACK: 
            return new double[]{-0.0877, 1.0177, -1.2451};
         case REACH_WAY_BACK: 
            return new double[]{-0.0877, 1.0177, -1.2451};
         case ARMS_03: 
            return new double[]{-0.3004, 0.4030, -1.2751};
         case REACH_FORWARD: 
            return new double[]{-0.0550, -0.5798, -1.1402};
         case REACH_WAY_FORWARD: 
            return new double[]{-0.0550, -0.5798, -1.1402};
         case REACH_FAR_FORWARD: 
            return new double[]{-0.0000, -1.2566, -1.1708};
         case REACH_FAR_BACK: 
            return new double[]{0.0000, 1.2566, -1.1708};
            
            
         case ARM_STRAIGHT_DOWN: 
            return new double[]{0.0000, -0.0000, -1.0708};

         case ARM_NINETY_ELBOW_DOWN: 
            return new double[]{-0.4636, 1.5708, -1.1071};
         case ARM_NINETY_ELBOW_DOWN2: 
            return new double[]{1.5708, 1.5708, 1.1071};
         case ARM_NINETY_ELBOW_FORWARD: 
            return new double[]{0.0000, -0.0000, -0.0000};
         case ARM_NINETY_ELBOW_FORWARD2: 
            return new double[]{0.0000, -0.0000, -0.0000};
         case ARM_NINETY_ELBOW_UP: 
            return new double[]{0.4636, -1.5708, -1.1071};
         case ARM_NINETY_ELBOW_UP2: 
            return new double[]{-1.5708, -1.5708, 1.1071};
         case ARM_FORTFIVE_ELBOW_UP: 
            return new double[]{0.0000, -0.7854, -0.0000};
         case ARM_FORTFIVE_ELBOW_UP2: 
            return new double[]{0.0000, -0.7854, -0.0000};
         case ARM_FORTFIVE_ELBOW_UP3: 
            return new double[]{0.0000, -0.7854, -0.0000};
         case ARM_FORTFIVE_ELBOW_DOWN: 
            return new double[]{-0.0000, 0.6000, -0.0000};
         case ARM_FORTFIVE_ELBOW_DOWN2: 
            return new double[]{0.0000, 0.7854, -0.0000};
         case ARM_FORTFIVE_ELBOW_DOWN3: 
            return new double[]{0.0000, 0.7854, -0.0000};
            
         case ARM_OUT_TRICEP_EXERCISE: 
            return new double[]{-0.7780, 1.3298, -0.7928};
            
                     
         default:
            throw new RuntimeException("Shouldn't get here!");
      }
   }
   
   /**
    * Get the orientation of the hand w.r.t. to the upper arm.
    * @return
    */
   public double[] getDesiredHandYawPitchRoll()
   {
      switch (this)
      {
      case STAND_PREP:
         return new double[]{0.0, Math.PI/2.0, 0.0};
      case REACH_BACK:
         return new double[]{0.0, Math.PI/2.0, 0.0};
      case REACH_WAY_BACK:
         return new double[]{0.0, Math.PI/2.0, 0.0};
      case ARMS_03:
         return new double[]{0.0, Math.PI, 0.0};
      case REACH_FORWARD:
         return new double[]{0.0, 0.0, 0.0};
      case SMALL_CHICKEN_WINGS:
         return new double[]{0.0, Math.PI/2.0, 0.0};
      case LARGE_CHICKEN_WINGS:
         return new double[]{0.0, Math.PI/2.0, 0.0};
      case STRAIGHTEN_ELBOWS:
         return new double[]{0.0, 0.0, 0.0};
      case SUPPINATE_ARMS_IN_A_LITTLE:
         return new double[]{0.0, 0.0, 0.0};
      case ARMS_BACK:
         return new double[]{0.0, 0.0, 0.0};
      case LARGER_CHICKEN_WINGS:
         return new double[]{0.0, Math.PI/2.0, 0.0};
      case ARMS_OUT_EXTENDED:
         return new double[]{0.0, 0.0, 0.0};
      case SUPPINATE_ARMS_IN_MORE:
         return new double[]{0.0, 0.0, 0.0};
      case SUPPINATE_ARMS_IN_A_LOT:
         return new double[]{0.0, 0.0, 0.0};
      case SUPER_CHICKEN_WINGS:
         return new double[]{0.0, Math.PI/2.0, 0.0};
      case FLYING:
         return new double[]{0.0,  Math.PI/2.0, 0.0};
      case FLYING_PALMS_UP:
         return new double[]{0.0, 0.0, 0.0};
      case KARATE_KID_KRANE_KICK:
         return new double[]{0.0,0.0,0.0};
      case FLEX_UP:
         return new double[]{0.0,0.0,0.0};
      case FLEX_DOWN:
         return new double[]{0.0,0.0,0.0};
      case FLYING_SUPPINATE_IN:
         return new double[]{0.0, Math.PI/2.0, 0.0};
      case FLYING_SUPPINATE_OUT:
         return new double[]{0.0, -Math.PI/2.0, 0.0};
      case ARM_NINETY_ELBOW_DOWN:
         return new double[]{0.0, 0.0, 0.0};
      case ARM_NINETY_ELBOW_FORWARD:
         return new double[]{0.0, 0.0, 0.0};
      case ARM_NINETY_ELBOW_UP:
         return new double[]{0.0, 0.0, 0.0};
      case ARM_FORTFIVE_ELBOW_UP:
         return new double[]{0.0, 0.0, 0.0};
      case ARM_FORTFIVE_ELBOW_DOWN:
         return new double[]{0.0, 0.0, 0.0};
      case ARM_OUT_TRICEP_EXERCISE:
         return new double[]{0.0, 0.0, 0.0};
      case ARM_STRAIGHT_DOWN:
         return new double[]{0.0, Math.PI / 2.0, 0.0};
         
      default:
         return new double[]{0.0, 0.0, 0.0};
      }
   }
}
