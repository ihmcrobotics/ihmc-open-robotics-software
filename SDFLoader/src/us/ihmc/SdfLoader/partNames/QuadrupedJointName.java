package us.ihmc.SdfLoader.partNames;

import us.ihmc.robotics.robotSide.RobotQuadrant;

public enum QuadrupedJointName
{
   /* Neck joints */
   PROXIMAL_NECK_YAW,
   PROXIMAL_NECK_PITCH,
   PROXIMAL_NECK_ROLL,
   DISTAL_NECK_YAW,
   DISTAL_NECK_PITCH,
   DISTAL_NECK_ROLL,

   /* Leg joints */
   FRONT_LEFT_HIP_ROLL,
   FRONT_LEFT_HIP_PITCH,
   FRONT_LEFT_KNEE_PITCH,
   FRONT_LEFT_ANKLE_PITCH,
   FRONT_RIGHT_HIP_ROLL,
   FRONT_RIGHT_HIP_PITCH,
   FRONT_RIGHT_KNEE_PITCH,
   FRONT_RIGHT_ANKLE_PITCH,
   HIND_LEFT_HIP_ROLL,
   HIND_LEFT_HIP_PITCH,
   HIND_LEFT_KNEE_PITCH,
   HIND_LEFT_ANKLE_PITCH,
   HIND_RIGHT_HIP_ROLL,
   HIND_RIGHT_HIP_PITCH,
   HIND_RIGHT_KNEE_PITCH,
   HIND_RIGHT_ANKLE_PITCH;

   public JointRole getRole()
   {
      switch (this)
      {
      case PROXIMAL_NECK_YAW:
      case PROXIMAL_NECK_PITCH:
      case PROXIMAL_NECK_ROLL:
      case DISTAL_NECK_YAW:
      case DISTAL_NECK_PITCH:
      case DISTAL_NECK_ROLL:
         return JointRole.NECK;
      case FRONT_LEFT_HIP_ROLL:
      case FRONT_LEFT_HIP_PITCH:
      case FRONT_LEFT_KNEE_PITCH:
      case FRONT_LEFT_ANKLE_PITCH:
      case FRONT_RIGHT_HIP_ROLL:
      case FRONT_RIGHT_HIP_PITCH:
      case FRONT_RIGHT_KNEE_PITCH:
      case FRONT_RIGHT_ANKLE_PITCH:
      case HIND_LEFT_HIP_ROLL:
      case HIND_LEFT_HIP_PITCH:
      case HIND_LEFT_KNEE_PITCH:
      case HIND_LEFT_ANKLE_PITCH:
      case HIND_RIGHT_HIP_ROLL:
      case HIND_RIGHT_HIP_PITCH:
      case HIND_RIGHT_KNEE_PITCH:
      case HIND_RIGHT_ANKLE_PITCH:
         return JointRole.LEG;
      }

      // Should never get here
      throw new IllegalStateException("Invalid joint for role: " + this);
   }

   public RobotQuadrant getQuadrant()
   {
      switch (this)
      {
      case PROXIMAL_NECK_YAW:
      case PROXIMAL_NECK_PITCH:
      case PROXIMAL_NECK_ROLL:
      case DISTAL_NECK_YAW:
      case DISTAL_NECK_PITCH:
      case DISTAL_NECK_ROLL:
         throw new IllegalArgumentException("Neck joints do not have a quadrant");
      case FRONT_LEFT_HIP_ROLL:
      case FRONT_LEFT_HIP_PITCH:
      case FRONT_LEFT_KNEE_PITCH:
      case FRONT_LEFT_ANKLE_PITCH:
         return RobotQuadrant.FRONT_LEFT;
      case FRONT_RIGHT_HIP_ROLL:
      case FRONT_RIGHT_HIP_PITCH:
      case FRONT_RIGHT_KNEE_PITCH:
      case FRONT_RIGHT_ANKLE_PITCH:
         return RobotQuadrant.FRONT_RIGHT;
      case HIND_LEFT_HIP_ROLL:
      case HIND_LEFT_HIP_PITCH:
      case HIND_LEFT_KNEE_PITCH:
      case HIND_LEFT_ANKLE_PITCH:
         return RobotQuadrant.HIND_LEFT;
      case HIND_RIGHT_HIP_ROLL:
      case HIND_RIGHT_HIP_PITCH:
      case HIND_RIGHT_KNEE_PITCH:
      case HIND_RIGHT_ANKLE_PITCH:
         return RobotQuadrant.HIND_RIGHT;
      }

      // Should never get here
      throw new IllegalStateException("Invalid joint for quadrant: " + this);
   }

   public static QuadrupedJointName getName(RobotQuadrant quadrant, LegJointName legJointName)
   {
      switch(quadrant)
      {
      case FRONT_LEFT:
         switch(legJointName)
         {
         case HIP_ROLL:
            return FRONT_LEFT_HIP_ROLL;
         case HIP_PITCH:
            return FRONT_LEFT_HIP_PITCH;
         case KNEE:
            return FRONT_LEFT_KNEE_PITCH;
         }
         break;
      case FRONT_RIGHT:
         switch(legJointName)
         {
         case HIP_ROLL:
            return FRONT_RIGHT_HIP_ROLL;
         case HIP_PITCH:
            return FRONT_RIGHT_HIP_PITCH;
         case KNEE:
            return FRONT_RIGHT_KNEE_PITCH;
         }
         break;
      case HIND_RIGHT:
         switch(legJointName)
         {
         case HIP_ROLL:
            return HIND_RIGHT_HIP_ROLL;
         case HIP_PITCH:
            return HIND_RIGHT_HIP_PITCH;
         case KNEE:
            return HIND_RIGHT_KNEE_PITCH;
         }
         break;
      case HIND_LEFT:
         switch(legJointName)
         {
         case HIP_ROLL:
            return HIND_LEFT_HIP_ROLL;
         case HIP_PITCH:
            return HIND_LEFT_HIP_PITCH;
         case KNEE:
            return HIND_LEFT_KNEE_PITCH;
         }
         break;
      }

      throw new IllegalArgumentException("Leg joint does not exist: " + quadrant + " " + legJointName);
   }
}
