package us.ihmc.robotics.partNames;

import us.ihmc.robotics.robotSide.RobotQuadrant;

public enum QuadrupedJointName
{
   /* Neck joints */
   PROXIMAL_NECK_YAW(NeckJointName.PROXIMAL_NECK_YAW),
   PROXIMAL_NECK_PITCH(NeckJointName.PROXIMAL_NECK_PITCH),
   PROXIMAL_NECK_ROLL(NeckJointName.PROXIMAL_NECK_ROLL),
   DISTAL_NECK_YAW(NeckJointName.DISTAL_NECK_YAW),
   DISTAL_NECK_PITCH(NeckJointName.DISTAL_NECK_PITCH),
   DISTAL_NECK_ROLL(NeckJointName.DISTAL_NECK_ROLL),

   /* Leg joints */
   FRONT_LEFT_HIP_ROLL(RobotQuadrant.FRONT_LEFT, LegJointName.HIP_ROLL),
   FRONT_LEFT_HIP_PITCH(RobotQuadrant.FRONT_LEFT, LegJointName.HIP_PITCH),
   FRONT_LEFT_KNEE_PITCH(RobotQuadrant.FRONT_LEFT, LegJointName.KNEE_PITCH),
   FRONT_LEFT_ANKLE_PITCH(RobotQuadrant.FRONT_LEFT, LegJointName.ANKLE_PITCH),
   FRONT_RIGHT_HIP_ROLL(RobotQuadrant.FRONT_RIGHT, LegJointName.HIP_ROLL),
   FRONT_RIGHT_HIP_PITCH(RobotQuadrant.FRONT_RIGHT, LegJointName.HIP_PITCH),
   FRONT_RIGHT_KNEE_PITCH(RobotQuadrant.FRONT_RIGHT, LegJointName.KNEE_PITCH),
   FRONT_RIGHT_ANKLE_PITCH(RobotQuadrant.FRONT_RIGHT, LegJointName.ANKLE_PITCH),
   HIND_RIGHT_HIP_ROLL(RobotQuadrant.HIND_RIGHT, LegJointName.HIP_ROLL),
   HIND_RIGHT_HIP_PITCH(RobotQuadrant.HIND_RIGHT, LegJointName.HIP_PITCH),
   HIND_RIGHT_KNEE_PITCH(RobotQuadrant.HIND_RIGHT, LegJointName.KNEE_PITCH),
   HIND_RIGHT_ANKLE_PITCH(RobotQuadrant.HIND_RIGHT, LegJointName.ANKLE_PITCH),
   HIND_LEFT_HIP_ROLL(RobotQuadrant.HIND_LEFT, LegJointName.HIP_ROLL),
   HIND_LEFT_HIP_PITCH(RobotQuadrant.HIND_LEFT, LegJointName.HIP_PITCH),
   HIND_LEFT_KNEE_PITCH(RobotQuadrant.HIND_LEFT, LegJointName.KNEE_PITCH),
   HIND_LEFT_ANKLE_PITCH(RobotQuadrant.HIND_LEFT, LegJointName.ANKLE_PITCH),
   
   ;
   
   public static final QuadrupedJointName[] values = values();
   
   private final JointRole jointRole;
   private final NeckJointName neckJointName;
   private final RobotQuadrant robotQuadrant;
   private final LegJointName legJointName;
   
   private QuadrupedJointName(NeckJointName neckJointName)
   {
      this.jointRole = JointRole.NECK;
      this.neckJointName = neckJointName;
      this.robotQuadrant = null;
      this.legJointName = null;
   }
   
   private QuadrupedJointName(RobotQuadrant robotQuadrant, LegJointName legJointName)
   {
      this.jointRole = JointRole.LEG;
      this.robotQuadrant = robotQuadrant;
      this.legJointName = legJointName;
      this.neckJointName = null;
   }
   
   public JointRole getRole()
   {
      return jointRole;
   }

   public RobotQuadrant getQuadrant()
   {
      if (jointRole.equals(JointRole.NECK))
      {
         throw new IllegalArgumentException("Neck joints do not have a quadrant");
      }
      else
      {
         return robotQuadrant;
      }
   }

   @SuppressWarnings("incomplete-switch")
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
         case KNEE_PITCH:
            return FRONT_LEFT_KNEE_PITCH;
         case ANKLE_PITCH:
            return FRONT_LEFT_ANKLE_PITCH;
         }
         break;
      case FRONT_RIGHT:
         switch(legJointName)
         {
         case HIP_ROLL:
            return FRONT_RIGHT_HIP_ROLL;
         case HIP_PITCH:
            return FRONT_RIGHT_HIP_PITCH;
         case KNEE_PITCH:
            return FRONT_RIGHT_KNEE_PITCH;
         case ANKLE_PITCH:
            return FRONT_RIGHT_ANKLE_PITCH;
         }
         break;
      case HIND_RIGHT:
         switch(legJointName)
         {
         case HIP_ROLL:
            return HIND_RIGHT_HIP_ROLL;
         case HIP_PITCH:
            return HIND_RIGHT_HIP_PITCH;
         case KNEE_PITCH:
            return HIND_RIGHT_KNEE_PITCH;
         case ANKLE_PITCH:
            return HIND_RIGHT_ANKLE_PITCH;
         }
         break;
      case HIND_LEFT:
         switch(legJointName)
         {
         case HIP_ROLL:
            return HIND_LEFT_HIP_ROLL;
         case HIP_PITCH:
            return HIND_LEFT_HIP_PITCH;
         case KNEE_PITCH:
            return HIND_LEFT_KNEE_PITCH;
         case ANKLE_PITCH:
            return HIND_LEFT_ANKLE_PITCH;
         }
         break;
      }

      throw new IllegalArgumentException("Leg joint does not exist: " + quadrant + " " + legJointName);
   }
   
   public String getUnderBarName()
   {
      if (jointRole.equals(JointRole.NECK))
      {
         return neckJointName.getUnderBarName();
      }
      else
      {
         return robotQuadrant.getUnderBarName() + "_" + legJointName.getUnderBarName();
      }
   }
}
