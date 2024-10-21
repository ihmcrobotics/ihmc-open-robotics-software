package us.ihmc.robotics.partNames;

import us.ihmc.euclid.Axis3D;

/**
 * Enum for Leg Joint Names. The user should not assume any ordering, or assume anything about the
 * total number of Enums here. They are meant to support multiple robots with common control
 * modules. So joints can be added even if they exist on one robot but not on others.
 * <p>
 * Therefore, the user should not use LegJointName.values() or LegJointName.foo.ordinal().
 * Unfortunately, we cannot override those methods and make them throw exceptions.
 * </p>
 * <p>
 * For now to make sure the user doesn't misuse LegJointName in that way, we throw in a bunch of
 * FUTURE_EXPANSION_TEST elements, which should cause some exceptions when things are misused. If
 * they are all iterated over but fewer are expected, then that might throw an out of bounds
 * exception somewhere. However, they can still be misused in many ways so we need to occasionally
 * search for LegJointName.values(), LegJointName.foo.ordinal() and make sure they are not used. But
 * checking for variableName.ordinal() is harder, since we do use this a lot in places where the
 * ordinals are pretty much guaranteed not to change, like RobotSide.
 * <p>
 * TODO: Search for and remove bad uses of LegJointNames throughout the code.
 * </p>
 * <p>
 * Also, a simple test is to run a sim, then reorder these, then run again and see if anything
 * changes.
 * </p>
 */
public enum LegJointName
{
   FUTURE_EXPANSION_TEST1,
   HIP_PITCH,
   HIP_ROLL,
   FUTURE_EXPANSION_TEST2,
   HIP_YAW,
   KNEE_PITCH,
   KNEE_YAW,
   KNEE_ROLL,
   ANKLE_ROLL,
   ANKLE_PITCH,
   FUTURE_EXPANSION_TEST3;

   /**
    * @deprecated Do not use this!
    */
   public static final LegJointName[] values = values();

   public String getUnderBarName()
   {
      switch (this)
      {
         case HIP_YAW:
            return "hip_yaw";

         case HIP_ROLL:
            return "hip_roll";

         case HIP_PITCH:
            return "hip_pitch";

         case KNEE_PITCH:
            return "knee_pitch";

         case KNEE_YAW:
            return "knee_yaw";

         case ANKLE_PITCH:
            return "ankle_pitch";

         case ANKLE_ROLL:
            return "ankle_roll";

         default:
            return "unknown";
      }
   }

   public String getShortUnderBarName()
   {
      switch (this)
      {
         case HIP_YAW:
            return "h_yaw";

         case HIP_ROLL:
            return "h_roll";

         case HIP_PITCH:
            return "h_pitch";

         case KNEE_PITCH:
            return "k_pitch";

         case ANKLE_PITCH:
            return "a_pitch";

         case ANKLE_ROLL:
            return "a_roll";

         default:
            return "unknown";
      }
   }

   public String getCamelCaseName()
   {
      switch (this)
      {
         case HIP_YAW:
            return "hipYaw";

         case HIP_ROLL:
            return "hipRoll";

         case HIP_PITCH:
            return "hipPitch";

         case KNEE_PITCH:
            return "kneePitch";

         case KNEE_YAW:
            return "kneeYaw";

         case ANKLE_ROLL:
            return "ankleRoll";

         case ANKLE_PITCH:
            return "anklePitch";

         default:
            return "unknown Position";
      }
   }

   /**
    * @deprecated Use getCamelCaseName() instead.
    */
   public String getCamelCaseNameForStartOfExpression()
   {
      return getCamelCaseName();
   }

   public String getPascalCaseName()
   {
      switch (this)
      {
         case HIP_YAW:
            return "HipYaw";

         case HIP_ROLL:
            return "HipRoll";

         case HIP_PITCH:
            return "HipPitch";

         case KNEE_PITCH:
            return "KneePitch";

         case KNEE_YAW:
            return "KneeYaw";

         case ANKLE_ROLL:
            return "AnkleRoll";

         case ANKLE_PITCH:
            return "AnklePitch";

         default:
            return "Unknown Position";
      }
   }

   /**
    * @deprecated Use getPascalCaseName() instead.
    */
   public String getCamelCaseNameForMiddleOfExpression()
   {
      return getPascalCaseName();
   }

   public Axis3D getJointAxis()
   {
      return switch (this)
      {
         case HIP_YAW -> Axis3D.Z;
         case HIP_ROLL -> Axis3D.X;
         case HIP_PITCH -> Axis3D.Y;
         case KNEE_PITCH -> Axis3D.Y;
         case KNEE_YAW -> Axis3D.Z;
         case ANKLE_PITCH -> Axis3D.Y;
         case ANKLE_ROLL -> Axis3D.X;
         default -> throw new RuntimeException("Enum constant not handled.");
      };
   }

   @Override
   public String toString()
   {
      return getPascalCaseName();
   }
}
