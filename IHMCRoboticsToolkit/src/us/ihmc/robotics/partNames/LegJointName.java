package us.ihmc.robotics.partNames;

import us.ihmc.euclid.tuple3D.Vector3D;

/**
 * Enum for Leg Joint Names. The user should not assume any ordering, or assume anything about the total
 * number of Enums here. They are meant to support multiple robots with common control modules. So joints
 * can be added even if they exist on one robot but not on others.
 *
 * Therefore, the user should not use LegJointName.values() or LegJointName.foo.ordinal().
 * Unfortunately, we cannot override those methods and make them throw exceptions.
 *
 * For now to make sure the user doesn't misuse LegJointName in that way, we throw in a bunch of
 * FUTURE_EXPANSION_TEST elements, which should cause some exceptions when things are misused.
 * If they are all iterated over but fewer are expected, then that might throw an out of bounds exception
 * somewhere. However, they can still be misused in many ways so we need to occasionally search for
 * LegJointName.values(), LegJointName.foo.ordinal() and make sure they are not used. But checking for
 * variableName.ordinal() is harder, since we do use this a lot in places where the ordinals are pretty
 * much guaranteed not to change, like RobotSide.
 *
 * TODO: Search for and remove bad uses of LegJointNames throughout the code.
 *
 * Also, a simple test is to run a sim, then reorder these, then run again and see if anything changes.
 */
public enum LegJointName
{
   FUTURE_EXPANSION_TEST1, HIP_PITCH, HIP_ROLL, FUTURE_EXPANSION_TEST2, HIP_YAW, KNEE_PITCH, KNEE_ROLL, ANKLE_ROLL, ANKLE_PITCH, FUTURE_EXPANSION_TEST3;
   
   /**
    * @deprecated Do not use this!
    */
   public static final LegJointName[] values = values();
   
   public String getUnderBarName()
   {
      switch (this)
      {
         case HIP_YAW :
            return "hip_yaw";

         case HIP_ROLL :
            return "hip_roll";

         case HIP_PITCH :
            return "hip_pitch";

         case KNEE_PITCH :
            return "knee_pitch";

         case ANKLE_PITCH :
            return "ankle_pitch";

         case ANKLE_ROLL :
            return "ankle_roll";

         default :
            return "unknown";
      }
   }
   
   public String getShortUnderBarName()
   {
      switch (this)
      {
         case HIP_YAW :
            return "h_yaw";

         case HIP_ROLL :
            return "h_roll";

         case HIP_PITCH :
            return "h_pitch";

         case KNEE_PITCH :
            return "k_pitch";

         case ANKLE_PITCH :
            return "a_pitch";

         case ANKLE_ROLL :
            return "a_roll";

         default :
            return "unknown";
      }
   }

   public String getCamelCaseName()
   {
      switch (this)
      {
         case HIP_YAW :
            return "hipYaw";

         case HIP_ROLL :
            return "hipRoll";

         case HIP_PITCH :
            return "hipPitch";

         case KNEE_PITCH :
            return "kneePitch";

         case ANKLE_ROLL :
            return "ankleRoll";

         case ANKLE_PITCH :
            return "anklePitch";

         default :
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
         case HIP_YAW :
            return "HipYaw";

         case HIP_ROLL :
            return "HipRoll";

         case HIP_PITCH :
            return "HipPitch";

         case KNEE_PITCH :
            return "KneePitch";

         case ANKLE_ROLL :
            return "AnkleRoll";

         case ANKLE_PITCH :
            return "AnklePitch";

         default :
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

   public Vector3D getJointAxis()
   {
      switch (this)
      {
         case HIP_YAW :
            return zAxis();

         case HIP_ROLL :
            return xAxis();

         case HIP_PITCH :
            return yAxis();

         case KNEE_PITCH :
            return yAxis();

         case ANKLE_PITCH :
            return yAxis();

         case ANKLE_ROLL :
            return xAxis();

         default :
            throw new RuntimeException("Enum constant not handled.");
      }
   }

   @Override
   public String toString()
   {
      return getPascalCaseName();
   }

   private static Vector3D xAxis()
   {
      return new Vector3D(1.0, 0.0, 0.0);
   }

   private static Vector3D yAxis()
   {
      return new Vector3D(0.0, 1.0, 0.0);
   }

   private static Vector3D zAxis()
   {
      return new Vector3D(0.0, 0.0, 1.0);
   }
}
