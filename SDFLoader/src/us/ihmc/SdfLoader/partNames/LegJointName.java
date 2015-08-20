package us.ihmc.SdfLoader.partNames;

import javax.vecmath.Vector3d;

import us.ihmc.tools.FormattingTools;

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
   // M2V2: HIP_YAW, HIP_ROLL, HIP_PITCH, KNEE, ANKLE_PITCH, ANKLE_ROLL;
   // R2: HIP_PITCH, HIP_ROLL, HIP_YAW, KNEE, ANKLE_ROLL, ANKLE_PITCH;
// HIP_YAW, HIP_ROLL, HIP_PITCH, KNEE, ANKLE_PITCH, ANKLE_ROLL;

   FUTURE_EXPANSION_TEST1, HIP_PITCH, HIP_ROLL, FUTURE_EXPANSION_TEST2, HIP_YAW, KNEE, ANKLE_ROLL, ANKLE_PITCH, FUTURE_EXPANSION_TEST3;

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

         case KNEE :
            return "k";

         case ANKLE_PITCH :
            return "a_pitch";

         case ANKLE_ROLL :
            return "a_roll";

         default :
            return "unknown";
      }
   }


   public String getCamelCaseNameForStartOfExpression()
   {
      switch (this)
      {
         case HIP_YAW :
            return "hipYaw";

         case HIP_ROLL :
            return "hipRoll";

         case HIP_PITCH :
            return "hipPitch";

         case KNEE :
            return "knee";

         case ANKLE_ROLL :
            return "ankleRoll";

         case ANKLE_PITCH :
            return "anklePitch";

         default :
            return "unknown Position";
      }
   }


   public String getCamelCaseNameForMiddleOfExpression()
   {
      return FormattingTools.capitalizeFirstLetter(getCamelCaseNameForStartOfExpression());
   }

   public Vector3d getJointAxis()
   {
      switch (this)
      {
         case HIP_YAW :
            return zAxis();

         case HIP_ROLL :
            return xAxis();

         case HIP_PITCH :
            return yAxis();

         case KNEE :
            return yAxis();

         case ANKLE_PITCH :
            return yAxis();

         case ANKLE_ROLL :
            return xAxis();

         default :
            throw new RuntimeException("Enum constant not handled.");
      }
   }


   public String toString()
   {
      return getCamelCaseNameForMiddleOfExpression();
   }

   private static Vector3d xAxis()
   {
      return new Vector3d(1.0, 0.0, 0.0);
   }

   private static Vector3d yAxis()
   {
      return new Vector3d(0.0, 1.0, 0.0);
   }

   private static Vector3d zAxis()
   {
      return new Vector3d(0.0, 0.0, 1.0);
   }
}
