package us.ihmc.robotics.partNames;

import org.apache.commons.lang3.StringUtils;

import com.google.common.base.CaseFormat;

public enum ArmJointName
{
   CLAVICLE_ROLL,
   SHOULDER_YAW,
   SHOULDER_ROLL,
   SHOULDER_PITCH,
   ELBOW_PITCH,
   WRIST_ROLL,
   FIRST_WRIST_PITCH,
   SECOND_WRIST_PITCH,
   ELBOW_ROLL,
   ELBOW_YAW,
   WRIST_YAW;

   public static final ArmJointName[] values = values();

   public String getCamelCaseNameForStartOfExpression()
   {
      return CaseFormat.UPPER_UNDERSCORE.to(CaseFormat.LOWER_CAMEL, name());
   }

   public String getCamelCaseNameForMiddleOfExpression()
   {
      return StringUtils.capitalize(getCamelCaseNameForStartOfExpression());
   }

   @Override
   public String toString()
   {
      return getCamelCaseNameForMiddleOfExpression();
   }
}
