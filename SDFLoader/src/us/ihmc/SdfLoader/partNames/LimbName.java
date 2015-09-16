package us.ihmc.SdfLoader.partNames;

import org.apache.commons.lang3.StringUtils;

public enum LimbName
{
   ARM, LEG;

   public static final LimbName[] values = values();
   
   public String getCamelCaseNameForStartOfExpression()
   {
      return name().toLowerCase();
   }

   public String getCamelCaseNameForMiddleOfExpression()
   {
      return StringUtils.capitalize(getCamelCaseNameForStartOfExpression());
   }
}
