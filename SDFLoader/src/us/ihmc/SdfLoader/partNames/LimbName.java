package us.ihmc.SdfLoader.partNames;

import us.ihmc.tools.FormattingTools;

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
      return FormattingTools.capitalizeFirstLetter(getCamelCaseNameForStartOfExpression());
   }
}
