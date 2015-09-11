package us.ihmc.robotics.math.frames;

import org.apache.commons.lang3.StringUtils;

public class YoFrameVariableNameTools
{
   public static String createXName(String namePrefix, String nameSuffix)
   {
      return createName(namePrefix, "x", nameSuffix);
   }
   
   public static String createYName(String namePrefix, String nameSuffix)
   {
      return createName(namePrefix, "y", nameSuffix);
   }
   
   public static String createZName(String namePrefix, String nameSuffix)
   {
      return createName(namePrefix, "z", nameSuffix);
   }

   public static String createName(String namePrefix, String between, String nameSuffix)
   {
      if ((namePrefix == null) || (namePrefix.equals("") || namePrefix.endsWith("_")))
      {
         return namePrefix +  StringUtils.uncapitalize(between) + nameSuffix; 
      }

      else
      {
         return namePrefix + StringUtils.capitalize(between) + nameSuffix;
      }
   }
}
