package us.ihmc.tools.property;

import org.apache.commons.lang3.ClassUtils;
import org.apache.commons.lang3.StringUtils;

public class StoredPropertyKey<T>
{
   private final String titleCasedName;
   private final String saveName;
   private final Class<T> type;
   private final int index;

   public StoredPropertyKey(Class<T> type, int index, String titleCasedName)
   {
      if (!ClassUtils.isPrimitiveOrWrapper(type))
      {
         throw new RuntimeException("Type must be primitive!");
      }

      this.type = type;
      this.index = index;
      this.titleCasedName = titleCasedName;

      saveName = buildCamelCasedName();
   }

   public String getTitleCasedName()
   {
      return titleCasedName;
   }

   public int getIndex()
   {
      return index;
   }

   public Class<T> getType()
   {
      return type;
   }

   public String getCamelCasedName()
   {
      return saveName;
   }

   private String buildCamelCasedName() // to lower camel case
   {
      String[] splitBySpaces = titleCasedName.split(" ");

      String saveName = "";
      for (String splitBySpace : splitBySpaces)
      {
         saveName += StringUtils.capitalize(splitBySpace);
      }
      saveName = StringUtils.uncapitalize(saveName);

      return saveName;
   }
}