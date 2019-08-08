package us.ihmc.tools.property;

import org.apache.commons.lang3.ClassUtils;
import org.apache.commons.lang3.StringUtils;

public class StoredPropertyKey<T>
{
   private final String titleCasedName;
   private final String saveName;
   private final Class<T> type;
   private final int index;
   private final Object defaultValue;

   public StoredPropertyKey(Class<T> type, int index, String titleCasedName)
   {
      this(type, index, titleCasedName, null);
   }

   public StoredPropertyKey(Class<T> type, int index, String titleCasedName, Object defaultValue)
   {
      if (!ClassUtils.isPrimitiveOrWrapper(type))
      {
         throw new RuntimeException("Type must be primitive!");
      }

      this.type = type;
      this.index = index;
      this.titleCasedName = titleCasedName;
      this.defaultValue = defaultValue;

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

   public Object getDefaultValue()
   {
      return defaultValue;
   }

   public boolean hasDefaultValue()
   {
      return defaultValue != null;
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