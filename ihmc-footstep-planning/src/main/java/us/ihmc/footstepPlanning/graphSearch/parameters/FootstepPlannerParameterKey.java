package us.ihmc.footstepPlanning.graphSearch.parameters;

import org.apache.commons.lang3.ClassUtils;
import org.apache.commons.lang3.StringUtils;

public class FootstepPlannerParameterKey<T>
{
   private final String titleCasedName;
   private final String saveName;
   private final Class<T> type;
   private final int id;

   public FootstepPlannerParameterKey(Class<T> type, int id, String titleCasedName)
   {
      if (!ClassUtils.isPrimitiveOrWrapper(type))
      {
         throw new RuntimeException("Type must be primitive!");
      }

      this.type = type;
      this.id = id;
      this.titleCasedName = titleCasedName;

      saveName = buildSaveName();
   }

   public String getTitleCasedName()
   {
      return titleCasedName;
   }

   public int getId()
   {
      return id;
   }

   public Class<T> getType()
   {
      return type;
   }

   public String getSaveName()
   {
      return saveName;
   }

   private String buildSaveName() // to lower camel case
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
