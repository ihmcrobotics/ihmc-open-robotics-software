package us.ihmc.footstepPlanning.graphSearch.parameters;

import org.apache.commons.lang3.StringUtils;

import java.util.List;

public class FootstepPlannerParameterKey<T>
{
   private final String titleCasedName;
   private final String saveName;
   private final int id;

   public FootstepPlannerParameterKey(List<FootstepPlannerParameterKey<?>> listToAddTo, int id, String titleCasedName)
   {
      this.id = id;
      this.titleCasedName = titleCasedName;

      saveName = buildSaveName();

      listToAddTo.add(this);
   }

   public String getTitleCasedName()
   {
      return titleCasedName;
   }

   public int getId()
   {
      return id;
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
