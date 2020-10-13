package us.ihmc.footstepPlanning.ui.controllers;

import us.ihmc.footstepPlanning.log.VariableDescriptor;

import java.util.ArrayList;
import java.util.List;
import java.util.regex.Pattern;

public class FootstepPlannerVisualizerTools
{
   public static List<VariableDescriptor> search(String searchQuery, List<VariableDescriptor> allDescriptors)
   {
      if (allDescriptors == null)
      {
         return new ArrayList<>();
      }

      String[] queryTerms = searchQuery.replaceAll("\\s+","").split("&");
      Pattern[] queryPatterns = new Pattern[queryTerms.length];
      for (int i = 0; i < queryTerms.length; i++)
      {
         queryPatterns[i] = Pattern.compile(Pattern.quote(queryTerms[i]), Pattern.CASE_INSENSITIVE);
      }

      List<VariableDescriptor> result = new ArrayList<>();
      for (int i = 0; i < allDescriptors.size(); i++)
      {
         String name = allDescriptors.get(i).getName();
         boolean match = true;

         for (int j = 0; j < queryPatterns.length; j++)
         {
            if (!queryPatterns[j].matcher(name).find())
            {
               match = false;
               break;
            }
         }

          if (match)
          {
             result.add(allDescriptors.get(i));
          }
      }

      return result;
   }
}
