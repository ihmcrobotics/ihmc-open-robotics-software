package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.regex.Pattern;

public class RegexWholeBodySetpointParameters implements WholeBodySetpointParameters
{
   private final Map<Pattern, Double> patternSetpoints = new LinkedHashMap<>();

   public RegexWholeBodySetpointParameters(Map<String, Double> regexToSetpoint)
   {
      for (Map.Entry<String, Double> entry : regexToSetpoint.entrySet())
         patternSetpoints.put(Pattern.compile(entry.getKey()), entry.getValue());
   }

   @Override
   public double getSetpoint(String jointName)
   {
      for (Map.Entry<Pattern, Double> entry : patternSetpoints.entrySet())
      {
         if (entry.getKey().matcher(jointName).matches())
            return entry.getValue();
      }

      return 0.0;
   }
}
