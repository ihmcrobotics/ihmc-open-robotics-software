package us.ihmc.gdx.ui.yo;

import java.util.Set;
import java.util.TreeSet;

public class GDXYoGraphGroup
{
   private TreeSet<String> variableNames = new TreeSet<>();

   public void addVariable(String variableName)
   {
      variableNames.add(variableName);
   }

   public Set<String> getVariableNames()
   {
      return variableNames;
   }
}
