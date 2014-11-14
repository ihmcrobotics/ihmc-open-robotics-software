package us.ihmc.simulationconstructionset.util.simulationRunner;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariableList;

public class VariablesThatShouldMatchList
{
   private final List<YoVariable<?>[]> variablesThatShouldMatch = new ArrayList<YoVariable<?>[]>();
   private final List<YoVariable<?>> variablesInOneButNotInTwo = new ArrayList<YoVariable<?>>();
   private final List<YoVariable<?>> variablesInTwoButNotInOne = new ArrayList<YoVariable<?>>();


   public VariablesThatShouldMatchList(YoVariableRegistry registryOne, YoVariableRegistry registryTwo, List<String> exceptions)
   {
      this(registryOne.getAllVariablesIncludingDescendants(), registryTwo.getAllVariablesIncludingDescendants(), exceptions);
   }

   public VariablesThatShouldMatchList(YoVariableList varListOne, YoVariableList varListTwo, List<String> exceptions)
   {
      this(varListOne.getVariables(), varListTwo.getVariables(), exceptions);
   }


   public VariablesThatShouldMatchList(List<YoVariable<?>> variablesOne, List<YoVariable<?>> variablesTwo, List<String> exceptions)
   {
      variablesOne = copyListButRemoveExceptions(variablesOne, exceptions);
      variablesTwo = copyListButRemoveExceptions(variablesTwo, exceptions);
      copyListAndReorder(variablesOne, variablesTwo, variablesInOneButNotInTwo, variablesInTwoButNotInOne);

      // Check to make sure there are no variables in VarListTwo that aren't in VarListOne
      if (variablesOne.size() != variablesTwo.size())
      {
         throw new RuntimeException("Variable lists don't have same length!");
      }

      for (int i = 0; i < variablesOne.size(); i++)
      {
         YoVariable<?> variableOne = variablesOne.get(i);
         YoVariable<?> variableTwo = variablesTwo.get(i);


         String nameOne = variableOne.getFullNameWithNameSpace();
         String nameTwo = variableTwo.getFullNameWithNameSpace();

         if (!nameOne.equals(nameTwo))
         {
            throw new RuntimeException(nameOne + " doesn't equal " + nameTwo);
         }

         variablesThatShouldMatch.add(new YoVariable[] {variableOne, variableTwo});
      }
   }

   private List<YoVariable<?>> copyListButRemoveExceptions(List<YoVariable<?>> variables, List<String> exceptions)
   {
      List<YoVariable<?>> ret = new ArrayList<YoVariable<?>>();

      for (YoVariable<?> variable : variables)
      {
         if (!isException(exceptions, variable))
         {
            ret.add(variable);
         }
      }

      return ret;
   }

   private static void copyListAndReorder(List<YoVariable<?>> variablesOne,
         List<YoVariable<?>> variablesTwo, List<YoVariable<?>> variablesInOneButNotInTwo, List<YoVariable<?>> variablesInTwoButNotInOne)
   {
      ArrayList<YoVariable<?>> newVariablesTwo = new ArrayList<YoVariable<?>>();

      for (YoVariable<?> variableInOne : variablesOne)
      {
         YoVariable<?> variableInTwo = getVariableWithExactName(variableInOne.getFullNameWithNameSpace(), variablesTwo);

         if (variableInTwo == null)
         {
            variablesInOneButNotInTwo.add(variableInOne);
         }

         else
         {
            newVariablesTwo.add(variableInTwo);
         }
      }

      for (YoVariable<?> variableToRemoveFromOne : variablesInOneButNotInTwo)
      {
         variablesOne.remove(variableToRemoveFromOne);
      }
      
      for (YoVariable<?> newVariableTwo : newVariablesTwo)
      {
         variablesTwo.remove(newVariableTwo);
      }
      
      for (YoVariable<?> extraVariablesInTwo : variablesTwo)
      {
         variablesInTwoButNotInOne.add(extraVariablesInTwo);
      }
      
      variablesTwo.clear();
      variablesTwo.addAll(newVariablesTwo); 
   }


   private static YoVariable<?> getVariableWithExactName(String name, List<YoVariable<?>> variables)
   {
      for (YoVariable<?> variable : variables)
      {
         if (variable.getFullNameWithNameSpace().equals(name))
            return variable;
      }

      return null;
   }

   private static boolean isException(List<String> exceptions, YoVariable<?> variable)
   {
      boolean isException = false;

      if (exceptions != null)
      {
         for (String exceptionName : exceptions)
         {
            String lowerCaseVariableName = variable.getName().toLowerCase();
            String lowerCaseExceptionString = exceptionName.toLowerCase();
            isException = (lowerCaseVariableName.contains(lowerCaseExceptionString));

            if (isException)
            {
               return isException;
            }
         }
      }

      return isException;
   }


   public boolean doVariableValuesMatch(List<VariableDifference> variableDifferences, double time, double maxDifferenceAllowed, boolean checkForPercentDifference)
   {
      boolean ret = true;

      for (YoVariable<?>[] twoVariablesThatShouldMatch : variablesThatShouldMatch)
      {
         YoVariable<?> variableOne = twoVariablesThatShouldMatch[0];
         YoVariable<?> variableTwo = twoVariablesThatShouldMatch[1];

         double valueOne = variableOne.getValueAsDouble();
         double valueTwo = variableTwo.getValueAsDouble();

         double absoluteDifference = Math.abs(valueTwo - valueOne);
         
         boolean variablesAreDifferent;

         if (checkForPercentDifference)
         {
            double max = Math.max(Math.abs(valueOne), Math.abs(valueTwo));
            double percentDiff = absoluteDifference / max;

            variablesAreDifferent = (percentDiff > maxDifferenceAllowed);
         }
         else
         {
            variablesAreDifferent = (absoluteDifference > maxDifferenceAllowed);
         }

         if (variablesAreDifferent)
         {
//            System.out.println("absoluteDifference = " + absoluteDifference + ", maxDifferenceAllowed = " + maxDifferenceAllowed);
            variableDifferences.add(new VariableDifference(time, variableOne, variableTwo));
            ret = false;
         }
      }
      
      for (YoVariable<?> variable : variablesInOneButNotInTwo)
      {
         variableDifferences.add(new VariableDifference(time, variable, null));
         ret = false;
      }
      
      for (YoVariable<?> variable : variablesInTwoButNotInOne)
      {
         variableDifferences.add(new VariableDifference(time, null, variable));
         ret = false;
      }

      return ret;
   }


}
