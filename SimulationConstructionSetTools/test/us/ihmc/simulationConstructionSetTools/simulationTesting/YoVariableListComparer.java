package us.ihmc.simulationConstructionSetTools.simulationTesting;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.variable.YoVariable;

public class YoVariableListComparer
{
   private final double epsilon;
   private final ArrayList<YoVariable<?>[]> differences = new ArrayList<YoVariable<?>[]>();
   private ArrayList<String> exceptions = new ArrayList<String>();

   public YoVariableListComparer(double epsilon)
   {
      this.epsilon = epsilon;
   }

   public boolean compare(ArrayList<YoVariable<?>> allVariables0, ArrayList<YoVariable<?>> allVariables1)
   {
      differences.clear();

      Comparator<YoVariable<?>> alphabeticOrderComparator = new Comparator<YoVariable<?>>()
      {
      @Override
      public int compare(YoVariable<?> arg0, YoVariable<?> arg1) 
      {
         return arg0.getFullNameWithNameSpace().compareTo(arg1.getFullNameWithNameSpace());
      }
      };
      
      // First sort, then remove exceptions. After that, everything should be identical...
      Collections.sort(allVariables0, alphabeticOrderComparator);
      Collections.sort(allVariables1, alphabeticOrderComparator);

      allVariables0 = removeExceptionalVariables(allVariables0);
      allVariables1 = removeExceptionalVariables(allVariables1);
      
      int index0 = 0, index1 = 0;
      int size0 = allVariables0.size();
      int size1 = allVariables1.size();
      
      while((index0 < size0) && (index1 < size1))
      {
         YoVariable<?> var0 = allVariables0.get(index0);
         YoVariable<?> var1 = allVariables1.get(index1);

         double val0 = var0.getValueAsDouble();
         double val1 = var1.getValueAsDouble();
         
         if(!var0.getFullNameWithNameSpace().equals(var1.getFullNameWithNameSpace()))
         {
          addDifference(var0, var1);
         }

         else if (Double.isNaN(val0))
         {
          if (!Double.isNaN(val1))
          {
             addDifference(var0, var1);
          }
         }
         else if (Double.isInfinite(val0))
         {
          if (!Double.isInfinite(val1))
          {
             addDifference(var0, var1);
          }

          if (Math.signum(val0) != Math.signum(val1))
          {
             addDifference(var0, var1);
          }
         }
         else if (!MathTools.epsilonEquals(val0, var1.getValueAsDouble(), epsilon))
         {
          addDifference(var0, var1);
         }
         
         index0++;
         index1++;
      }
      
      if (size0 != size1) return false;

      return differences.isEmpty();
   }

   private ArrayList<YoVariable<?>> removeExceptionalVariables(ArrayList<YoVariable<?>> variables) 
   {
      ArrayList<YoVariable<?>> variablesWithoutExceptions = new ArrayList<YoVariable<?>>();

      for (int i=0; i<variables.size(); i++)
      {
         YoVariable<?> variable = variables.get(i);
         if (!isException(exceptions, variable))
         {
            variablesWithoutExceptions.add(variable);
         }
      }

      return variablesWithoutExceptions;
   }

   private void addDifference(YoVariable<?> var0, YoVariable<?> var1)
   {
      YoVariable<?>[] difference = {var0, var1};
      differences.add(difference);
   }

   public ArrayList<YoVariable<?>[]> getDifferences()
   {
      return differences;
   }

   @Override
   public String toString()
   {
      StringBuffer buf = new StringBuffer();
      buf.append("Differences:\n");

      for (YoVariable<?>[] difference : differences)
      {
         buf.append(difference[0]);
         buf.append("\n");
         buf.append(difference[1]);
         buf.append("\n\n");
      }

      return buf.toString();
   }

   public void addException(String exception)
   {
      exceptions.add(exception);
   }

   private static boolean isException(ArrayList<String> exceptions, YoVariable<?> variable)
   {
      boolean isException = false;

      if (exceptions != null)
      {
         for (String exceptionName : exceptions)
         {
            String lowerCaseVariableName = variable.getFullNameWithNameSpace().toLowerCase();
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
}
