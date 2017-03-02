package us.ihmc.simulationconstructionset.util.simulationRunner;

import java.util.List;

import us.ihmc.robotics.dataStructures.variable.YoVariable;

public class VariableDifference
{
   private final double timeOfDifference;
   private final YoVariable<?> variableOne;
   private final YoVariable<?> variableTwo;
   
   public VariableDifference(double timeOfDifference, YoVariable<?> variableOne, YoVariable<?> variableTwo)
   {
      this.timeOfDifference = timeOfDifference;
      
      this.variableOne = variableOne;
      this.variableTwo = variableTwo;
   }
   
   public double getTimeOfDifference()
   {
      return timeOfDifference;
   }
   
   public YoVariable<?> getVariableOne()
   {
      return variableOne;
   }
   
   public YoVariable<?> getVariableTwo()
   {
      return variableTwo;
   }
   
   @Override
   public String toString()
   {
      String variableName1, variableName2;
      double value1, value2;
      
      if (variableOne == null)
      {
         variableName1 = "null";
         value1 = Double.NaN;
      }
      else
      {
         variableName1 = variableOne.getFullNameWithNameSpace();
         value1 = variableOne.getValueAsDouble();
      }
      
      if (variableTwo == null)
      {
         variableName2 = "null";
         value2 = Double.NaN;
      }
      else
      {
         variableName2 = variableTwo.getFullNameWithNameSpace();
         value2 = variableTwo.getValueAsDouble();
      }
      
      String ret = "At time = " + timeOfDifference + " -> " + variableName1 + " : " + value1;
      ret = ret + ", " + variableName2 + " : " + value2;
      
      return ret;
   }
   
   public static String allVariableDifferencesToString(List<VariableDifference> variableDifferences)
   {
      String ret = new String();
      
      for (VariableDifference variableDifference : variableDifferences)
      {
         ret = ret + variableDifference.toString() + "\n";
      }
      
      return ret;
   }

}
