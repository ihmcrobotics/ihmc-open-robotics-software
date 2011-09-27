package us.ihmc.util.parameterOptimization;

import java.util.ArrayList;

public class ListOfParametersToOptimize
{
   private final ArrayList<ParameterToOptimize> parametersToOptimize = new ArrayList<ParameterToOptimize>();
   
   public ListOfParametersToOptimize()
   {
      
   }
   
   public void addParameterToOptimize(ParameterToOptimize parameterToOptimize)
   {
      this.parametersToOptimize.add(parameterToOptimize);
   }

   public ParameterToOptimize get(int i)
   {
      return parametersToOptimize.get(i);
   }

   public int getNumberOfParameters()
   {
      return parametersToOptimize.size();
   }
   
   public int[] getBitsOfResolution()
   {
      int[] bitsOfResolution = new int[this.getNumberOfParameters()];

      for (int i=0; i<this.getNumberOfParameters(); i++)
      {
         ParameterToOptimize parameterToOptimize = this.get(i);
         bitsOfResolution[i] =  parameterToOptimize.getBitsOfResolution(); 
      }

      return bitsOfResolution;
   }
   
   public void setCurrentValuesGivenZeroToOnes(double[] zeroToOnes)
   {
      if (zeroToOnes.length != parametersToOptimize.size())
         throw new RuntimeException("zeroToOnes.length != parametersToOptimize.size()");
      
      for (int i=0; i<parametersToOptimize.size(); i++)
      {
         ParameterToOptimize parameterToOptimize = parametersToOptimize.get(i);
         parameterToOptimize.setCurrentValueGivenZeroToOne(zeroToOnes[i]);
      }
   }

   public void setCurrentValues(ListOfParametersToOptimize listOfParameters)
   {
      if (this.parametersToOptimize.size() != listOfParameters.parametersToOptimize.size())
      {
         throw new RuntimeException("List lengths are different!");
      }
      
      for (int i=0; i<parametersToOptimize.size(); i++)
      {
         parametersToOptimize.get(i).setCurrentValue(listOfParameters.parametersToOptimize.get(i));
      }
      
   }

}
