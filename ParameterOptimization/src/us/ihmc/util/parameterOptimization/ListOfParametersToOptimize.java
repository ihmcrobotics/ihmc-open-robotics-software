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
}
