package us.ihmc.utilities.parameterOptimization;

import java.util.ArrayList;

public class ListOfParametersToOptimize
{
   private final ArrayList<ParameterToOptimize> parametersToOptimize = new ArrayList<ParameterToOptimize>();
   
   public ListOfParametersToOptimize()
   {
      
   }
   
   public ListOfParametersToOptimize(ListOfParametersToOptimize parametersOne, ListOfParametersToOptimize parametersTwo)
   {
      this();
      
      this.addParametersToOptimize(parametersOne.parametersToOptimize);
      this.addParametersToOptimize(parametersTwo.parametersToOptimize); 
   }
   
   public void addParameterToOptimize(ParameterToOptimize parameterToOptimize)
   {
      this.parametersToOptimize.add(parameterToOptimize);
   }
   
   public void addParametersToOptimize(ArrayList<ParameterToOptimize> parametersToOptimize)
   {
      this.parametersToOptimize.addAll(parametersToOptimize);
   }
   
   public ParameterToOptimize get(int i)
   {
      return parametersToOptimize.get(i);
   }

   public int getNumberOfParameters()
   {
      return parametersToOptimize.size();
   }
   
   public String[] getNames()
   {
      int numberOfParameters = parametersToOptimize.size();
      String[] ret = new String[numberOfParameters];
      
      for(int i=0; i<numberOfParameters; i++)
      {
        ret[i] = parametersToOptimize.get(i).getName();
      }
      
      return ret;
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
   
   public double[] getCurrentValuesAsZeroToOnes()
   {
      double[] ret = new double[parametersToOptimize.size()];
      
      for (int i=0; i<parametersToOptimize.size(); i++)
      {
         ParameterToOptimize parameterToOptimize = parametersToOptimize.get(i);
         ret[i] = parameterToOptimize.getCurrentValueFromZeroToOne();
      }
      
      return ret;
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
   
   public String toString()
   {
      StringBuilder builder = new StringBuilder();
      
      for(int i=0; i<parametersToOptimize.size(); i++)
      {
         ParameterToOptimize parameterToOptimize = parametersToOptimize.get(i);
         builder.append(parameterToOptimize.toString() + "\n");        
      }
      
      return builder.toString();
   }

   public double[] getValuesAsDoubles()
   {
      double[] ret = new double[this.getNumberOfParameters()];
      for (int i=0; i<this.getNumberOfParameters(); i++)
      {
         ret[i] = this.get(i).getCurrentValueAsADouble();
      }

      return ret;
   }

}
