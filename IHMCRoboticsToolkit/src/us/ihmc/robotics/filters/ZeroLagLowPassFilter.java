package us.ihmc.robotics.filters;

import us.ihmc.robotics.MathTools;
import us.ihmc.tools.ArrayTools;

import java.util.ArrayList;
import java.util.Arrays;

public class ZeroLagLowPassFilter
{
   private static final int numberFactor = 3; //This is from the Matlab algorithm
   
   /**
    * This is a zero phase lag filter with the same performance and the Matlab filtfilt function
    * when the filter type is a first order low pass filter. This will extrapolate beginning and end of data sequence using a "reflection method".  
    * Slopes of original and extrapolated sequences match at the end points. This reduces end effects.
    * @param arrayToFilter
    * @param alpha this is [0, 1]. 0 = no filter, 1= all filter.
    * @return
    */
   
   public static ArrayList<Double> getFilteredArray(ArrayList<Double> arrayToFilter, double alpha)
   {
       return ArrayTools.getArrayListFromArray(getFilteredArray(ArrayTools.getArrayFromArrayList(arrayToFilter), alpha));
   }
   
   public static double[] getFilteredArray(double[] arrayToFilter, double alpha)
   {
      if (arrayToFilter.length < 3)
         return ArrayTools.copyArray(arrayToFilter);
      
      alpha = MathTools.clipToMinMax(alpha, 0.0, 1.0);

      ArrayList<Double> arrayListToFilter = ArrayTools.getArrayListFromArray(arrayToFilter);
            
      int originalArrayLength = arrayListToFilter.size();
   
      //Extrapolate at the ends
      preAndPostPendedToList(arrayListToFilter);
      
      double[] listToFilter = new double[arrayListToFilter.size()];
      
      for(int i=0; i<listToFilter.length; i++)
      {
         listToFilter[i] = arrayListToFilter.get(i);
      }
              
      double[] filteredList = new double[listToFilter.length];
      
      filteredList[0] = listToFilter[0];

      for (int i = 1; i < filteredList.length; i++)
      {
         filteredList[i] = alpha * filteredList[i - 1] + (1.0 - alpha) * listToFilter[i];
      }
 
      double[] reverseOrderFilteredList = ArrayTools.getReserveredOrderedArrayCopy(filteredList);
      
      filteredList[0] = reverseOrderFilteredList[0];

      for (int i = 1; i < filteredList.length; i++)
      {
         filteredList[i] = alpha * filteredList[i - 1] + (1.0 - alpha) * reverseOrderFilteredList[i];
      }
      
      filteredList = ArrayTools.getReserveredOrderedArrayCopy(filteredList);
   
      //trim beginning and end
      double[] ret = Arrays.copyOfRange(filteredList, numberFactor, originalArrayLength + numberFactor);
      
      return ret;

   }
   
   private static void preAndPostPendedToList(ArrayList<Double> list)
   {
      
     
      double startValue = list.get(0);
      for(int i= 0; i<numberFactor; i++)
      {
         double value = 2.0 * startValue - list.get(numberFactor);
         list.add(i, value);
      }
      
      double endValue = list.get(list.size()-1);
      int startingIndex = list.size() - 2;
      for(int i= 0; i<numberFactor; i++)
      {
         double value = 2.0 * endValue - list.get(startingIndex - i);
         list.add(value);
      }
   }
   
   public static void main(String[] args)
   {
      ArrayList<Double> list = new ArrayList<Double>();
      
      double[] newList = new double[10];
      for(int i=0; i<10; i++)
      {
         list.add((double)i + 1);
         newList[i]=i+ 1;
      }
      
//      System.out.println("Original List:");
//      ArrayTools.printArray(newList, System.out);
//      System.out.println("");
      
//      ZeroLagLowPassFilter.preAndPostPendedToList(list);
//      
//      ArrayTools.printArray(list, System.out);

      double[] filteredList = ZeroLagLowPassFilter.getFilteredArray(newList, 0.5);
      
      ArrayTools.printArray(filteredList, System.out);
   }

}
