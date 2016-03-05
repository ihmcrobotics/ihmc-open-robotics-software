package us.ihmc.robotics.filters;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

import org.apache.commons.lang3.ArrayUtils;

public class GlitchFilterForDataSet
{
   private ArrayList<Double> fractionOfPointsFilteredList = new ArrayList<Double>();
   private ArrayList<Integer> numberOfPointsFilteredList = new ArrayList<Integer>();
   private final int maximumNumberOfIterations;
   private final double accelerationThreshold;
   private int numberOfPointsFiltered = Integer.MAX_VALUE;

   public GlitchFilterForDataSet(int maximumNumberOfIterations, double accelerationThreshold)
   {
      this.maximumNumberOfIterations = maximumNumberOfIterations;
      this.accelerationThreshold = accelerationThreshold;
   }

   public ArrayList<Double> getGlitchFilteredSet(ArrayList<Double> dataSetToFilter)
   {
      double[] filteredArray = getGlitchFilteredSet(ArrayUtils.toPrimitive(dataSetToFilter.toArray(new Double[dataSetToFilter.size()])));
      return new ArrayList<Double>(Arrays.asList(ArrayUtils.toObject(filteredArray)));
   }

   public double[] getGlitchFilteredSet(double[] dataSetToFilter)
   {
      fractionOfPointsFilteredList = new ArrayList<Double>();
      numberOfPointsFilteredList = new ArrayList<Integer>();

      double[] ret = Arrays.copyOf(dataSetToFilter, dataSetToFilter.length);
      int totalNumberOfPoints = ret.length;

      int numberOfRuns = 0;
      while (numberOfRuns < maximumNumberOfIterations && numberOfPointsFiltered > 0)
      {
         ret = getFilteredSet(ret);

         numberOfPointsFilteredList.add(numberOfPointsFiltered);
         fractionOfPointsFilteredList.add(((double) numberOfPointsFiltered) / ((double) totalNumberOfPoints));
         numberOfRuns++;
      }

      return ret;
   }

   public ArrayList<Double> getFractionOfPointsFiltered()
   {
      return fractionOfPointsFilteredList;
   }

   public ArrayList<Integer> getNumberOfPointsFiltered()
   {
      return numberOfPointsFilteredList;
   }

   private double[] getFilteredSet(double[] dataToFilter)
   {
      int sizeOfWindow = Math.min(20, (int) (Math.floor(((dataToFilter.length + 1) / 2.0))));

      int numberOfPoints = dataToFilter.length;

      double[] acceleration = new double[numberOfPoints];
      int sizeOfAccelerationWindow = 1;

      //Calculate Acceleration
      for (int i = sizeOfAccelerationWindow; i < (numberOfPoints - sizeOfAccelerationWindow); i++)
      {
         acceleration[i] = (dataToFilter[i + sizeOfAccelerationWindow] - 2 * dataToFilter[i] + dataToFilter[i - sizeOfAccelerationWindow]) / (sizeOfAccelerationWindow * sizeOfAccelerationWindow);
      }

      //Get indicies where acceleration is too large
      ArrayList<Integer> listOfHighAccelIndecies = new ArrayList<Integer>();
      for (int i = 0; i < acceleration.length; i++)
      {
         if (Math.abs(acceleration[i]) > accelerationThreshold)
            listOfHighAccelIndecies.add(i);
      }

      double[] averagedData = GlitchFilterForDataSet.getSetFilteredWithWindowedAverage(dataToFilter, sizeOfWindow);

      //Remove the false positives
      listOfHighAccelIndecies = getListWithFalsePositivesRemoved(listOfHighAccelIndecies, dataToFilter, averagedData);

      numberOfPointsFiltered = listOfHighAccelIndecies.size();

      //Replace all data points for the glitch indicies with smoothed points

      double[] ret = Arrays.copyOf(dataToFilter, dataToFilter.length);

      replaceGlitchesWithSmoothedData(ret, listOfHighAccelIndecies);

      return ret;
   }

   private void replaceGlitchesWithSmoothedData(double[] data, ArrayList<Integer> listOfHighAccelIndecies)
   {
      for (int i = 0; i < listOfHighAccelIndecies.size(); i++)
      {
         int indexToReplace = listOfHighAccelIndecies.get(i);

         int indexBelow = indexToReplace - 1;

         while (listOfHighAccelIndecies.contains(indexBelow))
            indexBelow = indexBelow - 1;

         int indexAbove = indexToReplace + 1;
         while (listOfHighAccelIndecies.contains(indexAbove))
            indexAbove = indexAbove + 1;

         double newValue = (data[indexBelow] + data[indexAbove]) / 2.0;
         data[indexToReplace] = newValue;
      }
   }

   private ArrayList<Integer> getListWithFalsePositivesRemoved(ArrayList<Integer> listOfHighAccelIndecies, double[] dataToFilter, double[] averagedData)
   {
      ArrayList<Integer> ret = new ArrayList<Integer>();

      int foundCount = 1;

      //Start from the second element
      for (int i = 1; i < listOfHighAccelIndecies.size(); i++)
      {
         if (listOfHighAccelIndecies.get(i) - listOfHighAccelIndecies.get(i - 1) == 1)
            foundCount++;
         else
         {
            //Skip if find only 1 point in a row. They always come in 2 or more together
            if (foundCount != 1)
            {
               List<Integer> indeciesToCheck = listOfHighAccelIndecies.subList(i - foundCount, i); //    badIndicies((i-foundCount):i-1);
               ret.addAll(getBadIndecies(indeciesToCheck, dataToFilter, averagedData));
               foundCount = 1;
            }
         }

         if (i == (listOfHighAccelIndecies.size() - 1) && foundCount > 1)
         {
            List<Integer> indeciesToCheck = listOfHighAccelIndecies.subList(i - foundCount + 1, i + 1); //    badIndicies((i-foundCount):i-1);
            ret.addAll(getBadIndecies(indeciesToCheck, dataToFilter, averagedData));
         }
      }

      return ret;
   }

   private ArrayList<Integer> getBadIndecies(List<Integer> indeciesToCheck, double[] dataToFilter, double[] averagedData)
   {
      //find the average difference
      int numberOfElements = indeciesToCheck.size();

      double total = 0.0;
      double[] difference = new double[numberOfElements];
      for (int i = 0; i < numberOfElements; i++)
      {
         difference[i] = Math.abs(dataToFilter[indeciesToCheck.get(i)] - averagedData[indeciesToCheck.get(i)]);
         total = total + difference[i];
      }

      double averageDifference = total / ((double) numberOfElements);

      ArrayList<Integer> ret = new ArrayList<Integer>();
      for (int i = 0; i < numberOfElements; i++)
      {
         if (difference[i] > averageDifference)
            ret.add(indeciesToCheck.get(i));
      }

      return ret;
   }

   public static double[] getSetFilteredWithWindowedAverage(double[] dataSetToFilter, int numberOfPointsOnEitherSideFormingWindow)
   {
      if ((dataSetToFilter == null) || (2 * numberOfPointsOnEitherSideFormingWindow + 1) > dataSetToFilter.length)
         return dataSetToFilter;

      double[] ret = Arrays.copyOf(dataSetToFilter, dataSetToFilter.length);
      for (int i = 0; i < (dataSetToFilter.length); i++)
      {
         double total = 0;

         int startingIndex = i - numberOfPointsOnEitherSideFormingWindow;
         if (startingIndex < 0)
            startingIndex = 0;

         int endingIndex = i + numberOfPointsOnEitherSideFormingWindow;
         if (endingIndex > dataSetToFilter.length - 1)
            endingIndex = dataSetToFilter.length - 1;

//         int counter = 0;
         for (int j = startingIndex; j <= endingIndex; j++)
         {
            total = total + dataSetToFilter[j];
//            counter++;
         }

         int pointsInSample = endingIndex - startingIndex + 1;

         ret[i] = total / ((double) (pointsInSample));
      }

      return ret;
   }

   public static void main(String[] args)
   {
      GlitchFilterForDataSet glitchFilterForDataSet = new GlitchFilterForDataSet(10, 4.0);

      double stepSize = 0.01;
      int numberOfPoints = 1000;

      double[] data = new double[numberOfPoints];
      Random random = new Random(100L);
      int glitchConter = 0;
      for (int i = 0; i < numberOfPoints; i++)
      {
         if (random.nextDouble() > 0.95 && i > 40 && i < (numberOfPoints - 40))
         {
            data[i] = 10.0;
            glitchConter++;
         }
         else
            data[i] = Math.sin(i * stepSize * 2.0 * Math.PI);
      }

      double[] filteredFata = glitchFilterForDataSet.getGlitchFilteredSet(data);

      System.out.println("Actual number of glitches = " + glitchConter);

      System.out.println(glitchFilterForDataSet.getNumberOfPointsFiltered());

      System.out.print("dataFromJava=[");
      for (double value : data)
      {
         System.out.println(value);
      }
      System.out.println("];");

      System.out.print("\nfilteredFromJava=[");
      for (double value : filteredFata)
      {
         System.out.println(value);
      }
      
      System.out.println("];");
   }
}
