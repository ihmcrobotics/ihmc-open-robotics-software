package us.ihmc.tools.dataSampling;

import java.util.ArrayList;
import java.util.HashMap;

/**
 * This class can be used to online sample Objects with a determined number of samples for a given time interval.
 * The samples are stored in an HashMap, and samples must come as an HashMap as well to guarantee
 * that are assigned to the correct variable.
 *
 * @author not attributable
 * @param <T> Object type. Since is an ArrayList do not use primitive types.
 */

public class DataSampler<T>
{
   private int requestedNumberOfSamples;
   private int numberOfVariables;
   private int counter;
   private double timeInterval;
   private double deltaTime;
   private double initialTime;
   private double acquisitionTime;
   private String[] variableNames;
   private HashMap<String, ArrayList<T>> allData;

   public DataSampler()
   {
      allData = new HashMap<String, ArrayList<T>>();
   }

   public void initialize(String[] variableNames, double initialTime, double timeInterval, int requestedNumberOfSamples)
   {
      this.requestedNumberOfSamples = requestedNumberOfSamples;
      this.variableNames = variableNames;
      this.numberOfVariables = variableNames.length;
      this.initialTime = initialTime;
      setTimeInterval(timeInterval);

      generateAllDataMap();
   }

   public void reInitialize(double initialTime, double timeInterval, int requestedNumberOfSamples)
   {
      this.initialTime = initialTime;
      this.requestedNumberOfSamples = requestedNumberOfSamples;
      setTimeInterval(timeInterval);
   }

   public void acquire(double time, HashMap<String, T> values)
   {
      acquisitionTime = time - initialTime;

      if (acquisitionTime > deltaTime * counter)
      {
         addSample(values);
         counter++;
      }
   }

   public void setTimeInterval(double time)
   {
      timeInterval = time;
      computeDeltaTimeAndResetCounter();
   }

   public synchronized void emptyAllSamplesButKeepVariables()
   {
      for (int i = 0; i < numberOfVariables; i++)
      {
         allData.get(variableNames[i]).clear();
      }
   }

   public synchronized void removeAllVariables()
   {
      allData.clear();
   }

   public synchronized ArrayList<T> getSingleVariableSamples(String variableName)
   {
      return allData.get(variableName);
   }

   public synchronized HashMap<String, ArrayList<T>> getAllData()
   {
      return allData;
   }

   public synchronized HashMap<String, ArrayList<T>> getAllDataCopy()
   {
      HashMap<String, ArrayList<T>> ret = new HashMap<String, ArrayList<T>>();

      for (int i = 0; i < numberOfVariables; i++)
      {
         ret.put(variableNames[i], new ArrayList<T>(allData.get(variableNames[i])));
      }

      return ret;
   }

   public double getDeltaTime()
   {
      return deltaTime;
   }

   private synchronized void generateAllDataMap()
   {
      removeAllVariables();

      for (int i = 0; i < numberOfVariables; i++)
      {
         allData.put(variableNames[i], new ArrayList<T>());
      }

      emptyAllSamplesButKeepVariables();
   }

   private synchronized void addSample(HashMap<String, T> values)
   {
      for (int i = 0; i < numberOfVariables; i++)
      {
         allData.get(variableNames[i]).add(values.get(variableNames[i]));
      }
   }

   private void computeDeltaTimeAndResetCounter()
   {
      deltaTime = timeInterval / requestedNumberOfSamples;
      counter = 0;
   }
}