package us.ihmc.rdx.ui.tools;

import us.ihmc.log.LogTools;

import java.io.*;
import java.lang.reflect.Array;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.IntStream;
import java.util.stream.Stream;

/**
 * Class to record and replay multidimensional trajectories of primitive Number types.
 * The multi-dimension represents the number of elements that are considered.
 * For each dimension, at each time step the class stores or reads a set point.
 * The collection of set points defines the trajectory.
 * The trajectories are saved and loaded from .csv files.
 * Each column represents the trajectory of a distinct element (e.g., X position of the right hand, or Y position of the right hand, etc...).
 */
public class TrajectoryRecordReplay<T extends Number>
{
   private String filePath;
   private final Class<T> clazz;
   private int numberParts; // specify the number of parts you want to record (e.g., left hand, right hand, chest)
   private ArrayList<T[]> dataMatrix = new ArrayList<>();
   private ArrayList<T[]> concatenatedDataMatrix = new ArrayList<>();
   private ArrayList<T[]> splitDataMatrix = new ArrayList<>();
   private int timeStepReplay = 0;
   private boolean savedRecording = true;
   private boolean doneReplaying = true;
   private boolean concatenated = false;
   private String recordFileName = "";

   public TrajectoryRecordReplay(Class<T> clazz, String filePath, int numberParts)
   {
      super();
      this.clazz = clazz;
      this.filePath = filePath;
      this.numberParts = numberParts;
   }

   public T[] play()
   {
      return this.play(false);
   }

   public T[] play(boolean split)
   {
      if (timeStepReplay < 1)
      {
         this.readCSV();
         if (split)
            this.splitData();
      }
      T[] values;
      int size;
      if (split)
      {
         // read split data (a row for each body part)
         values = splitDataMatrix.get(timeStepReplay);
         size = splitDataMatrix.size();
      }
      else
      {
         // read default data as they are stored in the csv file
         values = dataMatrix.get(timeStepReplay);
         size = dataMatrix.size();
      }
      if (timeStepReplay >= size - 1)
      {
         doneReplaying = true;
         this.reset();
      }
      else
      {
         timeStepReplay++;
      }
      return values;
   }

   public void record(T[] values)
   {
      if (savedRecording)
         savedRecording = false;
      T[] localValues = newNumberArray(values.length);
      System.arraycopy(values, 0, localValues, 0, localValues.length);
      dataMatrix.add(localValues);
   }

   /** Useful if we are recording trajectories of different parts but not in the same scope
    * and we want to concatenate them into one single row to have a single csv file
    * rather than having multiple TrajectoryRecordReplay objects and multiple csv files */
   public void concatenateData()
   {
      for (int i = 0; i < dataMatrix.size(); i = i + numberParts)
      {
         T[] concatenatedRow = dataMatrix.get(i);
         for (int j = 1; j <= numberParts - 1; j++)
         {
            concatenatedRow = concatenateWithCopy(concatenatedRow, dataMatrix.get(i + j));
         }
         concatenatedDataMatrix.add(concatenatedRow);
      }
      concatenated = true;
   }

   /** Useful if we are replaying a csv file where multiple parts have been concatenated in one single row
    * and we want the info of each part in a separate row.
    * Not useful if you have different parts with different number of elements */
   private void splitData()
   {
      for (int i = 0; i < dataMatrix.size(); i++)
      {
         T[] row = dataMatrix.get(i);
         for (int n = 0; n <= numberParts - 1; n++)
         {
            T[] splitRow = newNumberArray(row.length / numberParts);
            for (int j = 0; j < splitRow.length; j++)
            {
               splitRow[j] = row[j + n * splitRow.length];
            }
            splitDataMatrix.add(splitRow);
         }
      }
   }

   public void saveRecording()
   {
      if (concatenated) // save concatenated data (a single row for every body part)
         writeCSV(concatenatedDataMatrix);
      else
         writeCSV(dataMatrix);
      this.reset();
      savedRecording = true;
   }

   public void readCSV()
   {
      doneReplaying = false;
      try
      {
         BufferedReader fileReader = new BufferedReader(new FileReader(filePath));
         String line = "";
         while ((line = fileReader.readLine()) != null)
         {
            String[] stringValues = line.split(",");
            T[] dataValues = newNumberArray(stringValues.length);
            IntStream.range(0, stringValues.length).forEach(i -> dataValues[i] = setValue(stringValues[i]));
            dataMatrix.add(dataValues);
         }
         fileReader.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public void writeCSV(ArrayList<T[]> dataMatrix)
   {
      List<String[]> dataLines = new ArrayList<>();
      for (T[] dataLine : dataMatrix)
      {
         String[] stringValues = new String[dataMatrix.get(0).length];
         Arrays.setAll(stringValues, j -> "" + dataLine[j]);
         dataLines.add(stringValues);
      }
      // if recordFile name has not been set, generate file with current date and time as name
      String fileName = "";
      if (recordFileName.isEmpty())
         fileName = new SimpleDateFormat("yyMMddHHmmssZ'.csv'").format(new Date());
      else
         fileName = recordFileName;
      File csvFile = new File(filePath + "/" + fileName);
      try (PrintWriter writer = new PrintWriter(csvFile))
      {
         dataLines.stream().map(this::convertToCSV).forEach(writer::println);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   private String convertToCSV(String[] data)
   {
      return Stream.of(data).map(this::escapeSpecialCharacters).collect(Collectors.joining(","));
   }

   private String escapeSpecialCharacters(String data)
   {
      String escapedData = data.replaceAll("\\R", " ");
      if (data.contains(",") || data.contains("\"") || data.contains("'"))
      {
         data = data.replace("\"", "\"\"");
         escapedData = "\"" + data + "\"";
      }
      return escapedData;
   }

   private void reset()
   {
      timeStepReplay = 0;
      dataMatrix.clear();
      concatenated = false;
      concatenatedDataMatrix.clear();
      splitDataMatrix.clear();
   }

   @SuppressWarnings("unchecked")
   private T[] newNumberArray(int size)
   {
      T[] value;
      if (clazz.isAssignableFrom(Integer.class))
      {
         value = (T[]) new Integer[size];
      }
      else if (clazz.isAssignableFrom(Short.class))
      {
         value = (T[]) new Short[size];
      }
      else if (clazz.isAssignableFrom(Long.class))
      {
         value = (T[]) new Long[size];
      }
      else if (clazz.isAssignableFrom(Double.class))
      {
         value = (T[]) new Double[size];
      }
      else if (clazz.isAssignableFrom(Float.class))
      {
         value = (T[]) new Float[size];
      }
      else
      {
         throw new IllegalArgumentException("Invalid type for TrajectoryRecordReplay. It only accepts primitive Number types.");
      }
      return value;
   }

   @SuppressWarnings("unchecked")
   private T setValue(String input)
   {
      T value;
      if (clazz.isAssignableFrom(Integer.class))
      {
         value = (T) Integer.valueOf(input);
      }
      else if (clazz.isAssignableFrom(Short.class))
      {
         value = (T) Short.valueOf(input);
      }
      else if (clazz.isAssignableFrom(Long.class))
      {
         value = (T) Long.valueOf(input);
      }
      else if (clazz.isAssignableFrom(Double.class))
      {
         value = (T) Double.valueOf(input);
      }
      else if (clazz.isAssignableFrom(Float.class))
      {
         value = (T) Float.valueOf(input);
      }
      else
      {
         throw new IllegalArgumentException("Invalid type for TrajectoryRecordReplay. It only accepts primitive Number types.");
      }
      return value;
   }

   private <T> T concatenateWithCopy(T array1, T array2)
   {
      if (!array1.getClass().isArray() || !array2.getClass().isArray())
      {
         throw new IllegalArgumentException("Only arrays are accepted.");
      }

      Class<?> componentType1 = array1.getClass().getComponentType();
      Class<?> componentType2 = array2.getClass().getComponentType();

      if (!componentType1.equals(componentType2))
      {
         throw new IllegalArgumentException("Two arrays have different types.");
      }

      int len1 = Array.getLength(array1);
      int len2 = Array.getLength(array2);

      @SuppressWarnings("unchecked")
      // the cast is safe due to the previous checks
      T result = (T) Array.newInstance(componentType1, len1 + len2);

      System.arraycopy(array1, 0, result, 0, len1);
      System.arraycopy(array2, 0, result, len1, len2);

      return result;
   }

   public boolean hasSavedRecording()
   {
      return savedRecording;
   }

   public boolean hasDoneReplay()
   {
      return doneReplaying;
   }

   public void setDoneReplay(boolean doneReplaying)
   {
      this.doneReplaying = doneReplaying;
   }

   public String getPath()
   {
      return this.filePath;
   }

   public void setPath(String filePath)
   {
      this.filePath = filePath;
      this.reset();
   }

   public void setPath(String filePath, boolean reset)
   {
      this.filePath = filePath;
      if (reset)
         this.reset();
   }

   public void setNumberParts(int numberParts)
   {
      this.numberParts = numberParts;
   }

   public ArrayList<T[]> getData()
   {
      return dataMatrix;
   }

   public ArrayList<T[]> getConcatenatedData()
   {
      return concatenatedDataMatrix;
   }

   public String getRecordFileName()
   {
      return recordFileName;
   }

   public void setRecordFileName(String recordFileName)
   {
      this.recordFileName = recordFileName;
   }
}