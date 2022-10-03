package us.ihmc.gdx.ui.tools;

import org.apache.commons.lang3.tuple.MutablePair;

import java.io.*;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.IntStream;
import java.util.stream.Stream;

/**
 * Class to record and replay multi-dimensional trajectories of primitive Number types.
 * The multi-dimension represents the number of elements that are considered.
 * For each dimension, at each time step the class stores or reads a setpoint.
 * The collection of setpoints defines the trajectory.
 * The trajectories are saved and loaded from .csv files.
 * Each column represents the trajectory of a distinct element (e.g., X position of the right hand, or Y position of the center of mass, etc...).
 */
public class TrajectoryRecordReplay<T extends Number>
{
   private String filePath;
   private final Class<T> clazz;
   private ArrayList<T[]> dataMatrix = new ArrayList<>();
   private int timeStepReplay = 0;
   private boolean savedRecording = true;
   private boolean doneReplaying = true;

   public TrajectoryRecordReplay(Class<T> clazz, String filePath)
   {
      super();
      this.clazz = clazz;
      this.filePath = filePath;
   }

   public T[] play()
   {
      if (timeStepReplay < 1)
         this.readCSV();
      T[] values = dataMatrix.get(timeStepReplay);
      if (timeStepReplay >= dataMatrix.size() - 2)
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

   public void saveRecording()
   {
      writeCSV(dataMatrix);
      this.reset();
      savedRecording = true;
   }

   private void readCSV()
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

   private void writeCSV(ArrayList<T[]> dataMatrix)
   {
      List<String[]> dataLines = new ArrayList<>();
      for (T[] dataLine : dataMatrix)
      {
         String[] stringValues = new String[dataMatrix.get(0).length];
         Arrays.setAll(stringValues, j -> "" + dataLine[j]);
         dataLines.add(stringValues);
      }

      String fileName = new SimpleDateFormat("yyyyMMddHHmm'.csv'").format(new Date());
      File csvFile = new File(filePath + fileName);
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

   public boolean hasSavedRecording()
   {
      return savedRecording;
   }

   public boolean hasDoneReplay()
   {
      return doneReplaying;
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
}