package us.ihmc.avatar.networkProcessor.referenceSpreading;

import us.ihmc.log.LogTools;

import java.io.*;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.HashMap;
import java.util.List;

/**
 * Class to record and replay multidimensional trajectories.
 * The multi-dimension represents the number of elements that are considered.
 * For each dimension, at each time step the class stores or reads a set point.
 * The collection of set points defines the trajectory.
 * The trajectories are saved and loaded from .csv files.
 * Each column represents the trajectory of a distinct element (e.g., X position of the right hand, or Y position of the right hand, etc...).
 */
public class TrajectoryRecordReplay
{
   private String filePath;
   private int numberOfParts; // specify the number of parts you want to record (e.g., left hand, right hand, chest)
   private final List<double[]> dataMatrix = new ArrayList<>();
   private final List<double[]> dataMatrixSaved = new ArrayList<>();
   private final List<String> keyMatrix = new ArrayList<>();
   private final List<double[]> concatenatedDataMatrix = new ArrayList<>();
   private final List<double[]> splitDataMatrix = new ArrayList<>();
   private int timeStepReplay = 0;
   private boolean savedRecording = true;
   private boolean doneReplaying = false;
   private boolean concatenated = false;
   private String recordFileName = "";
   private boolean createKeyMap = false;


   public TrajectoryRecordReplay(String filePath, int numberParts)
   {
      this(filePath, numberParts, false);
   }

   public TrajectoryRecordReplay(String filePath, int numberParts, boolean createKeyMap)
   {
      this.filePath = filePath;
      this.numberOfParts = numberParts;
      this.createKeyMap = createKeyMap;
   }

   public double[] play()
   {
      return this.play(false);
   }

   public double[] play(boolean split)
   {
      if (timeStepReplay < 1 && dataMatrix.isEmpty())
      {
         if (dataMatrixSaved.isEmpty())
            this.readCSV();
         else
         {
            dataMatrix.addAll(dataMatrixSaved);
            doneReplaying = false;
         }
         if (split)
            this.splitData();
      }
      double[] values;
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

   public void record(double[] values)
   {
      if (savedRecording)
         savedRecording = false;
      double[] localValues = new double[values.length];
      System.arraycopy(values, 0, localValues, 0, localValues.length);
      dataMatrix.add(localValues);
   }

   /**
    * Useful if we are recording trajectories of different parts but not in the same scope
    * and we want to concatenate them into one single row to have a single csv file
    * rather than having multiple TrajectoryRecordReplay objects and multiple csv files
    */
   public void concatenateData()
   {
      for (int i = 0; i < dataMatrix.size(); i = i + numberOfParts)
      {
         double[] concatenatedRow = dataMatrix.get(i);
         for (int j = 1; j <= numberOfParts - 1; j++)
         {
            concatenatedRow = concatenateWithCopy(concatenatedRow, dataMatrix.get(i + j));
         }
         concatenatedDataMatrix.add(concatenatedRow);
      }
      concatenated = true;
   }

   public void importData(boolean split)
   {
      this.readCSV();
      if (split)
         this.splitData();
   }

   /**
    * Useful if we are replaying a csv file where multiple parts have been concatenated in one single row
    * and we want the info of each part in a separate row.
    * Not useful if you have different parts with different number of elements
    */
   private void splitData()
   {
      for (int i = 0; i < dataMatrix.size(); i++)
      {
         double[] row = dataMatrix.get(i);
         for (int n = 0; n <= numberOfParts - 1; n++)
         {
            double[] splitRow = new double[row.length / numberOfParts];
            for (int j = 0; j < splitRow.length; j++)
            {
               splitRow[j] = row[j + n * splitRow.length];
            }
            splitDataMatrix.add(splitRow);
         }
      }
   }

   public void clearRecordingMemory()
   {
      dataMatrixSaved.clear();
      this.reset();
   }

   public void saveRecordingMemory()
   {
      dataMatrixSaved.clear();
      dataMatrixSaved.addAll(dataMatrix);
      this.reset();

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
         String line;
         boolean createKeyMapTemp = this.createKeyMap;
         while ((line = fileReader.readLine()) != null)
         {
            if (createKeyMapTemp){
               String[] keys = line.split(",");
               keyMatrix.addAll(Arrays.asList(keys));
               parseKeyMatrix(keyMatrix);
               createKeyMapTemp = false;
               continue;
            }

            String[] values = line.split(",");
            double[] dataValues = new double[values.length];
            for (int i = 0; i < values.length; i++)
            {
               dataValues[i] = Double.parseDouble(values[i]);
            }
            dataMatrix.add(dataValues);
         }
         fileReader.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public void writeCSV(List<double[]> dataMatrix)
   {
      // if recordFile name has not been set, generate file with current date and time as name
      String fileName = "";
      if (recordFileName.isEmpty())
      {
         fileName = new SimpleDateFormat("yyMMddHHmmssZ'.csv'").format(new Date());
         recordFileName = fileName;
      }
      else
         fileName = recordFileName;
      File csvFile = new File(filePath + "/" + fileName);
      try (PrintWriter writer = new PrintWriter(csvFile))
      {
         for (int row = 0; row < dataMatrix.size(); row++)
         {
            double[] dataLine = dataMatrix.get(row);
            for (int col = 0; col < dataLine.length; col++)
            {
               writer.print(dataLine[col]);
               if (col < dataLine.length - 1)
                  writer.append(",");
            }
            if (row < dataMatrix.size() - 1)
            {
               writer.println();
            }
         }
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public void reset()
   {
      timeStepReplay = 0;
      dataMatrix.clear();
      concatenated = false;
      concatenatedDataMatrix.clear();
      splitDataMatrix.clear();
      recordFileName = "";
   }

   private double[] concatenateWithCopy(double[] array1, double[] array2)
   {
      double[] result = new double[array1.length + array2.length];
      System.arraycopy(array1, 0, result, 0, array1.length);
      System.arraycopy(array2, 0, result, array1.length, array2.length);

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

   public void setNumberOfParts(int numberOfParts)
   {
      this.numberOfParts = numberOfParts;
   }

   public List<double[]> getData()
   {
      return dataMatrix;
   }

   public List<double[]> getConcatenatedData()
   {
      return concatenatedDataMatrix;
   }

   public int getTimeStepReplay()
   {
      return timeStepReplay;
   }

   public int getNumberOfLines() { return dataMatrix.size(); }

   public String getRecordFileName()
   {
      return recordFileName;
   }

   public void setRecordFileName(String recordFileName)
   {
      this.recordFileName = recordFileName;
   }

   public List<String> getKeyMatrix()
   {
      return keyMatrix;
   }

   private void parseKeyMatrix(List<String> keyMatrixToParse)
   {
      for (int i = 0; i < keyMatrixToParse.size(); i++)
      {
         String key = keyMatrixToParse.get(i);
         int lastDotIndex = key.lastIndexOf('.');
         if (lastDotIndex != -1)
         {
            key = key.substring(lastDotIndex + 1);
         }
         keyMatrixToParse.set(i, key);
      }
   }
}