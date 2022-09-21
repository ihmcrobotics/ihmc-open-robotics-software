package us.ihmc.gdx.ui.tools;

import org.apache.commons.lang3.tuple.MutablePair;

import java.io.*;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.IntStream;
import java.util.stream.Stream;

public class TrajectoryRecordReplay<T extends Number>
{
   private String filePath;
   private ArrayList<T[]> dataMatrix = new ArrayList<>();
   private int timeStepReplay = 0;
   private boolean savedRecording = false;

   public TrajectoryRecordReplay(String filePath)
   {
      this.filePath = filePath;
   }

   public MutablePair<Boolean, T[]> play(){
      if (timeStepReplay<1)
         dataMatrix = this.readCSV();
      MutablePair<Boolean, T[]> values = new MutablePair<>(true, dataMatrix.get(timeStepReplay));
      if (timeStepReplay>=dataMatrix.size()-2)
      {
         values.setLeft(false);
         return values;
      }
      else
         values.setRight(dataMatrix.get(timeStepReplay));
      timeStepReplay++;
      return values;
   }

   public void record(T[] values)
   {
      savedRecording = false;
      dataMatrix.add(values);
   }

   public void saveRecording(){
      writeCSV(dataMatrix);
      this.reset();
      savedRecording = true;
   }

   private ArrayList<T[]> readCSV()
   {
      try {
         BufferedReader fileReader = new BufferedReader(new FileReader(filePath));
         String line = "";
         while((line = fileReader.readLine()) != null) {
            String[] stringValues = line.split(",");
            T[] dataValues = (T[]) new Double[stringValues.length];
            IntStream.range(0, stringValues.length).forEach(i -> dataValues[i] = (T) Double.valueOf(stringValues[i]));
            dataMatrix.add(dataValues);
         }
         fileReader.close();
      }
      catch (IOException e) {
         e.printStackTrace();
      }
      return dataMatrix;
   }

   private void reset(){
      timeStepReplay = 0;
      dataMatrix.clear();
   }

   private void writeCSV(ArrayList<T[]> dataMatrix)
   {
      List<String[]> dataLines = new ArrayList<>();
      String[] sVals = new String[dataMatrix.get(0).length];
      for (T[] d : dataMatrix)
      {
         Arrays.setAll(sVals, j -> "" + d[j]);
         dataLines.add(sVals);
      }

      File csvFile = new File(filePath);
      try (PrintWriter pw = new PrintWriter(csvFile)) {
         dataLines.stream()
                  .map(this::convertToCSV)
                  .forEach(pw::println);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   /* create a method for formatting a single line of data represented as an array of Strings */
   private String convertToCSV(String[] data)
   {
      return Stream.of(data)
                   .map(this::escapeSpecialCharacters)
                   .collect(Collectors.joining(","));
   }

   private String escapeSpecialCharacters(String data)
   {
      String escapedData = data.replaceAll("\\R", " ");
      if (data.contains(",") || data.contains("\"") || data.contains("'")) {
         data = data.replace("\"", "\"\"");
         escapedData = "\"" + data + "\"";
      }
      return escapedData;
   }

   public boolean hasSavedRecording(){
      return savedRecording;
   }


   public String getPath()
   {
      return this.filePath;
   }

   public void setPath(String filePath)
   {
      this.filePath = filePath;
   }
}