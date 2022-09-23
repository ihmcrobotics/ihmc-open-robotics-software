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

public class TrajectoryRecordReplay<T extends Number>
{
   private String filePath;
   private ArrayList<T[]> dataMatrix = new ArrayList<>();
   private int timeStepReplay = 0;
   private boolean savedRecording = true;
   private boolean doneReplay= true;

   public TrajectoryRecordReplay(String filePath)
   {
      this.filePath = filePath;
   }

   public T[] play(){
      if (timeStepReplay<1)
         this.readCSV();
      T[] values = dataMatrix.get(timeStepReplay);
      if (timeStepReplay>=dataMatrix.size()-2){
         doneReplay=true;
         return values;
      }
      else{
         timeStepReplay++;
         return values;
      }
   }

   public void record(T[] values)
   {
      if (savedRecording)
         savedRecording=false;
      T[] localValues = (T[]) new Double[values.length];
      System.arraycopy(values, 0, localValues, 0, localValues.length);
      dataMatrix.add(localValues);
   }

   public void saveRecording(){
      writeCSV(dataMatrix);
      this.reset();
      savedRecording = true;
   }

   private void readCSV()
   {
      doneReplay=false;
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
   }

   private void reset(){
      timeStepReplay = 0;
      dataMatrix.clear();
   }

   private void writeCSV(ArrayList<T[]> dataMatrix)
   {
      List<String[]> dataLines = new ArrayList<>();
      for (T[] d : dataMatrix)
      {
         String[] sVals = new String[dataMatrix.get(0).length];
         Arrays.setAll(sVals, j -> "" + d[j]);
         dataLines.add(sVals);
      }

      String fileName = new SimpleDateFormat("yyyyMMddHHmm'.csv'").format(new Date());
      File csvFile = new File(filePath+fileName);
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

   public boolean hasDoneReplay(){
      return doneReplay;
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