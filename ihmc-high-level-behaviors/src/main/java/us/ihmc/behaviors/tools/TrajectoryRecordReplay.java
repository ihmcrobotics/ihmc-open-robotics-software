package us.ihmc.behaviors.tools;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.providers.BooleanProvider;

import java.io.*;
import java.text.SimpleDateFormat;
import java.util.*;
import java.util.function.IntPredicate;
import java.util.function.IntToDoubleFunction;

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
   private final List<double[]> concatenatedDataMatrix = new ArrayList<>();
   private final List<double[]> splitDataMatrix = new ArrayList<>();
   private int timeStepReplay = 0;
   private boolean savedRecording = true;
   private boolean doneReplaying = false;
   private boolean concatenated = false;
   private String recordFileName = "";
   private final List<JoystickData> joystickData = new ArrayList<>();
   private int replayIndex = 0;

   private long replayStartTimeMillis;
   private int lastReplayIndex = -1;

   public TrajectoryRecordReplay(String filePath, int numberParts)
   {
      this.filePath = filePath;
      this.numberOfParts = numberParts;
   }

   public double[] play()
   {
      return this.play(false);
   }

   public double[] play(boolean split)
   {
      if (timeStepReplay < 1)
      {
         this.readCSV();
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
    * Updates replay index and returns whether it is still replaying
    */
   public boolean onUpdateStartReplay()
   {
      if (replayIndex == -1)
      { // First replay tick, initialize
         replayIndex = 0;
         lastReplayIndex = -1;
         replayStartTimeMillis = System.currentTimeMillis();
         LogTools.info("Replay started");
         return true;
      }
      else
      { // Replaying, search for next index
         long timeInTrajectory = System.currentTimeMillis() - replayStartTimeMillis;
         lastReplayIndex = replayIndex;
         replayIndex = findCurrentIndex(timeInTrajectory);
         boolean isStillReplaying = replayIndex < joystickData.size() - 1;
         if (!isStillReplaying)
         {
            long duration = System.currentTimeMillis() - replayStartTimeMillis;
            LogTools.info("Done replaying... duration = " + duration + "ms");
         }
         return isStillReplaying;
      }
   }

   private int findCurrentIndex(long timeInTrajectory)
   {
      int searchIndex = lastReplayIndex;
      while (searchIndex < joystickData.size() && joystickData.get(searchIndex).timeInTrajectoryMillis < timeInTrajectory)
      {
         searchIndex++;
      }

      // grab the closer index
      if (searchIndex <= 0 || searchIndex >= joystickData.size() - 1)
         return searchIndex;

      long dtPrev = Math.abs(joystickData.get(searchIndex - 1).timeInTrajectoryMillis - timeInTrajectory);
      long dtNext = Math.abs(joystickData.get(searchIndex).timeInTrajectoryMillis - timeInTrajectory);
      return dtNext < dtPrev ? searchIndex : searchIndex - 1;
   }

   public void onUpdateStartRecord()
   {
      if (joystickData.isEmpty())
      { // First record tick, initialize
         replayStartTimeMillis = System.currentTimeMillis();
      }

      long timeInTrajectoryMillis = System.currentTimeMillis() - replayStartTimeMillis;
      joystickData.add(new JoystickData(timeInTrajectoryMillis));
   }

   public void recordControllerData(RobotSide robotSide,
                                    boolean aButtonPressed,
                                    boolean triggerPressed,
                                    ReferenceFrame desiredControlFrame,
                                    ReferenceFrame recordFrame)
   {
      joystickData.get(joystickData.size() - 1).set(robotSide, aButtonPressed, triggerPressed, desiredControlFrame, recordFrame);
   }

   public void recordDesiredCenterOfMass(FramePoint3DReadOnly usedDesiredCenterOfMass, ReferenceFrame recordFrame)
   {
      FramePoint3D desiredCenterOfMass = joystickData.get(joystickData.size() - 1).desiredCenterOfMass;
      desiredCenterOfMass.setIncludingFrame(usedDesiredCenterOfMass);
      desiredCenterOfMass.changeFrame(recordFrame);
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

   public void onRecordStart()
   {
      joystickData.clear();
   }

   public void onRecordEnd()
   {
      if (joystickData.isEmpty())
         return;

      // save recording
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
      try (FileWriter writer = new FileWriter(csvFile))
      {
         for (int row = 0; row < this.joystickData.size(); row++)
         {
            JoystickData joystickData = this.joystickData.get(row);
            writer.write(joystickData.toString());
         }
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

      long duration = System.currentTimeMillis() - replayStartTimeMillis;
      LogTools.info("Record ended. Duration = " + duration + "ms");
   }

   public boolean onReplayStart(String replayFileToLoad, ReferenceFrame loadInFrame)
   {
      joystickData.clear();
      File replayFile = new File(replayFileToLoad);
      replayIndex = -1;
      replayStartTimeMillis = -1;

      try
      {
         BufferedReader fileReader = new BufferedReader(new FileReader(replayFile));
         String line;
         while ((line = fileReader.readLine()) != null)
         {
            joystickData.add(new JoystickData(line.split(","), loadInFrame));
         }
         fileReader.close();

         return true;
      }
      catch (Exception e)
      {
         e.printStackTrace();

         return false;
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
         String line;
         while ((line = fileReader.readLine()) != null)
         {
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

   private void reset()
   {
      timeStepReplay = 0;
      dataMatrix.clear();
      concatenated = false;
      concatenatedDataMatrix.clear();
      splitDataMatrix.clear();
      recordFileName = "";
      replayStartTimeMillis = -1;
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
   }

   public void setPath(String filePath, boolean reset)
   {
      this.filePath = filePath;
      if (reset)
         this.reset();
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

   public String getRecordFileName()
   {
      return recordFileName;
   }

   public void setRecordFileName(String recordFileName)
   {
      this.recordFileName = recordFileName;
   }

   private static class JoystickData
   {
      private boolean leftAButtonPressed;
      private boolean leftTriggerPressed;
      private final FramePose3D leftDesiredControllerPose;

      private boolean rightAButtonPressed;
      private boolean rightTriggerPressed;
      private final FramePose3D rightDesiredControllerPose;

      private final long timeInTrajectoryMillis;
      private final FramePoint3D desiredCenterOfMass;

      public JoystickData(long timeInTrajectoryMillis)
      {
         leftDesiredControllerPose = new FramePose3D();
         rightDesiredControllerPose = new FramePose3D();
         desiredCenterOfMass = new FramePoint3D();
         this.timeInTrajectoryMillis = timeInTrajectoryMillis;
      }

      public JoystickData(String[] data, ReferenceFrame loadInFrame)
      {
         int index = 0;

         leftAButtonPressed = Boolean.parseBoolean(data[index++]);
         leftTriggerPressed = Boolean.parseBoolean(data[index++]);
         leftDesiredControllerPose = new FramePose3D(loadInFrame);
         leftDesiredControllerPose.getOrientation().set(Double.parseDouble(data[index++]), Double.parseDouble(data[index++]), Double.parseDouble(data[index++]), Double.parseDouble(data[index++]));
         leftDesiredControllerPose.getPosition().set(Double.parseDouble(data[index++]), Double.parseDouble(data[index++]), Double.parseDouble(data[index++]));
         leftDesiredControllerPose.changeFrame(ReferenceFrame.getWorldFrame());

         rightAButtonPressed = Boolean.parseBoolean(data[index++]);
         rightTriggerPressed = Boolean.parseBoolean(data[index++]);
         rightDesiredControllerPose = new FramePose3D(loadInFrame);
         rightDesiredControllerPose.getOrientation().set(Double.parseDouble(data[index++]), Double.parseDouble(data[index++]), Double.parseDouble(data[index++]), Double.parseDouble(data[index++]));
         rightDesiredControllerPose.getPosition().set(Double.parseDouble(data[index++]), Double.parseDouble(data[index++]), Double.parseDouble(data[index++]));
         rightDesiredControllerPose.changeFrame(ReferenceFrame.getWorldFrame());

         String timestampToParse = data[index++];
         timeInTrajectoryMillis = Long.parseLong(timestampToParse);
         desiredCenterOfMass = new FramePoint3D(loadInFrame, Double.parseDouble(data[index++]), Double.parseDouble(data[index++]), Double.parseDouble(data[index++]));
      }

      void set(RobotSide robotSide,
               boolean aButtonPressed,
               boolean triggerPressed,
               ReferenceFrame desiredControlFrame,
               ReferenceFrame recordFrame)
      {
         if (robotSide == RobotSide.LEFT)
         {
            leftAButtonPressed = aButtonPressed;
            leftTriggerPressed = triggerPressed;
            leftDesiredControllerPose.setToZero(desiredControlFrame);
            leftDesiredControllerPose.changeFrame(recordFrame);
         }
         else
         {
            rightAButtonPressed = aButtonPressed;
            rightTriggerPressed = triggerPressed;
            rightDesiredControllerPose.setToZero(desiredControlFrame);
            rightDesiredControllerPose.changeFrame(recordFrame);
         }
      }

      @Override
      public String toString()
      {
         return leftAButtonPressed + "," +
                leftTriggerPressed + "," +
                leftDesiredControllerPose.getOrientation().getX() + "," +
                leftDesiredControllerPose.getOrientation().getY() + "," +
                leftDesiredControllerPose.getOrientation().getZ() + "," +
                leftDesiredControllerPose.getOrientation().getS() + "," +
                leftDesiredControllerPose.getPosition().getX() + "," +
                leftDesiredControllerPose.getPosition().getY() + "," +
                leftDesiredControllerPose.getPosition().getZ() + "," +

                rightAButtonPressed + "," +
                rightTriggerPressed + "," +
                rightDesiredControllerPose.getOrientation().getX() + "," +
                rightDesiredControllerPose.getOrientation().getY() + "," +
                rightDesiredControllerPose.getOrientation().getZ() + "," +
                rightDesiredControllerPose.getOrientation().getS() + "," +
                rightDesiredControllerPose.getPosition().getX() + "," +
                rightDesiredControllerPose.getPosition().getY() + "," +
                rightDesiredControllerPose.getPosition().getZ() + "," +

                timeInTrajectoryMillis + "," +
                desiredCenterOfMass.getX() + "," +
                desiredCenterOfMass.getY() + "," +
                desiredCenterOfMass.getZ() + "\n";
      }
   }

   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   ///////////////////////////////////////////////////   Getters for button presses   /////////////////////////////////////////////////////
   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

   public boolean getAButtonPressed(RobotSide robotSide)
   {
      return checkButtonPressOverInterval(lastReplayIndex, replayIndex, i -> getAButtonPressed(robotSide, i, joystickData));
   }

   public boolean getTriggerPressed(RobotSide robotSide)
   {
      return checkButtonPressOverInterval(lastReplayIndex, replayIndex, i -> getTriggerPressed(robotSide, i, joystickData));
   }

   public void packDesiredHandControlFrame(RobotSide robotSide, FramePose3D poseToPack)
   {
      JoystickData joystickData = this.joystickData.get(replayIndex);
      poseToPack.set(robotSide == RobotSide.LEFT ? joystickData.leftDesiredControllerPose : joystickData.rightDesiredControllerPose);
   }

   public FramePoint3D getDesiredCenterOfMass()
   {
      JoystickData joystickData = this.joystickData.get(replayIndex);
      return joystickData.desiredCenterOfMass;
   }

   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   /////////////////////////////////////////////////////////   Helper methods   ///////////////////////////////////////////////////////////
   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

   private static boolean getAButtonPressed(RobotSide robotSide, int index, List<JoystickData> joystickDataList)
   {
      JoystickData joystickData = joystickDataList.get(index);
      return robotSide == RobotSide.LEFT ? joystickData.leftAButtonPressed : joystickData.rightAButtonPressed;
   }

   private static boolean getTriggerPressed(RobotSide robotSide, int index, List<JoystickData> joystickDataList)
   {
      JoystickData joystickData = joystickDataList.get(index);
      return robotSide == RobotSide.LEFT ? joystickData.leftTriggerPressed : joystickData.rightTriggerPressed;
   }

   private static boolean checkButtonPressOverInterval(int indexStartExclusive, int indexEndInclusive, IntPredicate buttonPressProvider)
   {
      for (int index = indexEndInclusive; index > indexStartExclusive; index--)
      {
         if (buttonPressProvider.test(index))
            return true;
      }
      return false;
   }

   public static void main(String[] args)
   {
      System.out.println(Long.parseLong("156"));
      System.out.println(Double.parseDouble("NaN"));
   }
}