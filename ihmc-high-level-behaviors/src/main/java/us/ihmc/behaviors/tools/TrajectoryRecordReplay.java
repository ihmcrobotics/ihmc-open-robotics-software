package us.ihmc.behaviors.tools;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

import java.io.*;
import java.text.SimpleDateFormat;
import java.util.*;

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

   public void onUpdateStart()
   {
      joystickData.add(new JoystickData());
   }

   public void recordControllerData(RobotSide robotSide,
                                    boolean aButtonPressed,
                                    boolean triggerPressed,
                                    double forwardJoystickValue,
                                    double lateralJoystickValue,
                                    ReferenceFrame desiredControlFrame,
                                    ReferenceFrame recordInFrame)
   {
      joystickData.get(joystickData.size() - 1).set(robotSide, aButtonPressed, triggerPressed, forwardJoystickValue, lateralJoystickValue, desiredControlFrame, recordInFrame);
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

   public boolean onUpdateEnd()
   {
      replayIndex++;
      return replayIndex < joystickData.size();
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
   }

   public boolean onReplayStart(String replayFileToLoad, ReferenceFrame loadInFrame)
   {
      replayIndex = 0;
      joystickData.clear();
      File replayFile = new File(replayFileToLoad);

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

   public String getRecordFileName()
   {
      return recordFileName;
   }

   public void setRecordFileName(String recordFileName)
   {
      this.recordFileName = recordFileName;
   }

   private class JoystickData
   {
      private boolean leftAButtonPressed;
      private boolean leftTriggerPressed;
      private double leftForwardJoystickValue;
      private double leftLateralJoystickValue;
      private final FramePose3D leftDesiredControllerPose;

      private boolean rightAButtonPressed;
      private boolean rightTriggerPressed;
      private double rightForwardJoystickValue;
      private double rightLateralJoystickValue;
      private final FramePose3D rightDesiredControllerPose;

      public JoystickData()
      {
         leftDesiredControllerPose = new FramePose3D();
         rightDesiredControllerPose = new FramePose3D();
      }

      public JoystickData(String[] data, ReferenceFrame loadInFrame)
      {
         int index = 0;

         leftAButtonPressed = Boolean.parseBoolean(data[index++]);
         leftTriggerPressed = Boolean.parseBoolean(data[index++]);
         leftForwardJoystickValue = Double.parseDouble(data[index++]);
         leftLateralJoystickValue = Double.parseDouble(data[index++]);
         leftDesiredControllerPose = new FramePose3D(loadInFrame);
         leftDesiredControllerPose.getOrientation().set(Double.parseDouble(data[index++]), Double.parseDouble(data[index++]), Double.parseDouble(data[index++]), Double.parseDouble(data[index++]));
         leftDesiredControllerPose.getPosition().set(Double.parseDouble(data[index++]), Double.parseDouble(data[index++]), Double.parseDouble(data[index++]));
         leftDesiredControllerPose.changeFrame(ReferenceFrame.getWorldFrame());

         rightAButtonPressed = Boolean.parseBoolean(data[index++]);
         rightTriggerPressed = Boolean.parseBoolean(data[index++]);
         rightForwardJoystickValue = Double.parseDouble(data[index++]);
         rightLateralJoystickValue = Double.parseDouble(data[index++]);
         rightDesiredControllerPose = new FramePose3D(loadInFrame);
         rightDesiredControllerPose.getOrientation().set(Double.parseDouble(data[index++]), Double.parseDouble(data[index++]), Double.parseDouble(data[index++]), Double.parseDouble(data[index++]));
         rightDesiredControllerPose.getPosition().set(Double.parseDouble(data[index++]), Double.parseDouble(data[index++]), Double.parseDouble(data[index++]));
         rightDesiredControllerPose.changeFrame(ReferenceFrame.getWorldFrame());
      }

      void set(RobotSide robotSide,
               boolean aButtonPressed,
               boolean triggerPressed,
               double forwardJoystickValue,
               double lateralJoystickValue,
               ReferenceFrame desiredControlFrame,
               ReferenceFrame recordInFrame)
      {
         if (robotSide == RobotSide.LEFT)
         {
            leftAButtonPressed = aButtonPressed;
            leftTriggerPressed = triggerPressed;
            leftForwardJoystickValue = forwardJoystickValue;
            leftLateralJoystickValue = lateralJoystickValue;
            leftDesiredControllerPose.setToZero(desiredControlFrame);
            leftDesiredControllerPose.changeFrame(recordInFrame);
         }
         else
         {
            rightAButtonPressed = aButtonPressed;
            rightTriggerPressed = triggerPressed;
            rightForwardJoystickValue = forwardJoystickValue;
            rightLateralJoystickValue = lateralJoystickValue;
            rightDesiredControllerPose.setToZero(desiredControlFrame);
            rightDesiredControllerPose.changeFrame(recordInFrame);
         }
      }

      void set(RobotSide robotSide,
               boolean aButtonPressed,
               boolean triggerPressed,
               double forwardJoystickValue,
               double lateralJoystickValue,
               ReferenceFrame desiredControlFrame)
      {
         set(robotSide, aButtonPressed, triggerPressed, forwardJoystickValue, lateralJoystickValue, desiredControlFrame, ReferenceFrame.getWorldFrame());
      }

      @Override
      public String toString()
      {
         return leftAButtonPressed + "," +
                leftTriggerPressed + "," +
                leftForwardJoystickValue + "," +
                leftLateralJoystickValue + "," +
                leftDesiredControllerPose.getOrientation().getX() + "," +
                leftDesiredControllerPose.getOrientation().getY() + "," +
                leftDesiredControllerPose.getOrientation().getZ() + "," +
                leftDesiredControllerPose.getOrientation().getS() + "," +
                leftDesiredControllerPose.getPosition().getX() + "," +
                leftDesiredControllerPose.getPosition().getY() + "," +
                leftDesiredControllerPose.getPosition().getZ() + "," +

                rightAButtonPressed + "," +
                rightTriggerPressed + "," +
                rightForwardJoystickValue + "," +
                rightLateralJoystickValue + "," +
                rightDesiredControllerPose.getOrientation().getX() + "," +
                rightDesiredControllerPose.getOrientation().getY() + "," +
                rightDesiredControllerPose.getOrientation().getZ() + "," +
                rightDesiredControllerPose.getOrientation().getS() + "," +
                rightDesiredControllerPose.getPosition().getX() + "," +
                rightDesiredControllerPose.getPosition().getY() + "," +
                rightDesiredControllerPose.getPosition().getZ() + "\n";
      }
   }

   public boolean getAButtonPressed(RobotSide robotSide)
   {
      JoystickData joystickData = this.joystickData.get(replayIndex);
      return robotSide == RobotSide.LEFT ? joystickData.leftAButtonPressed : joystickData.rightAButtonPressed;
   }

   public boolean getTriggerPressed(RobotSide robotSide)
   {
      JoystickData joystickData = this.joystickData.get(replayIndex);
      return robotSide == RobotSide.LEFT ? joystickData.leftTriggerPressed : joystickData.rightTriggerPressed;
   }

   public double getForwardJoystick(RobotSide robotSide)
   {
      JoystickData joystickData = this.joystickData.get(replayIndex);
      return robotSide == RobotSide.LEFT ? joystickData.leftForwardJoystickValue : joystickData.rightForwardJoystickValue;
   }

   public double getLateralJoystick(RobotSide robotSide)
   {
      JoystickData joystickData = this.joystickData.get(replayIndex);
      return robotSide == RobotSide.LEFT ? joystickData.leftLateralJoystickValue : joystickData.rightLateralJoystickValue;
   }

   public void packDesiredHandControlFrame(RobotSide robotSide, FramePose3D poseToPack)
   {
      JoystickData joystickData = this.joystickData.get(replayIndex);
      poseToPack.set(robotSide == RobotSide.LEFT ? joystickData.leftDesiredControllerPose : joystickData.rightDesiredControllerPose);
   }
}