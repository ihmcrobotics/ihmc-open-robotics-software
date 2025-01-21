package us.ihmc.behaviors.tools;

import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KSTTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.ThreadTimer;

import java.awt.*;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Paths;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.function.IntPredicate;

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
   private double replayInterpolationAlpha = -1.0;

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

      // if first or last index, return it
      if (searchIndex <= 0 || searchIndex >= joystickData.size() - 1)
      {
         replayInterpolationAlpha = -1.0;
         return searchIndex;
      }

      // compute the interpolation between the two adjacent frames to smoothly replay
      long t0 = joystickData.get(searchIndex - 1).timeInTrajectoryMillis;
      long t1 = joystickData.get(searchIndex).timeInTrajectoryMillis;
      replayInterpolationAlpha = ((double) (timeInTrajectory - t0)) / (t1 - t0);

      return searchIndex;
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
                                    boolean bButtonPressed,
                                    boolean triggerPressed,
                                    ReferenceFrame desiredControlFrame,
                                    Vector3D angularVelocity,
                                    Vector3D linearVelocity,
                                    ReferenceFrame recordFrame)
   {
      joystickData.get(joystickData.size() - 1)
                  .set(robotSide, aButtonPressed, bButtonPressed, triggerPressed, desiredControlFrame, angularVelocity, linearVelocity, recordFrame);
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

   public boolean onReplayStart(String replayFileToLoad, ReferenceFrame recordFrame)
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
            joystickData.add(new JoystickData(line.split(","), recordFrame));
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
      private final long timeInTrajectoryMillis;

      private boolean leftAButtonPressed;
      private boolean leftBButtonPressed;
      private boolean leftTriggerPressed;

      private boolean rightAButtonPressed;
      private boolean rightBButtonPressed;
      private boolean rightTriggerPressed;

      private final SideDependentList<FramePose3D> controllerPoses = new SideDependentList<>(new FramePose3D(), new FramePose3D());
      private final SideDependentList<FrameVector3D> controllerAngularVelocities = new SideDependentList<>(new FrameVector3D(), new FrameVector3D());
      private final SideDependentList<FrameVector3D> controllerLinearVelocities = new SideDependentList<>(new FrameVector3D(), new FrameVector3D());
      private final FramePoint3D desiredCenterOfMass = new FramePoint3D();

      public JoystickData(long timeInTrajectoryMillis)
      {
         this.timeInTrajectoryMillis = timeInTrajectoryMillis;
      }

      public JoystickData(String[] data, ReferenceFrame recordFrame)
      {
         int index = 0;

         timeInTrajectoryMillis = Long.parseLong(data[index++]);

         leftAButtonPressed = Boolean.parseBoolean(data[index++]);
         leftBButtonPressed = Boolean.parseBoolean(data[index++]);
         leftTriggerPressed = Boolean.parseBoolean(data[index++]);

         rightAButtonPressed = Boolean.parseBoolean(data[index++]);
         rightBButtonPressed = Boolean.parseBoolean(data[index++]);
         rightTriggerPressed = Boolean.parseBoolean(data[index++]);

         for (RobotSide robotSide : RobotSide.values())
         {
            index = loadCSV(index, data, controllerPoses.get(robotSide), recordFrame, ReferenceFrame.getWorldFrame());
            index = loadCSV(index, data, controllerAngularVelocities.get(robotSide), recordFrame, ReferenceFrame.getWorldFrame());
            index = loadCSV(index, data, controllerLinearVelocities.get(robotSide), recordFrame, ReferenceFrame.getWorldFrame());
         }

         loadCSV(index, data, desiredCenterOfMass, recordFrame, ReferenceFrame.getWorldFrame());
      }

      void set(RobotSide robotSide,
               boolean aButtonPressed,
               boolean bButtonPressed,
               boolean triggerPressed,
               ReferenceFrame controllerReferenceFrame,
               Vector3D controllerAngularVelocity,
               Vector3D controllerLinearVelocity,
               ReferenceFrame recordFrame)
      {
         if (robotSide == RobotSide.LEFT)
         {
            leftAButtonPressed = aButtonPressed;
            leftBButtonPressed = bButtonPressed;
            leftTriggerPressed = triggerPressed;
         }
         else
         {
            rightAButtonPressed = aButtonPressed;
            rightBButtonPressed = bButtonPressed;
            rightTriggerPressed = triggerPressed;
         }

         controllerPoses.get(robotSide).setToZero(controllerReferenceFrame);
         controllerPoses.get(robotSide).changeFrame(recordFrame);
         controllerAngularVelocities.get(robotSide).set(ReferenceFrame.getWorldFrame(), controllerAngularVelocity);
         controllerAngularVelocities.get(robotSide).changeFrame(recordFrame);
         controllerLinearVelocities.get(robotSide).set(ReferenceFrame.getWorldFrame(), controllerLinearVelocity);
         controllerLinearVelocities.get(robotSide).changeFrame(recordFrame);
      }

      @Override
      public String toString()
      {
         return timeInTrajectoryMillis + "," +
                leftAButtonPressed + "," +
                leftBButtonPressed + "," +
                leftTriggerPressed + "," +
                rightAButtonPressed + "," +
                rightBButtonPressed + "," +
                rightTriggerPressed + "," +
                toCSV(controllerPoses.get(RobotSide.LEFT).getOrientation(), false) +
                toCSV(controllerPoses.get(RobotSide.LEFT).getPosition(), false) +
                toCSV(controllerAngularVelocities.get(RobotSide.LEFT), false) +
                toCSV(controllerLinearVelocities.get(RobotSide.LEFT), false) +
                toCSV(controllerPoses.get(RobotSide.RIGHT).getOrientation(), false) +
                toCSV(controllerPoses.get(RobotSide.RIGHT).getPosition(), false) +
                toCSV(controllerAngularVelocities.get(RobotSide.RIGHT), false) +
                toCSV(controllerLinearVelocities.get(RobotSide.RIGHT), false) +
                toCSV(desiredCenterOfMass, true);
      }
   }

   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   ///////////////////////////////////////////////////   Getters for button presses   /////////////////////////////////////////////////////
   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

   public boolean getAButtonPressed(RobotSide robotSide)
   {
      return checkButtonPressOverInterval(lastReplayIndex, replayIndex, i -> getAButtonPressed(robotSide, i, joystickData));
   }

   public boolean getBButtonPressed(RobotSide robotSide)
   {
      return checkButtonPressOverInterval(lastReplayIndex, replayIndex, i -> getBButtonPressed(robotSide, i, joystickData));
   }

   public boolean getTriggerPressed(RobotSide robotSide)
   {
      return checkButtonPressOverInterval(lastReplayIndex, replayIndex, i -> getTriggerPressed(robotSide, i, joystickData));
   }

   public void packControllerData(RobotSide robotSide, FramePose3D poseToPack, FrameVector3D angularVelocityToPack, FrameVector3D linearVelocityToPack)
   {
      if (replayInterpolationAlpha == -1.0)
      {
         JoystickData joystickData = this.joystickData.get(replayIndex);
         poseToPack.set(joystickData.controllerPoses.get(robotSide));
         angularVelocityToPack.set(joystickData.controllerAngularVelocities.get(robotSide));
         linearVelocityToPack.set(joystickData.controllerLinearVelocities.get(robotSide));
      }
      else
      {
         JoystickData joystickData0 = joystickData.get(replayIndex - 1);
         JoystickData joystickData1 = joystickData.get(replayIndex);

         poseToPack.interpolate(joystickData0.controllerPoses.get(robotSide), joystickData1.controllerPoses.get(robotSide), replayInterpolationAlpha);
         angularVelocityToPack.interpolate(joystickData0.controllerAngularVelocities.get(robotSide), joystickData1.controllerAngularVelocities.get(robotSide), replayInterpolationAlpha);
         linearVelocityToPack.interpolate(joystickData0.controllerLinearVelocities.get(robotSide), joystickData1.controllerLinearVelocities.get(robotSide), replayInterpolationAlpha);
      }
   }

   public FramePoint3D getDesiredCenterOfMass()
   {
      if (replayInterpolationAlpha == -1.0)
      {
         JoystickData joystickData = this.joystickData.get(replayIndex);
         return joystickData.desiredCenterOfMass;
      }
      else
      {
         JoystickData joystickData0 = this.joystickData.get(replayIndex);
         JoystickData joystickData1 = this.joystickData.get(replayIndex - 1);

         FramePoint3D desiredCenterOfMass = new FramePoint3D(joystickData0.desiredCenterOfMass);
         desiredCenterOfMass.interpolate(joystickData1.desiredCenterOfMass, replayInterpolationAlpha);

         return desiredCenterOfMass;
      }
   }

   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   /////////////////////////////////////////////////////////   Helper methods   ///////////////////////////////////////////////////////////
   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

   private static boolean getAButtonPressed(RobotSide robotSide, int index, List<JoystickData> joystickDataList)
   {
      JoystickData joystickData = joystickDataList.get(index);
      return robotSide == RobotSide.LEFT ? joystickData.leftAButtonPressed : joystickData.rightAButtonPressed;
   }

   private static boolean getBButtonPressed(RobotSide robotSide, int index, List<JoystickData> joystickDataList)
   {
      JoystickData joystickData = joystickDataList.get(index);
      return robotSide == RobotSide.LEFT ? joystickData.leftBButtonPressed : joystickData.rightBButtonPressed;
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

   private static String toCSV(Tuple3DReadOnly tuple, boolean end)
   {
      return tuple.getX() + "," + tuple.getY() + "," + tuple.getZ() + (end ? "\n" : ",");
   }

   private static String toCSV(QuaternionReadOnly quaternion, boolean end)
   {
      return quaternion.getX() + "," + quaternion.getY() + "," + quaternion.getZ() + "," + quaternion.getS() + (end ? "\n" : ",");
   }

   private static int loadCSV(int index, String[] data, FramePose3D poseToPack, ReferenceFrame saveFrame, ReferenceFrame loadFrame)
   {
      poseToPack.setReferenceFrame(saveFrame);
      poseToPack.getOrientation().set(Double.parseDouble(data[index++]), Double.parseDouble(data[index++]), Double.parseDouble(data[index++]), Double.parseDouble(data[index++]));
      poseToPack.getPosition().set(Double.parseDouble(data[index++]), Double.parseDouble(data[index++]), Double.parseDouble(data[index++]));
      poseToPack.changeFrame(loadFrame);
      return index;
   }

   private static int loadCSV(int index, String[] data, FrameVector3D vectorToPack, ReferenceFrame saveFrame, ReferenceFrame loadFrame)
   {
      vectorToPack.setReferenceFrame(saveFrame);
      vectorToPack.set(Double.parseDouble(data[index++]), Double.parseDouble(data[index++]), Double.parseDouble(data[index++]));
      vectorToPack.changeFrame(loadFrame);
      return index;
   }

   private static int loadCSV(int index, String[] data, FramePoint3D pointToPack, ReferenceFrame saveFrame, ReferenceFrame loadFrame)
   {
      pointToPack.setReferenceFrame(saveFrame);
      pointToPack.set(Double.parseDouble(data[index++]), Double.parseDouble(data[index++]), Double.parseDouble(data[index++]));
      pointToPack.changeFrame(loadFrame);
      return index;
   }

   public static void main(String[] args) throws Exception
   {
      FileDialog dialog = new FileDialog((Frame) null, "Choose file", FileDialog.LOAD);
      dialog.setFilenameFilter((dir, name) -> name.endsWith(".csv"));

      String filePath = Paths.get(System.getProperty("user.home"), ".ihmc/logs").toString();
      File logDirectory = new File(System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs");
      dialog.setFile(logDirectory.getAbsolutePath());
      dialog.setVisible(true);

      String directory = dialog.getDirectory();
      String filename = dialog.getFile();
      File file;

      if (filename != null)
      {
         file = new File(directory + filename);
      }
      else
      {
         return;
      }

      List<JoystickData> updatedJoystickData = new ArrayList<>();
      BufferedReader fileReader = new BufferedReader(new FileReader(directory + filename));

      String line;

      while ((line = fileReader.readLine()) != null)
      {
         String[] data = line.split(",");

         long timeInTrajectoryMillis = Long.parseLong(data[18]);
         JoystickData joystickData = new JoystickData(timeInTrajectoryMillis);

         int index = 0;
         joystickData.leftAButtonPressed = Boolean.parseBoolean(data[index++]);
         joystickData.leftTriggerPressed = Boolean.parseBoolean(data[index++]);
         index = loadCSV(index, data, joystickData.controllerPoses.get(RobotSide.LEFT), ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame());
         joystickData.rightAButtonPressed = Boolean.parseBoolean(data[index++]);
         joystickData.rightTriggerPressed = Boolean.parseBoolean(data[index++]);
         index = loadCSV(index, data, joystickData.controllerPoses.get(RobotSide.RIGHT), ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame());

         if (index != 18)
            throw new RuntimeException("Expecting index of 18 for timestamp");
         index++;

         index = loadCSV(index, data, joystickData.desiredCenterOfMass, ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame());
         updatedJoystickData.add(joystickData);
      }
      fileReader.close();

      // go back and compute velocities
      int indexDelta = 5;

      // compute velocities
      for (int i = indexDelta; i < updatedJoystickData.size() - indexDelta - 1; i++)
      {
         JoystickData joystickData0 = updatedJoystickData.get(i - indexDelta);
         JoystickData joystickData1 = updatedJoystickData.get(i + indexDelta);
         JoystickData joystickDataToPack = updatedJoystickData.get(i);

         double dtMillis = (double) joystickData1.timeInTrajectoryMillis - joystickData0.timeInTrajectoryMillis;
         double dtSec = dtMillis / 1000.0;

         for (RobotSide robotSide : RobotSide.values())
         {
            FramePose3D pose0 = joystickData0.controllerPoses.get(robotSide);
            FramePose3D pose1 = joystickData0.controllerPoses.get(robotSide);

            KSTTools.computeAngularVelocity(dtSec, pose0.getOrientation(), pose1.getOrientation(), joystickDataToPack.controllerAngularVelocities.get(robotSide));
            KSTTools.computeLinearVelocity(dtSec, pose0.getPosition(), pose1.getPosition(), joystickDataToPack.controllerLinearVelocities.get(robotSide));
         }
      }

      // export csvs

      String recordFileName = "241113182923-Wall0_updated.csv";

      File csvFile = new File(filePath + File.separator + recordFileName);
      csvFile.createNewFile();

      try (FileWriter writer = new FileWriter(csvFile))
      {
         for (int row = 0; row < updatedJoystickData.size(); row++)
         {
            JoystickData joystickData = updatedJoystickData.get(row);
            writer.write(joystickData.toString());
         }

         writer.flush();
         writer.close();
         ThreadTools.sleep(5000);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }
}