package us.ihmc.behaviors.tools;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.motionRetargeting.VRTrackedSegmentType;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.io.*;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;
import java.util.function.IntFunction;
import java.util.function.IntPredicate;

/**
 * Class to record and replay raw VR joystick input.
 */
public class TrajectoryRecordReplay
{
   private String filePath;
   private String recordFileName = "";

   private final List<VRInputData> vrInputData = new ArrayList<>();
   private int replayIndex = 0;
   private double replayInterpolationAlpha = -1.0;

   private long replayStartTimeMillis;
   private int lastReplayIndex = -1;

   public static final int numberOfTrackers = VRTrackedSegmentType.TRACKER_TYPES.length;
   public static final EnumMap<VRTrackedSegmentType, Integer> trackerIndex = new EnumMap<>(VRTrackedSegmentType.class);

   static
   {
      for (int i = 0; i < numberOfTrackers; i++)
      {
         trackerIndex.put(VRTrackedSegmentType.TRACKER_TYPES[i], i);
      }
   }

   public TrajectoryRecordReplay(String filePath)
   {
      this.filePath = filePath;
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
         boolean isStillReplaying = replayIndex < vrInputData.size() - 1;
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
      while (searchIndex < vrInputData.size() && vrInputData.get(searchIndex).timeInTrajectoryMillis < timeInTrajectory)
      {
         searchIndex++;
      }

      // if first or last index, return it
      if (searchIndex <= 0 || searchIndex >= vrInputData.size() - 1)
      {
         replayInterpolationAlpha = -1.0;
         return searchIndex;
      }

      // compute the interpolation between the two adjacent frames to smoothly replay
      long t0 = vrInputData.get(searchIndex - 1).timeInTrajectoryMillis;
      long t1 = vrInputData.get(searchIndex).timeInTrajectoryMillis;
      replayInterpolationAlpha = ((double) (timeInTrajectory - t0)) / (t1 - t0);

      return searchIndex;
   }

   public String getPath()
   {
      return this.filePath;
   }

   public void setPath(String filePath)
   {
      this.filePath = filePath;
   }

   public void onUpdateStartRecord()
   {
      if (vrInputData.isEmpty())
      { // First record tick, initialize
         replayStartTimeMillis = System.currentTimeMillis();
      }

      long timeInTrajectoryMillis = System.currentTimeMillis() - replayStartTimeMillis;
      vrInputData.add(new VRInputData(timeInTrajectoryMillis));
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
      vrInputData.get(vrInputData.size() - 1).controllerData.get(robotSide)
                                                            .set(aButtonPressed,
                                                                 bButtonPressed,
                                                                 triggerPressed,
                                                                 desiredControlFrame,
                                                                 angularVelocity,
                                                                 linearVelocity,
                                                                 recordFrame);
   }

   public void recordTrackerData(VRTrackedSegmentType segmentType,
                                 ReferenceFrame desiredControlFrame,
                                 Vector3D angularVelocity,
                                 Vector3D linearVelocity,
                                 ReferenceFrame recordFrame)
   {
      vrInputData.get(vrInputData.size() - 1).trackerData.get(trackerIndex.get(segmentType)).set(desiredControlFrame, angularVelocity, linearVelocity, recordFrame);
   }

   public void onRecordStart()
   {
      vrInputData.clear();
   }

   public void onRecordEnd()
   {
      if (vrInputData.isEmpty())
         return;

      // save recording
      // if recordFile name has not been set, generate file with current date and time as name
      String fileName = "";
      if (recordFileName.isEmpty())
      {
         fileName = new SimpleDateFormat("yyMMddHHmmss'.csv'").format(new Date());
         recordFileName = fileName;
      }
      else
         fileName = recordFileName;
      File csvFile = new File(filePath + "/" + fileName);
      try (FileWriter writer = new FileWriter(csvFile))
      {
         // First line is just the tracked segments
         for (int i = 0; i < numberOfTrackers; i++)
         {
            writer.write(VRTrackedSegmentType.TRACKER_TYPES[i] + (i < numberOfTrackers - 1 ? "," : "\n"));
         }

         for (int row = 0; row < this.vrInputData.size(); row++)
         {
            VRInputData inputData = this.vrInputData.get(row);
            writer.write(inputData.toString());
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
      vrInputData.clear();
      File replayFile = new File(replayFileToLoad);
      replayIndex = -1;
      replayStartTimeMillis = -1;

      try
      {
         BufferedReader fileReader = new BufferedReader(new FileReader(replayFile));
         String line;

         // Check first line matches tracked segments
         line = fileReader.readLine();

         boolean matchesLogFile = true;
         String[] segments = line.split(",");
         if (segments.length != numberOfTrackers)
         {
            matchesLogFile = false;
         }
         for (int i = 0; i < numberOfTrackers; i++)
         {
            if (!VRTrackedSegmentType.TRACKER_TYPES[i].name().equals(segments[i]))
               matchesLogFile = false;
         }

         if (!matchesLogFile)
         {
            LogTools.info("Log file has the following trackers, which doesn't match configuration: " + line);
            return false;
         }

         while ((line = fileReader.readLine()) != null)
         {
            vrInputData.add(new VRInputData(line.split(","), recordFrame));
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

   private static class VRInputData
   {
      private final long timeInTrajectoryMillis;
      private final SideDependentList<ControllerData> controllerData = new SideDependentList<>(new ControllerData(), new ControllerData());
      private final List<TrackerData> trackerData = new ArrayList<>();

      public VRInputData(long timeInTrajectoryMillis)
      {
         this.timeInTrajectoryMillis = timeInTrajectoryMillis;

         for (int i = 0; i < numberOfTrackers; i++)
         {
            trackerData.add(new TrackerData());
         }
      }

      public VRInputData(String[] data, ReferenceFrame recordFrame)
      {
         int index = 0;
         timeInTrajectoryMillis = Long.parseLong(data[index++]);

         for (RobotSide robotSide : RobotSide.values())
         {
            index = controllerData.get(robotSide).setFromCSV(index, data, recordFrame);
         }

         for (int i = 0; i < numberOfTrackers; i++)
         {
            trackerData.add(new TrackerData());
            index = trackerData.get(i).setFromCSV(index, data, recordFrame);
         }
      }

      @Override
      public String toString()
      {
         StringBuilder builder = new StringBuilder();
         builder.append(timeInTrajectoryMillis);

         for (RobotSide robotSide : RobotSide.values())
         {
            builder.append(",");
            builder.append(controllerData.get(robotSide));
         }
         for (int i = 0; i < numberOfTrackers; i++)
         {
            builder.append(",");
            builder.append(trackerData.get(i));
         }

         builder.append("\n");
         return builder.toString();
      }
   }

   private static class TrackerData
   {
      final FramePose3D pose = new FramePose3D();
      final FrameVector3D angularVelocity = new FrameVector3D();
      final FrameVector3D linearVelocity = new FrameVector3D();

      public int setFromCSV(int index, String[] data, ReferenceFrame recordFrame)
      {
         index = loadCSV(index, data, pose, recordFrame, ReferenceFrame.getWorldFrame());
         index = loadCSV(index, data, angularVelocity, recordFrame, ReferenceFrame.getWorldFrame());
         index = loadCSV(index, data, linearVelocity, recordFrame, ReferenceFrame.getWorldFrame());
         return index;
      }

      void set(ReferenceFrame trackerFrame, Vector3D angularVelocity, Vector3D linearVelocity, ReferenceFrame recordFrame)
      {
         this.pose.setToZero(trackerFrame);
         this.pose.changeFrame(recordFrame);
         this.angularVelocity.set(ReferenceFrame.getWorldFrame(), angularVelocity);
         this.angularVelocity.changeFrame(recordFrame);
         this.linearVelocity.set(ReferenceFrame.getWorldFrame(), linearVelocity);
         this.linearVelocity.changeFrame(recordFrame);
      }

      @Override
      public String toString()
      {
         return poseToCSV(pose) + "," + tupleToCSV(angularVelocity) + "," + tupleToCSV(linearVelocity);
      }
   }

   private static class ControllerData extends TrackerData
   {
      boolean aButtonPressed;
      boolean bButtonPressed;
      boolean triggerPressed;

      public int setFromCSV(int index, String[] data, ReferenceFrame recordFrame)
      {
         index = super.setFromCSV(index, data, recordFrame);

         aButtonPressed = Boolean.parseBoolean(data[index++]);
         bButtonPressed = Boolean.parseBoolean(data[index++]);
         triggerPressed = Boolean.parseBoolean(data[index++]);

         return index;
      }

      void set(boolean aButtonPressed, boolean bButtonPressed, boolean triggerPressed, ReferenceFrame trackerFrame, Vector3D angularVelocity, Vector3D linearVelocity, ReferenceFrame recordFrame)
      {
         super.set(trackerFrame, angularVelocity, linearVelocity, recordFrame);

         this.aButtonPressed = aButtonPressed;
         this.bButtonPressed = bButtonPressed;
         this.triggerPressed = triggerPressed;
      }

      @Override
      public String toString()
      {
         return super.toString() + "," + aButtonPressed + "," + bButtonPressed + "," + triggerPressed;
      }
   }

   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   ///////////////////////////////////////////////////   Getters for button presses   /////////////////////////////////////////////////////
   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

   public boolean getAButtonPressed(RobotSide robotSide)
   {
      return checkButtonPressOverInterval(lastReplayIndex, replayIndex, i -> getAButtonPressed(robotSide, i, vrInputData));
   }

   public boolean getBButtonPressed(RobotSide robotSide)
   {
      return checkButtonPressOverInterval(lastReplayIndex, replayIndex, i -> getBButtonPressed(robotSide, i, vrInputData));
   }

   public boolean getTriggerPressed(RobotSide robotSide)
   {
      return checkButtonPressOverInterval(lastReplayIndex, replayIndex, i -> getTriggerPressed(robotSide, i, vrInputData));
   }

   public void packTrackedData(VRTrackedSegmentType segmentType, FramePose3D poseToPack, FrameVector3D angularVelocityToPack, FrameVector3D linearVelocityToPack)
   {
      IntFunction<TrackerData> trackerDataSupplier = index -> segmentType.getSegmentSide() == null ? vrInputData.get(index).trackerData.get(trackerIndex.get(segmentType)) : vrInputData.get(index).controllerData.get(segmentType.getSegmentSide());
      packTrackerDataInternal(trackerDataSupplier, poseToPack, angularVelocityToPack, linearVelocityToPack);
   }

   private void packTrackerDataInternal(IntFunction<TrackerData> trackerSupplier, FramePose3D poseToPack, FrameVector3D angularVelocityToPack, FrameVector3D linearVelocityToPack)
   {
      if (replayInterpolationAlpha == -1.0)
      {
         TrackerData trackerData = trackerSupplier.apply(replayIndex);
         poseToPack.set(trackerData.pose);
         angularVelocityToPack.set(trackerData.angularVelocity);
         linearVelocityToPack.set(trackerData.linearVelocity);
      }
      else
      {
         TrackerData trackerData0 = trackerSupplier.apply(replayIndex - 1);
         TrackerData trackerData1 = trackerSupplier.apply(replayIndex);

         poseToPack.interpolate(trackerData0.pose, trackerData1.pose, replayInterpolationAlpha);
         angularVelocityToPack.interpolate(trackerData0.angularVelocity, trackerData1.angularVelocity, replayInterpolationAlpha);
         linearVelocityToPack.interpolate(trackerData0.linearVelocity, trackerData1.linearVelocity, replayInterpolationAlpha);
      }
   }

   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   /////////////////////////////////////////////////////////   Helper methods   ///////////////////////////////////////////////////////////
   ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

   private static boolean getAButtonPressed(RobotSide robotSide, int index, List<VRInputData> vrInputData)
   {
      ControllerData controllerData = vrInputData.get(index).controllerData.get(robotSide);
      return controllerData.aButtonPressed;
   }

   private static boolean getBButtonPressed(RobotSide robotSide, int index, List<VRInputData> vrInputData)
   {
      ControllerData controllerData = vrInputData.get(index).controllerData.get(robotSide);
      return controllerData.bButtonPressed;
   }

   private static boolean getTriggerPressed(RobotSide robotSide, int index, List<VRInputData> vrInputData)
   {
      ControllerData controllerData = vrInputData.get(index).controllerData.get(robotSide);
      return controllerData.triggerPressed;
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

   private static String tupleToCSV(Tuple3DReadOnly tuple)
   {
      return tuple.getX() + "," + tuple.getY() + "," + tuple.getZ();
   }

   private static String poseToCSV(FramePose3DReadOnly pose)
   {
      return pose.getOrientation().getX() + "," + pose.getOrientation().getY() + "," + pose.getOrientation().getZ() + "," + pose.getOrientation().getS() + "," + pose.getPosition().getX() + "," + pose.getPosition().getY() + "," + pose.getPosition().getZ();
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
}