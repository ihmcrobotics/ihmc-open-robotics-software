package us.ihmc.rdx.ui.tools;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.type.ImBoolean;
import imgui.type.ImString;
import us.ihmc.behaviors.tools.TrajectoryRecordReplay;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.log.LogTools;
import us.ihmc.motionRetargeting.VRTrackedSegmentType;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.io.File;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.function.Consumer;

/**
 * Class for recording or replaying motions in the UI.
 * Motions are represented as trajectories of frames and are loaded and saved as .csv files
 */
public class KinematicsRecordReplay
{
   private final TrajectoryRecordReplay trajectoryRecorder = new TrajectoryRecordReplay("");
   private final ImString recordPath = new ImString(Paths.get(System.getProperty("user.home"), ".ihmc/logs").toString(), 100);
   private final ImBoolean enableRecording = new ImBoolean(false);
   private boolean isRecording = false;
   private final ImString replayPath = new ImString(Paths.get(System.getProperty("user.home"), ".ihmc/logs/1.csv").toString(), 100);
   private final ImBoolean enableReplay = new ImBoolean(false);
   private boolean isReplaying = false;
   private final ImBoolean enabledKinematicsStreaming;

   private final SideDependentList<MutableReferenceFrame> handDesiredControlFrames;
   private boolean requestRecordReplay;
   private Consumer<Boolean> replayCallback = b -> {};

   public KinematicsRecordReplay(ImBoolean enabledKinematicsStreaming, SideDependentList<MutableReferenceFrame> handDesiredControlFrames)
   {
      this.enabledKinematicsStreaming = enabledKinematicsStreaming;
      this.handDesiredControlFrames = handDesiredControlFrames;

      loadLatestReplayFile();
   }

   public void loadLatestReplayFile()
   {
      File logDirectory = new File(System.getProperty("user.home"), ".ihmc/logs");
      if (!logDirectory.exists())
         return;

      File[] files = logDirectory.listFiles(f -> f.isFile() && f.getName().endsWith(".csv"));
      if (files == null || files.length == 0)
         return;

      Arrays.sort(files, (f1, f2) -> Long.valueOf(f2.lastModified()).compareTo(f1.lastModified()));
      replayPath.set(files[0].getAbsoluteFile());
   }

   public void requestRecordReplay()
   {
      /* Processed in onUpdateEnd, depending on if enableRecord or enableReplay are selected */
      LogTools.info("record/replay requested");
      requestRecordReplay = true;
   }

   public void setReplayCallback(Consumer<Boolean> replayCallback)
   {
      this.replayCallback = replayCallback;
   }

   public void onUpdateStart()
   {
      requestRecordReplay = false;

      if (isReplaying)
      {
         isReplaying = trajectoryRecorder.onUpdateStartReplay();
         if (!isReplaying)
         { // Finished replaying
            LogTools.info("Finished replayed!");
            isReplaying = false;
            replayCallback.accept(false);
         }
      }
      else if (isRecording)
      {
         trajectoryRecorder.onUpdateStartRecord();
      }
   }

   /**
    * Called each tick regardless of record/replay status.
    */
   public void onUpdateEnd(ReferenceFrame recordFrame)
   {
      if (requestRecordReplay && enableRecording.get())
      { // Toggle record state
         if (isRecording)
         {
            LogTools.info("Finished recording!");
            isRecording = false;
            trajectoryRecorder.setPath(recordPath.get());
            trajectoryRecorder.onRecordEnd();
         }
         else
         {
            LogTools.info("Starting to record!");
            isRecording = true;
            trajectoryRecorder.onRecordStart();
         }
      }

      else if (requestRecordReplay && enableReplay.get() && !isReplaying)
      { // Start to replay
         LogTools.info("Starting to replay!");
         isReplaying = trajectoryRecorder.onReplayStart(replayPath.get(), recordFrame);
         replayCallback.accept(true);
      }
   }

   public void recordControllerData(RobotSide robotSide, boolean aButtonPressed, boolean bButtonPressed, boolean triggerPressed, Vector3D angularVelocity, Vector3D linearVelocity, ReferenceFrame recordFrame)
   {
      if (!isRecording)
         return;

      trajectoryRecorder.recordControllerData(robotSide,
                                              aButtonPressed,
                                              bButtonPressed,
                                              triggerPressed,
                                              handDesiredControlFrames.get(robotSide).getReferenceFrame(),
                                              angularVelocity,
                                              linearVelocity,
                                              recordFrame);
   }

   /**
    * Pack frame with frame from replay
    */
   public void packLoggedData(VRTrackedSegmentType segmentType, FramePose3D framePose, FrameVector3D angularVelocity, FrameVector3D linearVelocity)
   {
      if (isReplaying)
      {
         trajectoryRecorder.packTrackedData(segmentType, framePose, angularVelocity, linearVelocity);
      }
   }

   public void renderRecordWidgets(ImGuiUniqueLabelMap labels)
   {
      if (ImGui.checkbox(labels.get("Record Motion"), enableRecording))
      {
         //         setRecording(enablerRecording.get());
      }
      ImGui.sameLine();
      ImGui.inputText(labels.get("Record Folder"), recordPath);
      if (isRecording)
      {
         ImGui.pushStyleColor(ImGuiCol.Text, ImGuiTools.RED);
         ImGui.text("Recording");
         ImGui.popStyleColor();
      }
   }

   public void renderReplayWidgets(ImGuiUniqueLabelMap labels)
   {
      if (ImGui.checkbox(labels.get("Replay Motion"), enableReplay))
      {
         setReplay(enableReplay.get());
      }
      ImGui.sameLine();
      ImGui.inputText(labels.get("Replay File"), replayPath);
      if (isReplaying)
      {
         ImGui.pushStyleColor(ImGuiCol.Text, ImGuiTools.RED);
         ImGui.text("Replaying");
         ImGui.popStyleColor();
      }
   }

   private void setRecording(boolean enableRecording)
   {
      if (enableRecording != this.enableRecording.get())
         this.enableRecording.set(enableRecording);
      if (enableRecording)
      {
         this.enableReplay.set(false); // check no concurrency replay and record
      }
   }

   public void setReplay(boolean enablerReplay)
   {
      if (enablerReplay != this.enableReplay.get())
         this.enableReplay.set(enablerReplay);
      if (enablerReplay)
      {
         if (enableRecording.get() || enabledKinematicsStreaming.get())
            this.enableReplay.set(false); // check no concurrency replay and record/streaming
      }
   }

   public ImBoolean isRecordingEnabled()
   {
      return enableRecording;
   }

   public ImBoolean isReplayingEnabled()
   {
      return enableReplay;
   }

   public boolean isRecording()
   {
      return isRecording;
   }

   public boolean isReplaying()
   {
      return isReplaying;
   }

   public boolean getAButtonPressed(RobotSide robotSide)
   {
      return trajectoryRecorder.getAButtonPressed(robotSide);
   }

   public boolean getBButtonPressed(RobotSide robotSide)
   {
      return trajectoryRecorder.getBButtonPressed(robotSide);
   }

   public boolean getTriggerPressed(RobotSide robotSide)
   {
      return trajectoryRecorder.getTriggerPressed(robotSide);
   }

   public String getReplayPath()
   {
      return replayPath.get();
   }
}