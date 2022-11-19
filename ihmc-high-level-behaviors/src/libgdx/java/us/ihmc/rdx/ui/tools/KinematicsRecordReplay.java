package us.ihmc.rdx.ui.tools;

import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImString;
import org.lwjgl.openvr.InputDigitalActionData;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;

import java.nio.file.Paths;

public class KinematicsRecordReplay
{
   private final TrajectoryRecordReplay<Double> trajectoryRecorder = new TrajectoryRecordReplay<>(Double.class, "");
   private final ImString recordPath = new ImString(Paths.get(System.getProperty("user.home"), ".ihmc/logs/").toString());
   private final ImBoolean enablerRecording = new ImBoolean(false);
   private boolean isRecording = false;
   private final ImString replayPath = new ImString(Paths.get(System.getProperty("user.home"), ".ihmc/logs/myFile.csv").toString());
   private final ImBoolean enablerReplay = new ImBoolean(false);
   private boolean isReplaying = false;
   private ImBoolean enabledKinematicsStreaming;

   public KinematicsRecordReplay(ImBoolean enabledKinematicsStreaming)
   {
      this.enabledKinematicsStreaming = enabledKinematicsStreaming;
   }

   public void processRecordReplayInput(InputDigitalActionData triggerButton)
   {
      // check streaming is on, recording is on and trigger button has been pressed once. if button is pressed again recording is stopped
      if (enabledKinematicsStreaming.get() && enablerRecording.get() && triggerButton.bChanged() && !triggerButton.bState())
      {
         isRecording = !isRecording;
         // check if recording file path has been set to a different one from previous recording. In case update file path.
         if (trajectoryRecorder.hasSavedRecording() && !(trajectoryRecorder.getPath().equals(recordPath.get())))
            trajectoryRecorder.setPath(recordPath.get());
      }
      // check replay is on and trigger button has been pressed once. if button is pressed again replay is stopped
      if (enablerReplay.get() && triggerButton.bChanged() && !triggerButton.bState())
      {
         isReplaying = !isReplaying;
         // check if replay file has been set to a different one from previous replay. In case update file path.
         if (trajectoryRecorder.hasDoneReplay() && !(trajectoryRecorder.getPath().equals(replayPath.get())))
            trajectoryRecorder.setPath(replayPath.get());
      }
   }

   public void framePoseToRecord(FramePose3DReadOnly framePose)
   {
      if (isRecording)
      {
         framePose.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
         // Store trajectories in file: store a setpoint per timestep until trigger button is pressed again
         // [0,1,2,3] quaternion of body segment; [4,5,6] position of body segment
         Double[] dataTrajectories = new Double[] {framePose.getOrientation().getX(),
                                                   framePose.getOrientation().getY(),
                                                   framePose.getOrientation().getZ(),
                                                   framePose.getOrientation().getS(),
                                                   framePose.getPosition().getX(),
                                                   framePose.getPosition().getY(),
                                                   framePose.getPosition().getZ()};
         trajectoryRecorder.record(dataTrajectories);
      }
      else if (!(trajectoryRecorder.hasSavedRecording()))
      {
         trajectoryRecorder.saveRecording();
      }
   }

   public void framePoseToPack(FixedFramePose3DBasics framePose)
   {
      framePose.setFromReferenceFrame(ReferenceFrame.getWorldFrame());
      // Read file with stored trajectories: read setpoint per timestep until file is over
      Double[] dataPoint = trajectoryRecorder.play();
      if (!trajectoryRecorder.hasDoneReplay())
      {
         // [0,1,2,3] quaternion of body segment; [4,5,6] position of body segment
         framePose.getOrientation().set(dataPoint[0], dataPoint[1], dataPoint[2], dataPoint[3]);
         framePose.getPosition().set(dataPoint[4], dataPoint[5], dataPoint[6]);
      }
      else
      {
         isReplaying = false;
         enablerReplay.set(isReplaying);
      }
   }

   public void renderRecordWidgets(ImGuiUniqueLabelMap labels)
   {
      if (ImGui.checkbox(labels.get("Record motion"), enablerRecording))
      {
         setRecording(enablerRecording.get());
      }
      ImGui.sameLine();
      ImGui.inputText(labels.get("Record folder"), recordPath);
   }

   public void renderReplayWidgets(ImGuiUniqueLabelMap labels)
   {
      if (ImGui.checkbox(labels.get("Replay motion"), enablerReplay))
      {
         setReplay(enablerReplay.get());
      }
      ImGui.sameLine();
      ImGui.inputText(labels.get("Replay file"), replayPath);
   }

   private void setRecording(boolean enablerRecording)
   {
      if (enablerRecording != this.enablerRecording.get())
         this.enablerRecording.set(enablerRecording);
      if (enablerRecording)
         this.enablerReplay.set(false); //check no concurrency replay and record
   }

   public void setReplay(boolean enablerReplay)
   {
      if (enablerReplay != this.enablerReplay.get())
         this.enablerReplay.set(enablerReplay);
      if (enablerReplay)
      {
         if (enablerRecording.get() || enabledKinematicsStreaming.get())
            this.enablerReplay.set(false); //check no concurrency replay and record/streaming
      }
   }

   public boolean isRecording()
   {
      return isRecording;
   }

   public boolean isReplaying()
   {
      return isReplaying;
   }
}
