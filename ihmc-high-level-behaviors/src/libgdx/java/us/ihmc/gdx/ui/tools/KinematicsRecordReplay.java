package us.ihmc.gdx.ui.tools;

import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImString;
import org.lwjgl.openvr.InputDigitalActionData;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;

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
      if (enabledKinematicsStreaming.get() && enablerRecording.get() && triggerButton.bChanged() && !triggerButton.bState())
      {
         isRecording = !isRecording;
         if (trajectoryRecorder.hasSavedRecording() && !(trajectoryRecorder.getPath().equals(recordPath.get())))
            trajectoryRecorder.setPath(recordPath.get());
      }
      if (enablerReplay.get() && triggerButton.bChanged() && !triggerButton.bState())
      {
         isReplaying = !isReplaying;
         if (trajectoryRecorder.hasDoneReplay() && !(trajectoryRecorder.getPath().equals(replayPath.get())))
            trajectoryRecorder.setPath(replayPath.get());
      }
   }

   public void record(FramePose3D framePose)
   {
      if (isRecording)
      {
         // Store trajectories in file: store a setpoint per timestep until trigger button is pressed again
         // [0,1,2] position of body segment; [3,4,5,6] quaternion of body segment
         Double[] dataTrajectories = new Double[] {framePose.getPosition().getX(),
                                                   framePose.getPosition().getY(),
                                                   framePose.getPosition().getZ(),
                                                   framePose.getOrientation().getX(),
                                                   framePose.getOrientation().getY(),
                                                   framePose.getOrientation().getZ(),
                                                   framePose.getOrientation().getS()};
         trajectoryRecorder.record(dataTrajectories);
      }
      else if (!(trajectoryRecorder.hasSavedRecording()))
      {
         trajectoryRecorder.saveRecording();
      }
   }

   public void replay(FramePose3D framePose)
   {
      // Read file with stored trajectories: read setpoint per timestep until file is over
      Double[] dataPoint = trajectoryRecorder.play();
      if (!trajectoryRecorder.hasDoneReplay())
      {
         // [0,1,2] position of body segment; [3,4,5,6] quaternion of body segment
         framePose.getPosition().set(dataPoint[0], dataPoint[1], dataPoint[2]);
         framePose.getOrientation().set(dataPoint[3], dataPoint[4], dataPoint[5], dataPoint[6]);
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
