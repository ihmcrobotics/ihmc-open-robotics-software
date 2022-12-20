package us.ihmc.rdx.ui.tools;

import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImString;
import org.lwjgl.openvr.InputDigitalActionData;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;

import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

public class KinematicsRecordReplay
{
   private final TrajectoryRecordReplay<Double> trajectoryRecorder = new TrajectoryRecordReplay<>(Double.class, "", 1);
   private final ImString recordPath = new ImString(Paths.get(System.getProperty("user.home"), ".ihmc/logs").toString());
   private final ImBoolean enablerRecording = new ImBoolean(false);
   private boolean isRecording = false;
   private final ImString replayPath = new ImString(Paths.get(System.getProperty("user.home"), ".ihmc/logs/1.csv").toString());
   private final ImBoolean enablerReplay = new ImBoolean(false);
   private boolean isReplaying = false;
   private final ImBoolean enabledKinematicsStreaming;
   private boolean isUserMoving = false;
   private final List<List<Pose3DReadOnly>> framesToRecordHistory = new ArrayList<>();
   private int partId = 0; // identifier of current frame, used to now what body part among numberParts we are currently handling

   public KinematicsRecordReplay(ImBoolean enabledKinematicsStreaming, int numberParts)
   {
      this.enabledKinematicsStreaming = enabledKinematicsStreaming;
      trajectoryRecorder.setNumberParts(numberParts);
      for (int n = 0; n < numberParts; n++)
         framesToRecordHistory.add(new ArrayList<>());
   }

   public void processRecordReplayInput(InputDigitalActionData triggerButton)
   {
      // check streaming is on, recording is on and trigger button has been pressed once. if button is pressed again recording is stopped
      if (enabledKinematicsStreaming.get() && enablerRecording.get() && triggerButton.bChanged() && !triggerButton.bState())
      {
         isRecording = !isRecording;
         // check if recording file path has been set to a different one from previous recording. In case update file path.
         if (trajectoryRecorder.hasSavedRecording() && !(trajectoryRecorder.getPath().equals(recordPath.get())))
            trajectoryRecorder.setPath(recordPath.get()); //recorder is reset when changing path
      }
      // check replay is on and trigger button has been pressed once. if button is pressed again replay is stopped
      if (enablerReplay.get() && triggerButton.bChanged() && !triggerButton.bState())
      {
         isReplaying = !isReplaying;
         // check if replay file has been set to a different one from previous replay. In case update file path.
         if (trajectoryRecorder.hasDoneReplay() && !(trajectoryRecorder.getPath().equals(replayPath.get())))
            trajectoryRecorder.setPath(replayPath.get()); //replayer is reset when changing path
      }
   }

   public void framePoseToRecord(FramePose3DReadOnly framePose)
   {
      if (isRecording)
      {
         if (isMoving(framePose)) //check from frames if the user is moving
         { // we want to start the recording as soon as the user start moving, recordings with different initial pauses can lead to bad behaviors when used for learning
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
      }
      else if (!(trajectoryRecorder.hasSavedRecording()))
      {
         trajectoryRecorder.concatenateData();
         trajectoryRecorder.saveRecording();
         isUserMoving = false;
      }
   }

   private boolean isMoving(FramePose3DReadOnly framePose)
   {
      if (!isUserMoving)
      {
         Pose3D lastFramePose = new Pose3D();
         lastFramePose.getPosition().set(framePose.getPosition().getX(), framePose.getPosition().getY(), framePose.getPosition().getZ());
         lastFramePose.getOrientation()
                         .set(framePose.getOrientation().getX(),
                              framePose.getOrientation().getY(),
                              framePose.getOrientation().getZ(),
                              framePose.getOrientation().getS());
         framesToRecordHistory.get(partId).add(lastFramePose);
         if (framesToRecordHistory.get(partId).size() > 1)
         {
            double distance = (framesToRecordHistory.get(partId).get(framesToRecordHistory.get(partId).size() - 1)).getTranslation()
                                                                                                                   .distance(framesToRecordHistory.get(partId)
                                                                                                                                                  .get(0)
                                                                                                                                                  .getTranslation());
            isUserMoving = distance > 0.04;
         }
         partId++;
         if (partId >= framesToRecordHistory.size())
            partId = 0;
      }
      return isUserMoving;
   }

   public void framePoseToPack(FixedFramePose3DBasics framePose)
   {
      framePose.setFromReferenceFrame(ReferenceFrame.getWorldFrame());
      // Read file with stored trajectories: read setpoint per timestep until file is over
      Double[] dataPoint = trajectoryRecorder.play(true); //play split data (a body part per time)
      // [0,1,2,3] quaternion of body segment; [4,5,6] position of body segment
      framePose.getOrientation().set(dataPoint[0], dataPoint[1], dataPoint[2], dataPoint[3]);
      framePose.getPosition().set(dataPoint[4], dataPoint[5], dataPoint[6]);
      if (trajectoryRecorder.hasDoneReplay())
      {
         isReplaying = false;
         enablerReplay.set(false);
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

   public ImBoolean isRecordingEnabled()
   {
      return enablerRecording;
   }

   public ImBoolean isReplayingEnabled()
   {
      return enablerReplay;
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
