package us.ihmc.rdx.ui.tools;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.type.ImBoolean;
import imgui.type.ImInt;
import imgui.type.ImString;
import us.ihmc.behaviors.tools.TrajectoryRecordReplay;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.nio.file.Paths;
import java.util.*;
import java.util.function.Consumer;

/**
 * Class for recording or replaying motions in the UI.
 * Motions are represented as trajectories of frames and are loaded and saved as .csv files
 */
public class KinematicsRecordReplay
{
   private final ImInt numberOfParts = new ImInt(2);
   private final TrajectoryRecordReplay trajectoryRecorder = new TrajectoryRecordReplay("", 1);
   private final ImString recordPath = new ImString(Paths.get(System.getProperty("user.home"), ".ihmc/logs").toString(), 100);
   private final ImBoolean enableRecording = new ImBoolean(false);
   private boolean isRecording = false;

//   private final String defaultReplayFile = "241009_wallBracing0_goodRun.csv";
//      private final String defaultReplayFile = "241009_wallBracing_pitchedGoodRun.csv";
//   private final String defaultReplayFile = "241009_wallBracing_uneven.csv";

   private final String defaultReplayFile = "241104110158-0600.csv";

   private final ImString replayPath = new ImString(Paths.get(System.getProperty("user.home"), ".ihmc/logs/" + defaultReplayFile).toString(), 100);
   private final ImBoolean enableReplay = new ImBoolean(false);
   private boolean isReplaying = false;
   private final ImBoolean enabledKinematicsStreaming;
   private boolean isUserMoving = false;
   private int partId = 0; // identifier of current frame, used to now what body part among numberOfParts we are currently handling
   private final SceneGraph sceneGraph;
   private ReferenceFrame sceneNodeFrame;
   private List<String> selectableSceneNodeNames = new ArrayList<>();
   private int selectedNodeIndex = 0;
   private final Map<String, FramePose3D> previousFramePose = new HashMap<>();
   private final Map<String, FramePose3D> firstFramePose = new HashMap<>();
   private final SideDependentList<MutableReferenceFrame> handDesiredControlFrames;
   private boolean requestRecordReplay;
   private Consumer<Boolean> replayCallback = b -> {};

   public KinematicsRecordReplay(SceneGraph sceneGraph, ImBoolean enabledKinematicsStreaming, SideDependentList<MutableReferenceFrame> handDesiredControlFrames)
   {
      this.sceneGraph = sceneGraph;
      this.enabledKinematicsStreaming = enabledKinematicsStreaming;
      this.handDesiredControlFrames = handDesiredControlFrames;
      trajectoryRecorder.setDoneReplay(true);
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
   public void onUpdateEnd(ReferenceFrame loadInFrame)
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
         isReplaying = trajectoryRecorder.onReplayStart(replayPath.get(), loadInFrame);
         replayCallback.accept(true);
      }
   }

   public void recordControllerData(RobotSide robotSide, boolean aButtonPressed, boolean triggerPressed, ReferenceFrame recordFrame)
   {
      if (!isRecording)
         return;

      trajectoryRecorder.recordControllerData(robotSide,
                                              aButtonPressed,
                                              triggerPressed,
                                              handDesiredControlFrames.get(robotSide).getReferenceFrame(),
                                              recordFrame);
   }

   public void recordDesiredCenterOfMass(FramePoint3DReadOnly desiredCenterOfMass, ReferenceFrame recordFrame)
   {
      trajectoryRecorder.recordDesiredCenterOfMass(desiredCenterOfMass, recordFrame);
   }

   /**
    * Ensure that every recorded trajectory has the same quaternion sign (-pi/2,pi/2 range) to the previous recording
    * introduce a rule that allows us to likely get the same quaternion sign if we ever want to expand the dataset
    */
   private void ensureOrientationContinuity(FramePose3D frameToCheck, String frameName)
   {
      QuaternionReadOnly quaternionToCheck = frameToCheck.getOrientation();
      if (previousFramePose.containsKey(frameName))
      {
         if (sceneNodeFrame != null)
            previousFramePose.get(frameName).changeFrame(sceneNodeFrame);

         QuaternionReadOnly previousQuaternion = previousFramePose.get(frameName).getOrientation();
         // Check that quaternion is not changing 2pi range. Even if q = -q, the observed motion has to be continuous
         frameToCheck.getOrientation().interpolate(previousQuaternion, quaternionToCheck, 1.0);

         previousFramePose.get(frameName).set(frameToCheck);

         if (Math.abs(frameToCheck.getOrientation().getX() - previousQuaternion.getX()) > 0.05 ||
             Math.abs(frameToCheck.getOrientation().getY() - previousQuaternion.getY()) > 0.05 ||
             Math.abs(frameToCheck.getOrientation().getZ() - previousQuaternion.getZ()) > 0.05 ||
             Math.abs(frameToCheck.getOrientation().getS() - previousQuaternion.getS()) > 0.05)
         {
            LogTools.error("Quaternion discontinuity asymmetric wrt zero. Check recorded part was not disconnected nor occluded during recording.");
            frameToCheck.getOrientation().set(previousQuaternion);
         }
      }
      else
      {
         double x = quaternionToCheck.getX();
         double y = quaternionToCheck.getY();
         double z = quaternionToCheck.getZ();
         double s = quaternionToCheck.getS();

         // Calculate the maximum absolute value
         double max = Math.max(Math.abs(x), Math.max(Math.abs(y), Math.max(Math.abs(z), Math.abs(s))));

         // Check if the maximum absolute value is negative
         if ((Math.abs(x) == max && x < 0) ||
             (Math.abs(y) == max && y < 0) ||
             (Math.abs(z) == max && z < 0) ||
             (Math.abs(s) == max && s < 0))
         {
            frameToCheck.getOrientation().negate();
         }

         previousFramePose.put(frameName, new FramePose3D(frameToCheck));
         firstFramePose.put(frameName, new FramePose3D(frameToCheck));
      }
   }

   /**
    * Pack frame with frame from replay
    */
   public void framePoseToPack(RobotSide robotSide, FramePose3D framePose)
   {
      if (isReplaying)
      {
         if (robotSide == null)
            return;
         trajectoryRecorder.packDesiredHandControlFrame(robotSide, framePose);
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

   public void renderReferenceFrameSelection(ImGuiUniqueLabelMap labels)
   {
      if (sceneGraph.getNodeNameList().size() > 0)
      {
         if (sceneGraph.getNodeNameList().size() != selectableSceneNodeNames.size())
            selectableSceneNodeNames = sceneGraph.getNodeNameList();
         if(sceneNodeFrame == null)
            selectedNodeIndex = 0;

         ImGui.text("Select Record/Replay Reference Frame");
         if (ImGui.beginCombo(labels.get("Reference Frame"), selectedNodeIndex != 0 ? selectableSceneNodeNames.get(selectedNodeIndex) : "World"))
         {
            for (int i = 0; i < selectableSceneNodeNames.size(); i++)
            {
               // use world frame instead of root node
               String selectableSceneNodeName;
               if (i != 0)
                  selectableSceneNodeName = selectableSceneNodeNames.get(i);
               else
                  selectableSceneNodeName = "World";

               if (ImGui.selectable(selectableSceneNodeName, selectedNodeIndex == i))
               {
                  selectedNodeIndex = i;

                  SceneNode selectedNode = sceneGraph.getNamesToNodesMap().get(selectableSceneNodeNames.get(selectedNodeIndex));
                  if (selectedNode.getID() != 0)
                     sceneNodeFrame = selectedNode.getNodeFrame();
                  else
                     sceneNodeFrame = null;
               }
            }
            ImGui.endCombo();
         }
      }
   }

   private void setRecording(boolean enableRecording)
   {
      LogTools.info("Enable recording: " + enableRecording);

      if (enableRecording != this.enableRecording.get())
         this.enableRecording.set(enableRecording);
      if (enableRecording)
      {
         this.enableReplay.set(false); // check no concurrency replay and record
      }
      else
      {
         firstFramePose.clear();
         previousFramePose.clear();
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

   public boolean getTriggerPressed(RobotSide robotSide)
   {
      return trajectoryRecorder.getTriggerPressed(robotSide);
   }

   public FramePoint3D getDesiredCenterOfMass()
   {
      return trajectoryRecorder.getDesiredCenterOfMass();
   }

   public String getReplayPath()
   {
      return replayPath.get();
   }
}