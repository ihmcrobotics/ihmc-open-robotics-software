package us.ihmc.rdx.ui.tools;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.type.ImBoolean;
import imgui.type.ImInt;
import imgui.type.ImString;
import org.lwjgl.openvr.InputDigitalActionData;
import us.ihmc.behaviors.tools.TrajectoryRecordReplay;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;

import java.nio.file.Paths;
import java.util.*;

/**
 * Class for recording or replaying motions in the UI.
 * Motions are represented as trajectories of frames and are loaded and saved as .csv files
 */
public class KinematicsRecordReplay
{
   private final ImInt numberOfParts = new ImInt(2);
   private final TrajectoryRecordReplay trajectoryRecorder = new TrajectoryRecordReplay("", 1);
   private final ImString recordPath = new ImString(Paths.get(System.getProperty("user.home"), ".ihmc/logs").toString());
   private final ImBoolean enablerRecording = new ImBoolean(false);
   private boolean isRecording = false;
   private final ImString replayPath = new ImString(Paths.get(System.getProperty("user.home"), ".ihmc/logs/1.csv").toString());
   private final ImBoolean enablerReplay = new ImBoolean(false);
   private boolean isReplaying = false;
   private final ImBoolean enabledKinematicsStreaming;
   private boolean isUserMoving = false;
   private final List<List<Pose3DReadOnly>> framesToRecordHistory = new ArrayList<>();
   private int partId = 0; // identifier of current frame, used to now what body part among numberOfParts we are currently handling
   private final SceneGraph sceneGraph;
   private ReferenceFrame sceneNodeFrame;
   private List<String> selectableSceneNodeNames = new ArrayList<>();
   private int selectedNodeIndex = 0;
   private final Map<String, FramePose3D> previousFramePose = new HashMap<>();
   private final Map<String, FramePose3D> firstFramePose = new HashMap<>();

   public KinematicsRecordReplay(SceneGraph sceneGraph, ImBoolean enabledKinematicsStreaming)
   {
      this.sceneGraph = sceneGraph;
      this.enabledKinematicsStreaming = enabledKinematicsStreaming;
      trajectoryRecorder.setNumberOfParts(numberOfParts.get()); // default to 2
      for (int n = 0; n < numberOfParts.get(); n++)
         framesToRecordHistory.add(new ArrayList<>());

      trajectoryRecorder.setDoneReplay(true);
   }

   public void updateNumberOfParts()
   {
      trajectoryRecorder.setNumberOfParts(numberOfParts.get());
      framesToRecordHistory.clear();
      for (int n = 0; n < numberOfParts.get(); n++)
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
            trajectoryRecorder.setPath(replayPath.get()); // replayer is reset when changing path
      }
   }

   /**
    * Record frame
    */
   public void framePoseToRecord(FramePose3DReadOnly framePose, String frameName)
   {
      if (isRecording)
      {
         if (isMoving(framePose)) //check from framePose if the user is moving
         { // we want to start the recording as soon as the user starts moving, recordings with different initial pauses can lead to bad behaviors when used for learning
            framePose.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
            FramePose3D frameToRecord = new FramePose3D(framePose);
            // transform to object reference frame if using object detection
            if (sceneNodeFrame != null)
               frameToRecord.changeFrame(sceneNodeFrame);

            ensureOrientationContinuity(frameToRecord, frameName);
            double[] dataTrajectories = new double[7];
            frameToRecord.getOrientation().get(dataTrajectories);
            frameToRecord.getPosition().get(4, dataTrajectories);
            trajectoryRecorder.record(dataTrajectories);
         }
      }
      else if (!(trajectoryRecorder.hasSavedRecording()))
      {
         trajectoryRecorder.concatenateData();
         trajectoryRecorder.saveRecording();
         if (!firstFramePose.isEmpty() && !previousFramePose.isEmpty())
         {
            for (String frame : previousFramePose.keySet())
               previousFramePose.replace(frame, firstFramePose.get(frame));
         }
         isUserMoving = false;
      }
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
    * Check if user is moving a certain bodyPart. Useful to start a recording only after the user started actually moving
    */
   private boolean isMoving(FramePose3DReadOnly framePose)
   {
      if (!isUserMoving)
      {
         Pose3D lastFramePose = new Pose3D(framePose);
         framesToRecordHistory.get(partId).add(lastFramePose);
         // check if last value of frame pose translated by 4cm with respect to first value of frame pose
         if (framesToRecordHistory.get(partId).size() > 1)
         {
            double distance = (framesToRecordHistory.get(partId).get(framesToRecordHistory.get(partId).size() - 1)).getTranslation()
                                                                                                                   .distance(framesToRecordHistory.get(partId)
                                                                                                                                                  .get(0)
                                                                                                                                                  .getTranslation());
            isUserMoving = distance > 0.04;
         }
         if (!isUserMoving) // if still not moving analyze next frame at next call
         {
            partId++;
            if (partId >= framesToRecordHistory.size())
               partId = 0;
         }
      }
      if (isUserMoving && partId > 0)  // preventing start recording while skipping the frame of other parts, wait to have all parts in the next frame
      {
         partId++;
         if (partId >= framesToRecordHistory.size())
            partId = 0;
         return false;
      }
      return isUserMoving && partId == 0;
   }

   /**
    * Pack frame with frame from replay
    */
   public void framePoseToPack(FramePose3D framePose)
   {
      framePose.setFromReferenceFrame(ReferenceFrame.getWorldFrame());
      // Read file with stored trajectories: read set point per timestep until file is over
      double[] dataPoint = trajectoryRecorder.play(true); //play split data (a body part per time)
      if (sceneNodeFrame != null)
         framePose.changeFrame(sceneNodeFrame);
      // [0,1,2,3] quaternion of body segment; [4,5,6] position of body segment
      framePose.getOrientation().set(dataPoint);
      framePose.getPosition().set(4, dataPoint);
      if (sceneNodeFrame != null)
         framePose.changeFrame(ReferenceFrame.getWorldFrame());
      if (trajectoryRecorder.hasDoneReplay())
      {
         isReplaying = false;
         enablerReplay.set(false);
      }
   }

   public void renderRecordWidgets(ImGuiUniqueLabelMap labels)
   {
      if (ImGuiTools.volatileInputInt(labels.get("Number of Robot Parts to Record/Replay"), numberOfParts))
         updateNumberOfParts();
      if (ImGui.checkbox(labels.get("Record Motion"), enablerRecording))
      {
         setRecording(enablerRecording.get());
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
      if (ImGui.checkbox(labels.get("Replay Motion"), enablerReplay))
      {
         setReplay(enablerReplay.get());
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

   private void setRecording(boolean enablerRecording)
   {
      if (enablerRecording != this.enablerRecording.get())
         this.enablerRecording.set(enablerRecording);
      if (enablerRecording)
      {
         this.enablerReplay.set(false); // check no concurrency replay and record
      }
      else
      {
         firstFramePose.clear();
         previousFramePose.clear();
      }
   }

   public void setReplay(boolean enablerReplay)
   {
      if (enablerReplay != this.enablerReplay.get())
         this.enablerReplay.set(enablerReplay);
      if (enablerReplay)
      {
         if (enablerRecording.get() || enabledKinematicsStreaming.get())
            this.enablerReplay.set(false); // check no concurrency replay and record/streaming
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