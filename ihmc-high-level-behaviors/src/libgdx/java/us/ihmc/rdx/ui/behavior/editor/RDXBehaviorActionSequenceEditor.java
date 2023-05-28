package us.ihmc.rdx.ui.behavior.editor;

import behavior_msgs.msg.dds.*;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.ImGui;
import imgui.type.ImBoolean;
import org.apache.commons.lang3.tuple.MutablePair;
import std_msgs.msg.dds.Bool;
import std_msgs.msg.dds.Empty;
import std_msgs.msg.dds.Int32;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.BehaviorActionSequence;
import us.ihmc.behaviors.sequence.actions.HandPoseActionData;
import us.ihmc.commons.FormattingTools;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.ui.behavior.editor.actions.*;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.io.*;

import java.util.LinkedList;
import java.util.UUID;

/**
 * This class is primarily an interactable sequence/list of robot actions.
 * Rendering it all gets kind of intense, it has some functionality to collapse everything
 * into a more simplified view.
 *
 * For example, this class is a panel that would render several hand poses and a walk goal
 * all in a sequence and allow you to run them on the real robot, but also preview and
 * tune them.
 *
 * TODO: Improve usability. Some things that need improving:
 *   - Actions get inserted in a weird place currently, I know I changed this a couple times,
 *     it may need to be dynamic.
 *   - Maybe including separators between the actions would help.
 *   - Icons like a little hand for the hand poses, and feet for walk goal would likely help.
 *     A little clock for the wait actions. etc.
 */
public class RDXBehaviorActionSequenceEditor
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private ImGuiPanel panel;
   private final ImBoolean automaticExecution = new ImBoolean(false);
   private String name;
   private final WorkspaceResourceFile workspaceFile;
   private final LinkedList<RDXBehaviorAction> actionSequence = new LinkedList<>();
   private String pascalCasedName;
   private RDX3DPanel panel3D;
   private DRCRobotModel robotModel;
   private ROS2SyncedRobotModel syncedRobot;
   private ReferenceFrameLibrary referenceFrameLibrary;
   private ROS2ControllerHelper ros2;
   private final MutablePair<Integer, Integer> reorderRequest = MutablePair.of(-1, 0);
   private boolean loading = false;
   private volatile long receivedSequenceStatusMessageCount = 0;
   private long receivedStatusMessageCount = 0;
   private int executionNextIndexStatus;
   private final Int32 currentActionIndexCommandMessage = new Int32();
   private IHMCROS2Input<Int32> executionNextIndexStatusSubscription;
   private IHMCROS2Input<Bool> automaticExecutionStatusSubscription;
   private IHMCROS2Input<ActionSequenceUpdateMessage> sequenceStatusSubscription;
   private final Empty manuallyExecuteNextActionMessage = new Empty();
   private final Bool automaticExecutionCommandMessage = new Bool();
   private final Bool playbackAutomaticExecutionCommandMessage = new Bool();

   public RDXBehaviorActionSequenceEditor(WorkspaceResourceFile fileToLoadFrom)
   {
      this.workspaceFile = fileToLoadFrom;
      loadNameFromFile();
      afterNameDetermination();
   }

   public RDXBehaviorActionSequenceEditor(String name, WorkspaceResourceDirectory storageDirectory)
   {
      this.name = name;
      afterNameDetermination();
      this.workspaceFile = new WorkspaceResourceFile(storageDirectory, pascalCasedName + ".json");
   }

   public void afterNameDetermination()
   {
      panel = new ImGuiPanel(name + " Behavior Sequence Editor", this::renderImGuiWidgets, false, true);
      pascalCasedName = FormattingTools.titleToPascalCase(name);
   }

   public void create(RDX3DPanel panel3D,
                      DRCRobotModel robotModel,
                      ROS2Node ros2Node,
                      ROS2SyncedRobotModel syncedRobot,
                      ReferenceFrameLibrary referenceFrameLibrary)
   {
      this.panel3D = panel3D;
      this.robotModel = robotModel;
      this.syncedRobot = syncedRobot;
      this.referenceFrameLibrary = referenceFrameLibrary;
      ros2 = new ROS2ControllerHelper(ros2Node, robotModel);

      executionNextIndexStatusSubscription = ros2.subscribe(BehaviorActionSequence.EXECUTION_NEXT_INDEX_STATUS_TOPIC);
      automaticExecutionStatusSubscription = ros2.subscribe(BehaviorActionSequence.AUTOMATIC_EXECUTION_STATUS_TOPIC);
      sequenceStatusSubscription = ros2.subscribe(BehaviorActionSequence.SEQUENCE_STATUS_TOPIC);
      sequenceStatusSubscription.addCallback(message -> ++receivedSequenceStatusMessageCount);
   }

   public void loadNameFromFile()
   {
      JSONFileTools.load(workspaceFile, jsonNode -> name = jsonNode.get("name").asText());
   }

   public void loadActionsFromFile()
   {
      actionSequence.clear();
      executionNextIndexStatus = 0;
      loading = true;
      LogTools.info("Loading from {}", workspaceFile.getClasspathResource().toString());
      JSONFileTools.load(workspaceFile.getClasspathResourceAsStream(), jsonNode ->
      {
         JSONTools.forEachArrayElement(jsonNode, "actions", actionNode ->
         {
            String actionType = actionNode.get("type").asText();
            RDXBehaviorAction action = switch (actionType)
            {
               case "RDXArmJointAnglesAction" -> new RDXArmJointAnglesAction();
               case "RDXChestOrientationAction" -> new RDXChestOrientationAction();
               case "RDXFootstepAction" -> newFootstepAction(null);
               case "RDXHandConfigurationAction" -> new RDXHandConfigurationAction();
               case "RDXHandPoseAction" -> newHandPoseAction();
               case "RDXHandWrenchAction" -> new RDXHandWrenchAction();
               case "RDXPelvisHeightAction" -> new RDXPelvisHeightAction();
               case "RDXWaitDurationAction" -> new RDXWaitDurationAction();
               case "RDXWalkAction" -> newWalkAction();
               default -> null;
            };

            action.getActionData().loadFromFile(actionNode);
            action.updateAfterLoading();
            insertNewAction(action);
         });
      });
      loading = false;
      commandNextActionIndex(0);
   }

   public void commandNextActionIndex(int nextActionIndex)
   {
      currentActionIndexCommandMessage.setData(nextActionIndex);
      ros2.publish(BehaviorActionSequence.EXECUTION_NEXT_INDEX_COMMAND_TOPIC, currentActionIndexCommandMessage);
   }

   public void saveToFile()
   {
      if (workspaceFile.isFileAccessAvailable())
      {
         LogTools.info("Saving to {}", workspaceFile.getPathForResourceLoadingPathFiltered());
         JSONFileTools.save(workspaceFile, jsonRootObjectNode ->
         {
            jsonRootObjectNode.put("name", name);
            ArrayNode actionsArrayNode = jsonRootObjectNode.putArray("actions");
            for (RDXBehaviorAction behaviorAction : actionSequence)
            {
               ObjectNode actionNode = actionsArrayNode.addObject();
               actionNode.put("type", behaviorAction.getClass().getSimpleName());
               behaviorAction.getActionData().saveToFile(actionNode);
            }
         });
      }
      else
      {
         LogTools.error("Saving not available.");
      }
   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      for (var action : actionSequence)
         action.calculate3DViewPick(input);
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      for (var action : actionSequence)
         action.process3DViewInput(input);
   }

   public void update()
   {
      for (var action : actionSequence)
         action.update();
   }

   public void renderImGuiWidgets()
   {
      ImGui.beginMenuBar();
      if (ImGui.beginMenu(labels.get("File")))
      {
         if (workspaceFile.isFileAccessAvailable() && ImGui.menuItem("Save to JSON"))
         {
            saveToFile();
         }
         if (ImGui.menuItem("Load from JSON"))
         {
            loadActionsFromFile();
         }
         ImGui.endMenu();
      }
//      if (ImGui.beginMenu(labels.get("View")))
//      {
//         ImGui.endMenu();
//      }
      ImGui.endMenuBar();

      if (ImGui.button(labels.get("[+]")))
      {
         for (var action : actionSequence)
         {
            action.getExpanded().set(true);
         }
      }
      ImGuiTools.previousWidgetTooltip("Expand all action settings");
      ImGui.sameLine();
      if (ImGui.button(labels.get("[-]")))
      {
         for (var action : actionSequence)
         {
            action.getExpanded().set(false);
         }
      }
      ImGuiTools.previousWidgetTooltip("Collapse all action settings");
      ImGui.sameLine();

      if (executionNextIndexStatusSubscription.getMessageNotification().poll())
         ++receivedStatusMessageCount;
      executionNextIndexStatus = executionNextIndexStatusSubscription.getLatest().getData();

      if (ImGui.button(labels.get("<")))
      {
         if (executionNextIndexStatus > 0)
            commandNextActionIndex(executionNextIndexStatus - 1);
      }
      ImGuiTools.previousWidgetTooltip("Go to previous action");
      ImGui.sameLine();
      ImGui.text("Index: " + String.format("%03d", executionNextIndexStatus));
      ImGui.sameLine();
      if (ImGui.button(labels.get(">")))
      {
         if (executionNextIndexStatus < actionSequence.size())
            commandNextActionIndex(executionNextIndexStatus + 1);
      }
      ImGuiTools.previousWidgetTooltip("Go to next action");

      long remoteSequenceSize = sequenceStatusSubscription.getLatest().getSequenceSize();
      boolean outOfSync = remoteSequenceSize != actionSequence.size();

      boolean endOfSequence = executionNextIndexStatus >= actionSequence.size();
      if (!endOfSequence && !outOfSync)
      {
         ImGui.sameLine();
         ImGui.text("Execute");
         ImGui.sameLine();

         automaticExecution.set(automaticExecutionStatusSubscription.getLatest().getData());
         if (ImGui.checkbox(labels.get("Autonomously"), automaticExecution))
         {
            automaticExecutionCommandMessage.setData(automaticExecution.get());
            ros2.publish(BehaviorActionSequence.AUTOMATIC_EXECUTION_COMMAND_TOPIC, automaticExecutionCommandMessage);
         }
         ImGuiTools.previousWidgetTooltip("Enables autonomous execution. Will immediately start executing when checked.");
         if (!automaticExecution.get())
         {
            ImGui.sameLine();
            if (ImGui.button(labels.get("Manually")))
            {
               ros2.publish(BehaviorActionSequence.MANUALLY_EXECUTE_NEXT_ACTION_TOPIC, manuallyExecuteNextActionMessage);
            }
            ImGuiTools.previousWidgetTooltip("Executes the next action.");
         }
      }

      // TODO: Automatically sync between this UI and remote process
      if (ImGui.button("Send to robot"))
      {
         sendToRobot();
      }
      ImGui.sameLine();
      ImGui.text("# " + receivedSequenceStatusMessageCount);
      if (outOfSync)
      {
         ImGui.sameLine();
         ImGui.text(String.format("Out of sync! # Actions: Local: %d Remote: %d", actionSequence.size(), remoteSequenceSize));
         ImGuiTools.previousWidgetTooltip("Try clicking \"Send to robot\"");
      }

      ImGui.text(String.format("Status # %d: Current action:", receivedStatusMessageCount));
      ImGui.sameLine();
      endOfSequence = executionNextIndexStatus >= actionSequence.size();
      if (endOfSequence)
         ImGui.text("End of sequence.");
      else
         ImGui.text(actionSequence.get(executionNextIndexStatus).getNameForDisplay());
      ImGui.separator();

      // This, paired with the endChild call after, allows this area to scroll separately
      // from the rest, so the top controls are still available while editing later parts
      // of the sequence.
      ImGui.beginChild(labels.get("childRegion"));

      reorderRequest.setLeft(-1);
      for (int i = 0; i < actionSequence.size(); i++)
      {
         RDXBehaviorAction action = actionSequence.get(i);
         if (ImGui.radioButton(labels.get("", "playbackNextIndex", i), executionNextIndexStatus == i))
         {
            commandNextActionIndex(i);
         }
         ImGuiTools.previousWidgetTooltip("Next for excecution");
         ImGui.sameLine();
         ImGui.checkbox(labels.get("", "selected", i), action.getSelected());
         ImGuiTools.previousWidgetTooltip("Selected");
         ImGui.sameLine();
         ImGui.checkbox(labels.get("", "expanded", i), action.getExpanded());
         ImGuiTools.previousWidgetTooltip("Expanded");
         ImGui.sameLine();
         ImGui.text(i + ": " + action.getNameForDisplay());
         ImGui.sameLine();
         if (i > 0)
         {
            if (ImGui.button(labels.get("^", i)))
            {
               reorderRequest.setLeft(i);
               reorderRequest.setRight(0);
            }
            ImGuiTools.previousWidgetTooltip("Swap with previous action (in ordering)");
            ImGui.sameLine();
         }
         if (i < actionSequence.size() - 1)
         {
            if (ImGui.button(labels.get("v", i)))
            {
               reorderRequest.setLeft(i);
               reorderRequest.setRight(1);
            }
            ImGuiTools.previousWidgetTooltip("Swap with next action (in ordering)");
            ImGui.sameLine();
         }
         if (ImGui.button(labels.get("X", i)))
         {
            RDXBehaviorAction removedAction = actionSequence.remove(i);
            commandNextActionIndex(actionSequence.size());
         }

         action.renderImGuiWidgets();
      }

      int indexToMove = reorderRequest.getLeft();
      if (indexToMove > -1)
      {
         int destinationIndex = reorderRequest.getRight() == 0 ? indexToMove - 1 : indexToMove + 1;
         actionSequence.add(destinationIndex, actionSequence.remove(indexToMove));
      }

      ImGui.separator();

      RDXBehaviorAction newAction = null;
      if (ImGui.button(labels.get("Add Walk")))
      {
         newAction = newWalkAction();
      }
      ImGui.text("Add Hand Pose:");
      ImGui.sameLine();
      for (var side : RobotSide.values)
      {
         if (ImGui.button(labels.get(side.getPascalCaseName(), "HandPose")))
         {
            RDXHandPoseAction handPoseAction = newHandPoseAction();
            // Set the new action to where the last one was for faster authoring
            handPoseAction.setSide(side, true, findNextPreviousHandPoseAction(side));
            newAction = handPoseAction;
         }
         if (side.ordinal() < 1)
            ImGui.sameLine();
      }
      ImGui.text("Add Hand Wrench:");
      ImGui.sameLine();
      for (var side : RobotSide.values)
      {
         if (ImGui.button(labels.get(side.getPascalCaseName(), "HandWrench")))
         {
            RDXHandWrenchAction handWrenchAction = new RDXHandWrenchAction();
            handWrenchAction.getActionData().setSide(side);
            newAction = handWrenchAction;
         }
         if (side.ordinal() < 1)
            ImGui.sameLine();
      }
      if (ImGui.button(labels.get("Add Hand Configuration")))
      {
         newAction = new RDXHandConfigurationAction();
      }
      if (ImGui.button(labels.get("Add Chest Orientation")))
      {
         newAction = new RDXChestOrientationAction();
      }
      if (ImGui.button(labels.get("Add Pelvis Height")))
      {
         newAction = new RDXPelvisHeightAction();
      }
      if (ImGui.button(labels.get("Add Arm Joint Angles")))
      {
         newAction = new RDXArmJointAnglesAction();
      }
      ImGui.text("Add Footstep:");
      ImGui.sameLine();
      for (var side : RobotSide.values)
      {
         if (ImGui.button(labels.get(side.getPascalCaseName(), 1)))
         {
            // Set the new action to where the last one was for faster authoring
            RDXFootstepAction footstepAction = newFootstepAction(findNextPreviousFootstepAction());
            footstepAction.setSide(side, true);
            newAction = footstepAction;
         }
         if (side.ordinal() < 1)
            ImGui.sameLine();
      }
      if (ImGui.button(labels.get("Add Wait")))
      {
         newAction = new RDXWaitDurationAction();
      }

      if (newAction != null)
         insertNewAction(newAction);

      ImGui.endChild();
   }

   public void sendToRobot()
   {
      long updateUUID = UUID.randomUUID().getLeastSignificantBits();
      ActionSequenceUpdateMessage actionSequenceUpdateMessage = new ActionSequenceUpdateMessage();
      actionSequenceUpdateMessage.setSequenceUpdateUuid(updateUUID);
      actionSequenceUpdateMessage.setSequenceSize(actionSequence.size());
      ros2.publish(BehaviorActionSequence.SEQUENCE_COMMAND_TOPIC, actionSequenceUpdateMessage);

      for (int i = 0; i < actionSequence.size(); i++)
      {
         RDXBehaviorAction action = actionSequence.get(i);
         if (action instanceof RDXArmJointAnglesAction armJointAnglesAction)
         {
            ArmJointAnglesActionMessage armJointAnglesActionMessage = new ArmJointAnglesActionMessage();
            armJointAnglesActionMessage.getActionInformation().setActionIndex(i);
            armJointAnglesActionMessage.getActionInformation().setSequenceUpdateUuid(updateUUID);
            armJointAnglesAction.getActionData().toMessage(armJointAnglesActionMessage);
            ros2.publish(BehaviorActionSequence.ARM_JOINT_ANGLES_UPDATE_TOPIC, armJointAnglesActionMessage);
         }
         else if (action instanceof RDXChestOrientationAction chestOrientationAction)
         {
            ChestOrientationActionMessage chestOrientationActionMessage = new ChestOrientationActionMessage();
            chestOrientationActionMessage.getActionInformation().setActionIndex(i);
            chestOrientationActionMessage.getActionInformation().setSequenceUpdateUuid(updateUUID);
            chestOrientationAction.getActionData().toMessage(chestOrientationActionMessage);
            ros2.publish(BehaviorActionSequence.CHEST_ORIENTATION_UPDATE_TOPIC, chestOrientationActionMessage);
         }
         else if (action instanceof RDXFootstepAction footstepAction)
         {
            FootstepActionMessage footstepActionMessage = new FootstepActionMessage();
            footstepActionMessage.getActionInformation().setActionIndex(i);
            footstepActionMessage.getActionInformation().setSequenceUpdateUuid(updateUUID);
            footstepAction.getActionData().toMessage(footstepActionMessage);
            ros2.publish(BehaviorActionSequence.FOOTSTEP_UPDATE_TOPIC, footstepActionMessage);
         }
         else if (action instanceof RDXHandConfigurationAction handConfigurationAction)
         {
            HandConfigurationActionMessage handConfigurationActionMessage = new HandConfigurationActionMessage();
            handConfigurationActionMessage.getActionInformation().setActionIndex(i);
            handConfigurationActionMessage.getActionInformation().setSequenceUpdateUuid(updateUUID);
            handConfigurationAction.getActionData().toMessage(handConfigurationActionMessage);
            ros2.publish(BehaviorActionSequence.HAND_CONFIGURATION_UPDATE_TOPIC, handConfigurationActionMessage);
         }
         else if (action instanceof RDXHandPoseAction handPoseAction)
         {
            HandPoseActionMessage handPoseActionMessage = new HandPoseActionMessage();
            handPoseActionMessage.getActionInformation().setActionIndex(i);
            handPoseActionMessage.getActionInformation().setSequenceUpdateUuid(updateUUID);
            handPoseAction.getActionData().toMessage(handPoseActionMessage);
            ros2.publish(BehaviorActionSequence.HAND_POSE_UPDATE_TOPIC, handPoseActionMessage);
         }
         else if (action instanceof RDXHandWrenchAction handWrenchAction)
         {
            HandWrenchActionMessage handWrenchActionMessage = new HandWrenchActionMessage();
            handWrenchActionMessage.getActionInformation().setActionIndex(i);
            handWrenchActionMessage.getActionInformation().setSequenceUpdateUuid(updateUUID);
            handWrenchAction.getActionData().toMessage(handWrenchActionMessage);
            ros2.publish(BehaviorActionSequence.HAND_WRENCH_UPDATE_TOPIC, handWrenchActionMessage);
         }
         else if (action instanceof RDXPelvisHeightAction pelvisHeightAction)
         {
            PelvisHeightActionMessage pelvisHeightActionMessage = new PelvisHeightActionMessage();
            pelvisHeightActionMessage.getActionInformation().setActionIndex(i);
            pelvisHeightActionMessage.getActionInformation().setSequenceUpdateUuid(updateUUID);
            pelvisHeightAction.getActionData().toMessage(pelvisHeightActionMessage);
            ros2.publish(BehaviorActionSequence.PELVIS_HEIGHT_UPDATE_TOPIC, pelvisHeightActionMessage);
         }
         else if (action instanceof RDXWaitDurationAction waitDurationAction)
         {
            WaitDurationActionMessage waitDurationActionMessage = new WaitDurationActionMessage();
            waitDurationActionMessage.getActionInformation().setActionIndex(i);
            waitDurationActionMessage.getActionInformation().setSequenceUpdateUuid(updateUUID);
            waitDurationAction.getActionData().toMessage(waitDurationActionMessage);
            ros2.publish(BehaviorActionSequence.WAIT_DURATION_UPDATE_TOPIC, waitDurationActionMessage);
         }
         else if (action instanceof RDXWalkAction walkAction)
         {
            WalkActionMessage walkActionMessage = new WalkActionMessage();
            walkActionMessage.getActionInformation().setActionIndex(i);
            walkActionMessage.getActionInformation().setSequenceUpdateUuid(updateUUID);
            walkAction.getActionData().toMessage(walkActionMessage);
            ros2.publish(BehaviorActionSequence.WALK_UPDATE_TOPIC, walkActionMessage);
         }
      }
   }

   public void playActions(boolean play)
   {
      automaticExecutionCommandMessage.setData(play);
      ros2.publish(BehaviorActionSequence.AUTOMATIC_EXECUTION_COMMAND_TOPIC, automaticExecutionCommandMessage);
   }

   public void rewindActions(boolean play)
   {
      playbackAutomaticExecutionCommandMessage.setData(play);
      ros2.publish(BehaviorActionSequence.PLAYBACK_AUTOMATIC_EXECUTION_COMMAND_TOPIC, playbackAutomaticExecutionCommandMessage);
   }

   private RDXFootstepAction findNextPreviousFootstepAction()
   {
      RDXFootstepAction previousAction = null;
      for (int i = 0; i < executionNextIndexStatus; i++)
         if (actionSequence.get(i) instanceof RDXFootstepAction)
            previousAction = (RDXFootstepAction) actionSequence.get(i);
      return previousAction;
   }

   private RDXHandPoseAction findNextPreviousHandPoseAction(RobotSide side)
   {
      RDXHandPoseAction previousAction = null;
      for (int i = 0; i < executionNextIndexStatus - 1; i++)
      {
         if (actionSequence.get(i) instanceof RDXHandPoseAction
             && ((RDXHandPoseAction) actionSequence.get(i)).getActionData().getSide() == side)
         {
            previousAction = (RDXHandPoseAction) actionSequence.get(i);
         }
      }
      return previousAction;
   }

   private void insertNewAction(RDXBehaviorAction action)
   {
      actionSequence.add(executionNextIndexStatus, action);
      for (int i = 0; i < actionSequence.size(); i++)
      {
         // When loading, we want to deselect all the actions, otherwise the last one ends up being selected.
         actionSequence.get(i).getSelected().set(!loading && i == executionNextIndexStatus);
      }
      executionNextIndexStatus++;
   }

   private RDXFootstepAction newFootstepAction(RDXFootstepAction possiblyNullPreviousFootstepAction)
   {
      return new RDXFootstepAction(panel3D, robotModel, syncedRobot, referenceFrameLibrary, possiblyNullPreviousFootstepAction);
   }

   private RDXWalkAction newWalkAction()
   {
      return new RDXWalkAction(panel3D, robotModel, referenceFrameLibrary);
   }

   private RDXHandPoseAction newHandPoseAction()
   {
      return new RDXHandPoseAction(panel3D, robotModel, syncedRobot, syncedRobot.getFullRobotModel(), referenceFrameLibrary);
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (panel.getIsShowing().get())
         for (var action : actionSequence)
            action.getRenderables(renderables, pool);
   }

   public ImGuiPanel getPanel()
   {
      return panel;
   }

   public String getName()
   {
      return name;
   }

   public WorkspaceResourceFile getWorkspaceFile()
   {
      return workspaceFile;
   }

   public RigidBodyTransform getInitialHandTransform(RobotSide side)
   {
      RigidBodyTransform initialHandTransform = new RigidBodyTransform();
      for (var action : actionSequence)
      {
         if (action instanceof RDXHandPoseAction)
         {
            HandPoseActionData handPoseData = (HandPoseActionData) action.getActionData();
            if (handPoseData.getSide() == side)
            {
               initialHandTransform.set(handPoseData.getTransformToParent());
            }
            break;
         }
      }
      return initialHandTransform;
   }
}
