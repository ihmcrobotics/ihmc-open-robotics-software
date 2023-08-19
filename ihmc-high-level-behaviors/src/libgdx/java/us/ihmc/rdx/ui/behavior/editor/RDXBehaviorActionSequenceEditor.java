package us.ihmc.rdx.ui.behavior.editor;

import behavior_msgs.msg.dds.*;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.ImGui;
import imgui.type.ImBoolean;
import org.apache.commons.lang3.mutable.MutableBoolean;
import org.apache.commons.lang3.tuple.MutablePair;
import std_msgs.msg.dds.Bool;
import std_msgs.msg.dds.Empty;
import std_msgs.msg.dds.Int32;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.BehaviorActionData;
import us.ihmc.behaviors.sequence.BehaviorActionSequence;
import us.ihmc.commons.FormattingTools;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.ui.behavior.editor.actions.*;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.io.*;

import javax.annotation.Nullable;
import java.util.ArrayList;
import java.util.LinkedList;

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
   private final ImBoolean automaticExecution = new ImBoolean(false);
   private String name;
   @Nullable
   private WorkspaceResourceFile workspaceFile = null;
   private final LinkedList<RDXBehaviorAction> actionSequence = new LinkedList<>();
   private String pascalCasedName;
   private RDX3DPanel panel3D;
   private DRCRobotModel robotModel;
   private ROS2SyncedRobotModel syncedRobot;
   private RobotCollisionModel selectionCollisionModel;
   private ReferenceFrameLibrary referenceFrameLibrary;
   private ROS2ControllerHelper ros2;
   private final MutablePair<Integer, Integer> reorderRequest = MutablePair.of(-1, 0);
   private volatile long receivedSequenceStatusMessageCount = 0;
   private long receivedStatusMessageCount = 0;
   private int executionNextIndexStatus;
   private final Int32 currentActionIndexCommandMessage = new Int32();
   private IHMCROS2Input<Int32> executionNextIndexStatusSubscription;
   private IHMCROS2Input<Bool> automaticExecutionStatusSubscription;
   private IHMCROS2Input<ActionSequenceUpdateMessage> sequenceStatusSubscription;
   private final Empty manuallyExecuteNextActionMessage = new Empty();
   private final Bool automaticExecutionCommandMessage = new Bool();
   private final ArrayList<BehaviorActionData> actionDataForMessage = new ArrayList<>();
   private final ActionSequenceUpdateMessage actionSequenceUpdateMessage = new ActionSequenceUpdateMessage();
   private boolean outOfSync = true;

   public void clear()
   {
      workspaceFile = null;
   }

   public void createNewSequence(String name, WorkspaceResourceDirectory storageDirectory)
   {
      this.name = name;
      afterNameDetermination();
      this.workspaceFile = new WorkspaceResourceFile(storageDirectory, pascalCasedName + ".json");
   }

   public void changeFileToLoadFrom(WorkspaceResourceFile fileToLoadFrom)
   {
      this.workspaceFile = fileToLoadFrom;
      loadNameFromFile();
      afterNameDetermination();
   }

   public void afterNameDetermination()
   {
      pascalCasedName = FormattingTools.titleToPascalCase(name);
   }

   public void create(RDX3DPanel panel3D,
                      DRCRobotModel robotModel,
                      ROS2Node ros2Node,
                      ROS2SyncedRobotModel syncedRobot,
                      RobotCollisionModel selectionCollisionModel,
                      ReferenceFrameLibrary referenceFrameLibrary)
   {
      this.panel3D = panel3D;
      this.robotModel = robotModel;
      this.syncedRobot = syncedRobot;
      this.selectionCollisionModel = selectionCollisionModel;
      this.referenceFrameLibrary = referenceFrameLibrary;
      ros2 = new ROS2ControllerHelper(ros2Node, robotModel);

      executionNextIndexStatusSubscription = ros2.subscribe(BehaviorActionSequence.EXECUTION_NEXT_INDEX_STATUS_TOPIC);
      automaticExecutionStatusSubscription = ros2.subscribe(BehaviorActionSequence.AUTOMATIC_EXECUTION_STATUS_TOPIC);
      sequenceStatusSubscription = ros2.subscribe(BehaviorActionSequence.SEQUENCE_STATUS_TOPIC);
      sequenceStatusSubscription.addCallback(message -> ++receivedSequenceStatusMessageCount);
   }

   public void loadNameFromFile()
   {
      JSONFileTools.load(workspaceFile.getFilesystemFile(), jsonNode -> name = jsonNode.get("name").asText());
   }

   public boolean loadActionsFromFile()
   {
      actionSequence.clear();
      executionNextIndexStatus = 0;
      LogTools.info("Loading from {}", workspaceFile.getFilesystemFile());
      MutableBoolean successfullyLoadedActions = new MutableBoolean(true);
      JSONFileTools.load(workspaceFile.getFilesystemFile(), jsonNode ->
      {
         JSONTools.forEachArrayElement(jsonNode, "actions", actionNode ->
         {
            String actionTypeName = actionNode.get("type").asText();
            RDXBehaviorAction action = RDXActionSequenceTools.createBlankAction(actionTypeName,
                                                                                robotModel,
                                                                                syncedRobot,
                                                                                selectionCollisionModel,
                                                                                panel3D,
                                                                                referenceFrameLibrary,
                                                                                ros2);
            if (action != null)
            {
               action.getActionData().loadFromFile(actionNode);
               action.updateAfterLoading();
               action.update();
               actionSequence.add(action);
               action.getSelected().set(false);
               action.getExpanded().set(false);
            }
            else
            {
               successfullyLoadedActions.setValue(false);
            }
         });
      });
      if (successfullyLoadedActions.getValue())
      {
         commandNextActionIndex(0);
         return true;
      }

      return false;
   }

   private void commandNextActionIndex(int nextActionIndex)
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

   public void calculateVRPick(RDXVRContext vrContext)
   {
      for (var action : actionSequence)
         action.calculateVRPick(vrContext);
   }

   public void processVRInput(RDXVRContext vrContext)
   {
      for (var action : actionSequence)
         action.processVRInput(vrContext);
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
      for (int i = 0; i < actionSequence.size(); i++)
      {
         RDXBehaviorAction action = actionSequence.get(i);
         action.setActionIndex(i);
         action.setActionNextExcecutionIndex(executionNextIndexStatus);
         action.update();
      }
   }

   public void renderFileMenu()
   {
      if (workspaceFile.isFileAccessAvailable() && ImGui.menuItem("Save to JSON"))
      {
         saveToFile();
      }
      if (ImGui.menuItem("Load from JSON"))
      {
         if (!loadActionsFromFile())
         {
            LogTools.warn("Invalid action!");
            actionSequence.clear();
         }
      }
   }

   public void renderImGuiWidgets()
   {
      if (isCleared())
      {
         ImGui.text("No behavior selected.");
      }
      else
      {
         renderSequencePrimaryControlsArea();

         ImGui.separator();

         // This, paired with the endChild call after, allows this area to scroll separately
         // from the rest, so the top controls are still available while editing later parts
         // of the sequence.
         ImGui.beginChild(labels.get("childRegion"));

         renderInteractableActionListArea();

         ImGui.separator();

         renderActionCreationArea();

         ImGui.endChild();
      }
   }

   private void renderSequencePrimaryControlsArea()
   {
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
      if (sequenceStatusSubscription.getMessageNotification().poll())
      {
         RDXActionSequenceTools.packActionSequenceUpdateMessage(actionSequence, actionDataForMessage, actionSequenceUpdateMessage);
         outOfSync = !sequenceStatusSubscription.getMessageNotification().read().equals(actionSequenceUpdateMessage);

         if (outOfSync)
         {
            // Automatically attempt to get back in sync
            RDXActionSequenceTools.packActionSequenceUpdateMessage(actionSequence, actionDataForMessage, actionSequenceUpdateMessage);
            ros2.publish(BehaviorActionSequence.SEQUENCE_COMMAND_TOPIC, actionSequenceUpdateMessage);
         }
      }

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
      {
         RDXBehaviorAction nextExecutionAction = actionSequence.get(executionNextIndexStatus);
         ImGui.text("%s (%s)".formatted(nextExecutionAction.getDescription(), nextExecutionAction.getActionTypeTitle()));
      }
   }

   private void renderInteractableActionListArea()
   {
      reorderRequest.setLeft(-1);
      for (int i = 0; i < actionSequence.size(); i++)
      {
         RDXBehaviorAction action = actionSequence.get(i);

         if (!action.getExpanded().get())
         {
            if (ImGui.button(labels.get("[+]", "expand", i)))
               action.getExpanded().set(true);
            ImGuiTools.previousWidgetTooltip("Expand action settings");
         }
         else
         {
            if (ImGui.button(labels.get("[-]", "collapse", i)))
               action.getExpanded().set(false);
            ImGuiTools.previousWidgetTooltip("Collapse action settings");
         }
         ImGui.sameLine();

         ImGui.sameLine();
         ImGui.text("Index %d".formatted(i));
         ImGui.sameLine();
         if (ImGui.radioButton(labels.get("", "playbackNextIndex", i), executionNextIndexStatus == i))
         {
            commandNextActionIndex(i);
         }
         ImGuiTools.previousWidgetTooltip("Next for excecution");
         action.getDescription().set(action.getActionData().getDescription());
         ImGui.sameLine();
         ImGui.pushItemWidth(9.0f + action.getActionData().getDescription().length() * 7.0f);
         ImGuiTools.inputText(labels.get("", "description", i), action.getDescription());
         action.getActionData().setDescription(action.getDescription().get());
         ImGui.popItemWidth();
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

         if (action.getExpanded().get())
         {
            action.getSelected().renderImGuiWidget();
            ImGuiTools.previousWidgetTooltip("(Show gizmo)");
            ImGui.sameLine();
            ImGui.text("Type: %s".formatted(action.getActionTypeTitle()));
         }

         action.renderImGuiWidgets();

         ImGui.separator();
      }

      int indexToMove = reorderRequest.getLeft();
      if (indexToMove > -1)
      {
         int destinationIndex = reorderRequest.getRight() == 0 ? indexToMove - 1 : indexToMove + 1;
         actionSequence.add(destinationIndex, actionSequence.remove(indexToMove));
      }
   }

   private void renderActionCreationArea()
   {
      RDXBehaviorAction newAction = null;

      ReferenceFrame nextPreviousParentFrame = findNextPreviousParentFrame();

      if (ImGui.button(labels.get("Add Walk")))
      {
         RDXWalkAction walkAction = new RDXWalkAction(panel3D, robotModel, referenceFrameLibrary);
         walkAction.setToReferenceFrame(syncedRobot.getReferenceFrames().getMidFeetZUpFrame());
         if (nextPreviousParentFrame != null)
            walkAction.getActionData().changeParentFrameWithoutMoving(nextPreviousParentFrame);
         newAction = walkAction;
      }
      ImGui.text("Add Hand Pose:");
      ImGui.sameLine();
      for (var side : RobotSide.values)
      {
         if (ImGui.button(labels.get(side.getPascalCaseName(), "HandPose")))
         {
            RDXHandPoseAction handPoseAction = new RDXHandPoseAction(panel3D,
                                                                     robotModel,
                                                                     syncedRobot.getFullRobotModel(),
                                                                     selectionCollisionModel,
                                                                     referenceFrameLibrary,
                                                                     ros2);
            // Set the new action to where the last one was for faster authoring
            handPoseAction.setSide(side);
            RDXHandPoseAction nextPreviousHandPoseAction = findNextPreviousHandPoseAction(side);
            if (nextPreviousHandPoseAction != null)
            {
               handPoseAction.setIncludingFrame(nextPreviousHandPoseAction.getReferenceFrame().getParent(),
                                                nextPreviousHandPoseAction.getReferenceFrame().getTransformToParent());
            }
            else // set to current robot's hand pose
            {
               handPoseAction.setToReferenceFrame(syncedRobot.getReferenceFrames().getHandFrame(side));
            }
            if (nextPreviousParentFrame != null)
               handPoseAction.getActionData().changeParentFrameWithoutMoving(nextPreviousParentFrame);
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
            RDXFootstepAction footstepAction = new RDXFootstepAction(panel3D, robotModel, syncedRobot, referenceFrameLibrary);
            // Set the new action to where the last one was for faster authoring
            footstepAction.setSide(side);
            RDXFootstepAction nextPreviousFootstepAction = findNextPreviousFootstepAction();
            if (nextPreviousFootstepAction != null)
            {
               footstepAction.setIncludingFrame(nextPreviousFootstepAction.getReferenceFrame().getParent(),
                                                nextPreviousFootstepAction.getReferenceFrame().getTransformToParent());
            }
            else // set to current robot's foot pose
            {
               footstepAction.setToReferenceFrame(syncedRobot.getReferenceFrames().getSoleFrame(side));
            }
            if (nextPreviousParentFrame != null)
               footstepAction.getActionData().changeParentFrameWithoutMoving(nextPreviousParentFrame);
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
   }

   /**
    * @return This used to find the most likely desired frame that a new action will
    *         be specified in, by traversing the sequence backwards and finding the
    *         first action that is specified in a frame, and returning that frame.
    *         This helps the authoring process by initializing new actions with
    *         spatially consistent values.
    */
   private ReferenceFrame findNextPreviousParentFrame()
   {
      for (int i = Math.min(executionNextIndexStatus, actionSequence.size() - 1); i >= 0; i--)
      {
         if (actionSequence.get(i) instanceof RDXFootstepAction footstepAction)
         {
            return footstepAction.getActionData().getParentReferenceFrame();
         }
         else if (actionSequence.get(i) instanceof RDXHandPoseAction handPoseAction)
         {
            return handPoseAction.getActionData().getParentReferenceFrame();
         }
         else if (actionSequence.get(i) instanceof RDXWalkAction walkAction)
         {
            return walkAction.getActionData().getParentReferenceFrame();
         }
      }
      return null;
   }

   private RDXFootstepAction findNextPreviousFootstepAction()
   {
      RDXFootstepAction previousAction = null;
      for (int i = 0; i < executionNextIndexStatus + 1 && i < actionSequence.size(); i++)
      {
         if (actionSequence.get(i) instanceof RDXFootstepAction footstepAction)
         {
            previousAction = footstepAction;
         }
      }
      return previousAction;
   }

   private RDXHandPoseAction findNextPreviousHandPoseAction(RobotSide side)
   {
      RDXHandPoseAction previousAction = null;
      for (int i = 0; i < executionNextIndexStatus + 1 && i < actionSequence.size(); i++)
      {
         if (actionSequence.get(i) instanceof RDXHandPoseAction handPoseAction)
         {
            if (handPoseAction.getActionData().getSide() == side)
            {
               previousAction = (RDXHandPoseAction) actionSequence.get(i);
            }
         }
      }
      return previousAction;
   }

   private void insertNewAction(RDXBehaviorAction action)
   {
      action.updateAfterLoading();
      action.update();

      int insertionIndex = executionNextIndexStatus == actionSequence.size() ? executionNextIndexStatus : executionNextIndexStatus + 1;
      actionSequence.add(insertionIndex, action);

      for (int i = 0; i < actionSequence.size(); i++)
      {
         // When loading, we want to deselect all the actions, otherwise the last one ends up being selected.
         actionSequence.get(i).getSelected().set(i == insertionIndex);
      }
      commandNextActionIndex(executionNextIndexStatus + 1);
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (RDXBehaviorAction action : actionSequence)
         action.getRenderables(renderables, pool);
   }

   public void destroy()
   {
      automaticExecutionStatusSubscription.destroy();
      executionNextIndexStatusSubscription.destroy();
      sequenceStatusSubscription.destroy();
   }

   public String getName()
   {
      return name;
   }

   public WorkspaceResourceFile getWorkspaceFile()
   {
      return workspaceFile;
   }

   public boolean isCleared()
   {
      return workspaceFile == null;
   }
}
