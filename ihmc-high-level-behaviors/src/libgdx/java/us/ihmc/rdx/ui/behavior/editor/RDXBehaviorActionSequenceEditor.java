package us.ihmc.rdx.ui.behavior.editor;

import behavior_msgs.msg.dds.ActionExecutionStatusMessage;
import behavior_msgs.msg.dds.ActionSequenceUpdateMessage;
import behavior_msgs.msg.dds.ActionsExecutionStatusMessage;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.ImVec2;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiStyleVar;
import imgui.internal.ImGui;
import imgui.internal.flag.ImGuiItemFlags;
import imgui.type.ImBoolean;
import org.apache.commons.lang3.mutable.MutableBoolean;
import org.apache.commons.lang3.tuple.MutablePair;
import std_msgs.msg.dds.Bool;
import std_msgs.msg.dds.Empty;
import std_msgs.msg.dds.Int32;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.BehaviorActionSequence;
import us.ihmc.behaviors.sequence.BehaviorActionSequenceTools;
import us.ihmc.commons.FormattingTools;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.idl.IDLSequence;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.imgui.ImGuiFlashingText;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.behavior.editor.actions.*;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SidedObject;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.io.JSONTools;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.tools.io.WorkspaceResourceFile;

import javax.annotation.Nullable;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

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
   private final ImVec2 calcDescriptionTextSize = new ImVec2();
   private final ImVec2 expandButtonSize = new ImVec2();
   private float longestDescriptionLength;
   private String name;
   @Nullable
   private WorkspaceResourceFile workspaceFile = null;
   private final LinkedList<RDXBehaviorAction> actionSequence = new LinkedList<>();
   private String pascalCasedName;
   private RDXBaseUI baseUI;
   private RDX3DPanel panel3D;
   private DRCRobotModel robotModel;
   private ROS2SyncedRobotModel syncedRobot;
   private RobotCollisionModel selectionCollisionModel;
   private ReferenceFrameLibrary referenceFrameLibrary;
   private ROS2ControllerHelper ros2ControllerHelper;
   private final TypedNotification<Class<? extends RDXBehaviorAction>> actionToCreate = new TypedNotification<>();
   private RobotSide sideOfNewAction;
   private final MutablePair<Integer, Integer> reorderRequest = MutablePair.of(-1, 0);
   private volatile long receivedSequenceStatusMessageCount = 0;
   private long receivedStatusMessageCount = 0;
   private int executionNextIndexStatus;
   private final Int32 currentActionIndexCommandMessage = new Int32();
   private IHMCROS2Input<Int32> executionNextIndexStatusSubscription;
   private IHMCROS2Input<Bool> automaticExecutionStatusSubscription;
   private IHMCROS2Input<std_msgs.msg.dds.String> executionNextIndexRejectionSubscription;
   private IHMCROS2Input<ActionSequenceUpdateMessage> sequenceStatusSubscription;
   private IHMCROS2Input<ActionsExecutionStatusMessage> executionStatusSubscription;
   private final List<ActionExecutionStatusMessage> executionStatusMessagesToDisplay = new ArrayList<>();
   private final List<RDXBehaviorAction> currentlyExecutingActions = new ArrayList<>();
   private final Empty manuallyExecuteNextActionMessage = new Empty();
   private final Bool automaticExecutionCommandMessage = new Bool();
   private final ActionSequenceUpdateMessage actionSequenceUpdateMessage = new ActionSequenceUpdateMessage();
   private boolean outOfSync = true;
   private final RDXMultipleActionProgressBars multipleActionProgressBars = new RDXMultipleActionProgressBars();
   private final ImGuiFlashingText executionRejectionTooltipText = new ImGuiFlashingText(Color.RED.toIntBits());
   private long lastManualExecutionConfirmTime = 0;
   private int lastManualExecutionActionIndex = -1;

   public void clear()
   {
      for (RDXBehaviorAction action : actionSequence)
         action.updateBeforeRemoving();
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

   public void create(RDXBaseUI baseUI,
                      RDX3DPanel panel3D,
                      DRCRobotModel robotModel,
                      ROS2Node ros2Node,
                      ROS2SyncedRobotModel syncedRobot,
                      RobotCollisionModel selectionCollisionModel,
                      ReferenceFrameLibrary referenceFrameLibrary)
   {
      this.baseUI = baseUI;
      this.panel3D = panel3D;
      this.robotModel = robotModel;
      this.syncedRobot = syncedRobot;
      this.selectionCollisionModel = selectionCollisionModel;
      this.referenceFrameLibrary = referenceFrameLibrary;
      ros2ControllerHelper = new ROS2ControllerHelper(ros2Node, robotModel);

      executionNextIndexStatusSubscription = ros2ControllerHelper.subscribe(BehaviorActionSequence.EXECUTION_NEXT_INDEX_STATUS_TOPIC);
      automaticExecutionStatusSubscription = ros2ControllerHelper.subscribe(BehaviorActionSequence.AUTOMATIC_EXECUTION_STATUS_TOPIC);
      executionNextIndexRejectionSubscription = ros2ControllerHelper.subscribe(BehaviorActionSequence.EXECUTION_NEXT_INDEX_REJECTION_TOPIC);
      sequenceStatusSubscription = ros2ControllerHelper.subscribe(BehaviorActionSequence.SEQUENCE_STATUS_TOPIC);
      sequenceStatusSubscription.addCallback(message -> ++receivedSequenceStatusMessageCount);
      executionStatusSubscription = ros2ControllerHelper.subscribe(BehaviorActionSequence.ACTIONS_EXECUTION_STATUS);
   }

   public void loadNameFromFile()
   {
      JSONFileTools.load(workspaceFile.getFilesystemFile(), jsonNode -> name = jsonNode.get("name").asText());
   }

   public boolean loadActionsFromFile()
   {
      for (RDXBehaviorAction action : actionSequence)
         action.updateBeforeRemoving();
      actionSequence.clear();
      executionNextIndexStatus = 0;
      lastManualExecutionActionIndex = -1;
      LogTools.info("Loading from {}", workspaceFile.getFilesystemFile());
      MutableBoolean successfullyLoadedActions = new MutableBoolean(true);
      JSONFileTools.load(workspaceFile.getFilesystemFile(), jsonNode ->
      {
         JSONTools.forEachArrayElement(jsonNode, "actions", actionNode ->
         {
            String actionTypeName = actionNode.get("type").asText();
            RDXBehaviorAction action = RDXActionSequenceTools.createBlankAction(actionTypeName,
                                                                                this,
                                                                                robotModel,
                                                                                syncedRobot,
                                                                                selectionCollisionModel,
                                                                                baseUI,
                                                                                panel3D,
                                                                                referenceFrameLibrary,
                                                                                ros2ControllerHelper);
            if (action != null)
            {
               try
               {
                  action.getState().loadFromFile(actionNode);
               }
               catch (Exception exception)
               {
                  exception.printStackTrace();
                  LogTools.error("Unable to load action sequence file: {}", workspaceFile.getFilesystemFile().getFileName());
               }
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
      ros2ControllerHelper.publish(BehaviorActionSequence.EXECUTION_NEXT_INDEX_COMMAND_TOPIC, currentActionIndexCommandMessage);
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
               behaviorAction.getState().saveToFile(actionNode);
               behaviorAction.updateAfterLoading();
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
      if (actionToCreate.poll())
      {
         Class<? extends RDXBehaviorAction> actionType = actionToCreate.read();
         RDXBehaviorAction newAction = RDXActionSequenceTools.createBlankAction(actionType,
                                                                                this,
                                                                                robotModel,
                                                                                syncedRobot,
                                                                                selectionCollisionModel,
                                                                                baseUI,
                                                                                panel3D,
                                                                                referenceFrameLibrary,
                                                                                ros2ControllerHelper);

         if (newAction instanceof RDXWalkAction walkAction)
         {
            walkAction.setToReferenceFrame(syncedRobot.getReferenceFrames().getMidFeetZUpFrame());
            walkAction.getState().getGoalFrame().update(findConvenientParentFrameName(walkAction, null));
         }
         else if (newAction instanceof RDXHandPoseAction handPoseAction)
         {
            // Set the new action to where the last one was for faster authoring
            handPoseAction.getDefinition().setSide(sideOfNewAction);
            handPoseAction.getDefinition().setPalmParentFrameName(findConvenientParentFrameName(handPoseAction, sideOfNewAction));
            handPoseAction.getState().update();

            RDXHandPoseAction nextPreviousHandPoseAction = findNextPreviousAction(RDXHandPoseAction.class, sideOfNewAction);
            if (nextPreviousHandPoseAction != null && nextPreviousHandPoseAction.getState().getPalmFrame().isChildOfWorld())
            {
               nextPreviousHandPoseAction.getState().getPalmFrame()
                        .getReferenceFrame().getTransformToDesiredFrame(handPoseAction.getDefinition().getPalmTransformToParent(),
                                                                        handPoseAction.getState().getPalmFrame().getReferenceFrame().getParent());
            }
            else // set to current robot's hand pose
            {
               syncedRobot.getReferenceFrames().getHandFrame(sideOfNewAction)
                          .getTransformToDesiredFrame(handPoseAction.getDefinition().getPalmTransformToParent(),
                                                      handPoseAction.getState().getPalmFrame().getReferenceFrame().getParent());
            }
            handPoseAction.getState().getPalmFrame().getReferenceFrame().update();
         }
         else if (newAction instanceof RDXHandWrenchAction handWrenchAction)
         {
            handWrenchAction.getDefinition().setSide(sideOfNewAction);
         }
         else if (newAction instanceof RDXChestOrientationAction chestOrientationAction)
         {
            // Set the new action to where the last one was for faster authoring
            RDXChestOrientationAction nextPreviousChestOrientationAction = findNextPreviousAction(RDXChestOrientationAction.class);
            if (nextPreviousChestOrientationAction != null)
            {
               chestOrientationAction.setIncludingFrame(nextPreviousChestOrientationAction.getReferenceFrame().getParent(),
                                                        nextPreviousChestOrientationAction.getReferenceFrame().getTransformToParent());
            }
            else // set to current robot's chest pose
            {
               chestOrientationAction.setToReferenceFrame(syncedRobot.getReferenceFrames().getChestFrame());
            }
            chestOrientationAction.getDefinition().getConditionalReferenceFrame().setParentFrameName(syncedRobot.getReferenceFrames().getPelvisZUpFrame().getName());
         }
         else if (newAction instanceof RDXPelvisHeightPitchAction pelvisHeightPitchAction)
         {
            // Set the new action to where the last one was for faster authoring
            RDXPelvisHeightPitchAction nextPreviousPelvisHeightAction = findNextPreviousAction(RDXPelvisHeightPitchAction.class);
            if (nextPreviousPelvisHeightAction != null)
            {
               pelvisHeightPitchAction.setIncludingFrame(nextPreviousPelvisHeightAction.getReferenceFrame().getParent(),
                                                         nextPreviousPelvisHeightAction.getReferenceFrame().getTransformToParent());
            }
            else // set to current robot's pelvis pose
            {
               pelvisHeightPitchAction.setToReferenceFrame(syncedRobot.getReferenceFrames().getPelvisFrame());
            }
            pelvisHeightPitchAction.getDefinition().getConditionalReferenceFrame().setParentFrameName(ReferenceFrame.getWorldFrame().getName());
         }
         else if (newAction instanceof RDXFootstepPlanAction footstepPlanAction)
         {
            if (nextPreviousParentFrame != null)
               footstepPlanAction.getDefinition().getConditionalReferenceFrame().setParentFrameName(nextPreviousParentFrame.getName());
         }

         insertNewAction(newAction);
      }

      for (int actionIndex = 0; actionIndex < actionSequence.size(); actionIndex++)
      {
         RDXBehaviorAction action = actionSequence.get(actionIndex);
         action.setActionIndex(actionIndex);
         action.setActionNextExecutionIndex(executionNextIndexStatus);
         boolean executeWithPreviousAction = false;
         if (actionIndex > 0)
            executeWithPreviousAction = actionSequence.get(actionIndex - 1).getDefinition().getExecuteWithNextAction();

         boolean firstConcurrentActionIsNextForExecution = (actionSequence.get(actionIndex).getDefinition().getExecuteWithNextAction()
                                                            && actionIndex == executionNextIndexStatus);
         boolean otherConcurrentActionIsNextForExecution
               = executeWithPreviousAction
               && actionIndex == (executionNextIndexStatus + getIndexShiftFromConcurrentActionRoot(actionIndex, executionNextIndexStatus, true));
         boolean concurrentActionIsNextForExecution = firstConcurrentActionIsNextForExecution || otherConcurrentActionIsNextForExecution;
         action.update(concurrentActionIsNextForExecution);
      }
   }

   private int getIndexShiftFromConcurrentActionRoot(int actionIndex, int executionNextIndex, boolean executeWithPreviousAction)
   {
      if (executeWithPreviousAction)
      {
         boolean isNotRootOfConcurrency = true;
         for (int j = 1; j <= actionIndex; j++)
         {
            boolean thisPreviousActionIsConcurrent = actionSequence.get(actionIndex - j).getDefinition().getExecuteWithNextAction();
            isNotRootOfConcurrency = thisPreviousActionIsConcurrent && executionNextIndex != (actionIndex - j + 1);
            if (!isNotRootOfConcurrency)
            {
               return (j - 1);
            }
            else if ((actionIndex - j) == 0 && thisPreviousActionIsConcurrent)
            {
               return j;
            }
         }
      }
      return -1;
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
            for (RDXBehaviorAction action : actionSequence)
               action.updateBeforeRemoving();
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
      ImGui.getItemRectSize(expandButtonSize);
      ImGui.sameLine();
      if (ImGui.button(labels.get("[-]"), expandButtonSize.x, expandButtonSize.y))
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
         BehaviorActionSequenceTools.packActionSequenceUpdateMessage(actionSequence, actionSequenceUpdateMessage);
         outOfSync = !sequenceStatusSubscription.getMessageNotification().read().equals(actionSequenceUpdateMessage);

         if (outOfSync)
         {
            // Automatically attempt to get back in sync
            BehaviorActionSequenceTools.packActionSequenceUpdateMessage(actionSequence, actionSequenceUpdateMessage);
            ros2ControllerHelper.publish(BehaviorActionSequence.SEQUENCE_COMMAND_TOPIC, actionSequenceUpdateMessage);
         }
      }

      boolean endOfSequence = executionNextIndexStatus >= actionSequence.size();
      if (!endOfSequence && !outOfSync)
      {
         String nextActionRejectionTooltip = executionNextIndexRejectionSubscription.getLatest().getDataAsString();
         boolean canExecuteNextAction = nextActionRejectionTooltip.isEmpty();

         ImGui.sameLine();
         ImGui.text("Execute");
         ImGui.sameLine();

         automaticExecution.set(automaticExecutionStatusSubscription.getLatest().getData());

         if (!canExecuteNextAction)
            ImGui.beginDisabled();
         if (ImGui.checkbox(labels.get("Autonomously"), automaticExecution))
         {
            automaticExecutionCommandMessage.setData(automaticExecution.get());
            ros2ControllerHelper.publish(BehaviorActionSequence.AUTOMATIC_EXECUTION_COMMAND_TOPIC, automaticExecutionCommandMessage);
         }
         if (!canExecuteNextAction)
            ImGui.endDisabled();

         ImGuiTools.previousWidgetTooltip("Enables autonomous execution. Will immediately start executing when checked.");
         if (!automaticExecution.get()
             && (lastManualExecutionActionIndex != executionNextIndexStatus)) // Prevent spamming the Manual button and sending the current action more than once
         {
            ImGui.sameLine();

            // Ensure we don't get the confirmation if it's safe to execute the action
            if (canExecuteNextAction)
            {
               if (ImGui.button(labels.get("Manually")))
               {
                  ros2ControllerHelper.publish(BehaviorActionSequence.MANUALLY_EXECUTE_NEXT_ACTION_TOPIC, manuallyExecuteNextActionMessage);
                  lastManualExecutionActionIndex = executionNextIndexStatus;
               }
            }
            else
            {
               if (System.currentTimeMillis() - lastManualExecutionConfirmTime < 5000)
               {
                  ImGui.pushStyleColor(ImGuiCol.Button, Color.RED.toIntBits());
                  if (ImGui.button(labels.get("Manually (confirm)")))
                  {
                     ros2ControllerHelper.publish(BehaviorActionSequence.MANUALLY_EXECUTE_NEXT_ACTION_TOPIC, manuallyExecuteNextActionMessage);
                     lastManualExecutionActionIndex = executionNextIndexStatus;
                     lastManualExecutionConfirmTime = 0;
                  }
                  ImGui.popStyleColor();
               }
               else
               {
                  ImGui.pushStyleColor(ImGuiCol.Button, Color.RED.toIntBits());
                  if (ImGui.button(labels.get("Manually")))
                  {
                     lastManualExecutionConfirmTime = System.currentTimeMillis();
                  }
                  ImGui.popStyleColor();
               }
            }

            ImGuiTools.previousWidgetTooltip("Executes the next action.");

            if (!executionNextIndexRejectionSubscription.getLatest().getDataAsString().isEmpty())
            {
               executionRejectionTooltipText.renderText(executionNextIndexRejectionSubscription.getLatest().getDataAsString(), true);
            }
         }
      }

      ImGui.text(String.format("Sequence update # %d, Status # %d:", receivedSequenceStatusMessageCount, receivedStatusMessageCount));
      if (outOfSync)
      {
         ImGui.sameLine();
         ImGui.textColored(ImGuiTools.RED, String.format("Out of sync! # Actions: Local: %d Remote: %d", actionSequence.size(), remoteSequenceSize));
      }
      else
      {
         ImGui.sameLine();
         ImGui.textColored(ImGuiTools.DARK_GREEN, "Synchronized.");
      }

      {  // These brackets here to take `latestExecutionStatus` out of scope below.
         // We use executionStatusMessageToDisplay in order to display the previously
         // executed action's results, otherwise it gets cleared.
         IDLSequence.Object<ActionExecutionStatusMessage> latestActionsExecutionStatus = executionStatusSubscription.getLatest().getActionStatusList();
         ActionExecutionStatusMessage last = latestActionsExecutionStatus.getLast();
         if (last == null || last.getActionIndex() < 0)
         {
            if (endOfSequence)
            {
               ImGui.text("End of sequence.");
            }
            else
            {
               ImGui.text("Nothing executing.");
            }
         }
         else
         {
            executionStatusMessagesToDisplay.clear();
            currentlyExecutingActions.clear();
            for (int i = 0; i < latestActionsExecutionStatus.size(); i++)
            {
               executionStatusMessagesToDisplay.add(latestActionsExecutionStatus.get(i));
               currentlyExecutingActions.add(actionSequence.get(executionStatusMessagesToDisplay.get(i).getActionIndex()));
            }
         }
      }

      multipleActionProgressBars.getActionProgressBars().clear();
      for (int i = 0; i < executionStatusMessagesToDisplay.size(); i++)
      {
         RDXSingleActionProgressBars actionProgressBars = multipleActionProgressBars.getActionProgressBars().add();
         actionProgressBars.setAction(currentlyExecutingActions.get(i));
         actionProgressBars.setActionExecutionStatusMessage(executionStatusMessagesToDisplay.get(i));
      }

      multipleActionProgressBars.render();
   }

   private void renderInteractableActionListArea()
   {
      reorderRequest.setLeft(-1);

      longestDescriptionLength = 50.0f;
      for (int i = 0; i < actionSequence.size(); i++)
      {
         ImGui.calcTextSize(calcDescriptionTextSize, actionSequence.get(i).getDefinition().getDescription());
         if (calcDescriptionTextSize.x > longestDescriptionLength)
            longestDescriptionLength = calcDescriptionTextSize.x;
      }

      for (int i = 0; i < actionSequence.size(); i++)
      {
         RDXBehaviorAction action = actionSequence.get(i);

         if (ImGui.radioButton(labels.get("", "playbackNextIndex", i), executionNextIndexStatus == i))
         {
            commandNextActionIndex(i);
         }
         ImGuiTools.previousWidgetTooltip("Next for execution. Index " + i);
         action.getImDescription().set(action.getDefinition().getDescription());
         ImGui.sameLine();
         ImGui.text("->");

         ImGui.sameLine();
         if (!action.getExpanded().get())
         {
            if (ImGui.button(labels.get("[+]", "expand", i)))
               action.getExpanded().set(true);
            ImGuiTools.previousWidgetTooltip("Expand action settings");
         }
         else
         {
            if (ImGui.button(labels.get("[-]", "collapse", i), expandButtonSize.x, expandButtonSize.y))
               action.getExpanded().set(false);
            ImGuiTools.previousWidgetTooltip("Collapse action settings");
         }

         ImGui.sameLine();
         ImGui.pushItemWidth(longestDescriptionLength + 30.0f);
         ImGuiTools.inputText(labels.get("", "description", i), action.getImDescription());
         action.getDefinition().setDescription(action.getImDescription().get());
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
            actionSequence.get(i).updateBeforeRemoving();
            RDXBehaviorAction removedAction = actionSequence.remove(i);
            commandNextActionIndex(actionSequence.size());
         }

         if (action.getExpanded().get())
         {
            action.getSelected().renderImGuiWidget();
            ImGuiTools.previousWidgetTooltip("(Show gizmo)");
            ImGui.sameLine();
            ImGui.text("Type: %s   Index: %d".formatted(action.getActionTypeTitle(), i));
         }

         action.renderImGuiWidgets();

         ImGui.separator();
      }

      int indexToMove = reorderRequest.getLeft();
      if (indexToMove > -1)
      {
         int destinationIndex = reorderRequest.getRight() == 0 ? indexToMove - 1 : indexToMove + 1;
         boolean toMoveWasSelected = indexToMove == executionNextIndexStatus;
         boolean destinationWasSelected = destinationIndex == executionNextIndexStatus;
         actionSequence.add(destinationIndex, actionSequence.remove(indexToMove));
         if (toMoveWasSelected) // Retain next action for execution
            commandNextActionIndex(destinationIndex);
         if (destinationWasSelected)
            commandNextActionIndex(indexToMove);
      }
   }

   protected void renderActionCreationArea()
   {
      if (workspaceFile == null)
      {
         ImGui.pushItemFlag(ImGuiItemFlags.Disabled, true);
         ImGui.pushStyleVar(ImGuiStyleVar.Alpha, ImGui.getStyle().getAlpha() * 0.5f);
      }

      if (ImGui.button(labels.get("Add Walk")))
      {
         actionToCreate.set(RDXWalkAction.class);
      }
      ImGui.text("Add Hand Pose:");
      ImGui.sameLine();
      for (var side : RobotSide.values)
      {
         if (ImGui.button(labels.get(side.getPascalCaseName(), "HandPose")))
         {
            actionToCreate.set(RDXHandPoseAction.class);
            sideOfNewAction = side;
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
            actionToCreate.set(RDXHandWrenchAction.class);
            sideOfNewAction = side;
         }
         if (side.ordinal() < 1)
            ImGui.sameLine();
      }
      if (ImGui.button(labels.get("Add Hand Configuration")))
      {
         actionToCreate.set(RDXSakeHandCommandAction.class);
      }
      if (ImGui.button(labels.get("Add Chest Orientation")))
      {
         actionToCreate.set(RDXChestOrientationAction.class);
      }
      if (ImGui.button(labels.get("Add Pelvis Height and Pitch")))
      {
         actionToCreate.set(RDXPelvisHeightPitchAction.class);
      }
      if (ImGui.button(labels.get("Add Arm Joint Angles")))
      {
         actionToCreate.set(RDXArmJointAnglesAction.class);
      }
      if (ImGui.button(labels.get("Add Footstep Plan")))
      {
         actionToCreate.set(RDXFootstepPlanAction.class);
      }
      if (ImGui.button(labels.get("Add Wait")))
      {
         actionToCreate.set(RDXWaitDurationAction.class);
      }

      if (workspaceFile == null)
      {
         ImGui.popStyleVar();
         ImGui.popItemFlag();
      }
   }

   /**
    * @return This used to find the most likely desired frame that a new action will
    *         be specified in, by traversing the sequence backwards and finding the
    *         first action that is specified in a frame, and returning that frame.
    *         This helps the authoring process by initializing new actions with
    *         spatially consistent values.
    */
   private String findConvenientParentFrameName(RDXBehaviorAction action, @Nullable RobotSide side)
   {
      RDXBehaviorAction nextPreviousAction = findNextPreviousAction(action.getClass(), side);

      if (nextPreviousAction instanceof RDXFootstepPlanAction footstepPlanAction)
      {
         return footstepPlanAction.getDefinition().getParentFrameName();
      }
      else if (nextPreviousAction instanceof RDXHandPoseAction handPoseAction)
      {
         return handPoseAction.getDefinition().getPalmParentFrameName();
      }
      else if (nextPreviousAction instanceof RDXWalkAction walkAction)
      {
         return walkAction.getDefinition().getParentFrameName();
      }

      return ReferenceFrame.getWorldFrame().getName();
   }

   private <T extends RDXBehaviorAction> T findNextPreviousAction(Class<T> actionClass, @Nullable RobotSide side)
   {
      T previousAction = null;
      for (int i = Math.min(executionNextIndexStatus, actionSequence.size() - 1); i >= 0; i--)
      {
         RDXBehaviorAction action = actionSequence.get(i);
         if (actionClass.isInstance(action))
         {
            boolean match = side == null;
            match |= action.getDefinition() instanceof SidedObject sidedAction && sidedAction.getSide() == side;

            if (match)
            {
               previousAction = actionClass.cast(action);
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
      if (automaticExecutionStatusSubscription != null)
      {
         automaticExecutionStatusSubscription.destroy();
         executionNextIndexStatusSubscription.destroy();
         sequenceStatusSubscription.destroy();
      }
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
