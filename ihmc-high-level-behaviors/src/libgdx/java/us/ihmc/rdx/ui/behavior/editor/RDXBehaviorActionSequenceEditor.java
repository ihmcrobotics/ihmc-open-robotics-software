package us.ihmc.rdx.ui.behavior.editor;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.ImGui;
import imgui.type.ImBoolean;
import org.apache.commons.lang3.tuple.MutablePair;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.ReferenceFrameLibrary;
import us.ihmc.commons.FormattingTools;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.ui.behavior.editor.actions.*;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.io.JSONTools;
import us.ihmc.tools.io.WorkspaceDirectory;
import us.ihmc.tools.io.WorkspaceFile;

import java.util.LinkedList;

public class RDXBehaviorActionSequenceEditor
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private ImGuiPanel panel;
   private final ImBoolean automaticExecution = new ImBoolean(false);
   private String name;
   private final WorkspaceFile workspaceFile;
   private final LinkedList<RDXBehaviorAction> actionSequence = new LinkedList<>();
   private String pascalCasedName;
   private RDX3DPanel panel3D;
   private DRCRobotModel robotModel;
   private RDXBehaviorAction currentlyExecutingAction = null;
   private int excecutionNextIndex = 0;
   private FootstepPlanningModule footstepPlanner;
   private ROS2SyncedRobotModel syncedRobot;
   private ReferenceFrameLibrary referenceFrameLibrary;
   private ROS2ControllerHelper ros2ControllerHelper;
   private final MutablePair<Integer, Integer> reorderRequest = MutablePair.of(-1, 0);
   private boolean loading = false;

   public RDXBehaviorActionSequenceEditor(WorkspaceFile fileToLoadFrom)
   {
      this.workspaceFile = fileToLoadFrom;
      loadNameFromFile();
      afterNameDetermination();
   }

   public RDXBehaviorActionSequenceEditor(String name, WorkspaceDirectory storageDirectory)
   {
      this.name = name;
      afterNameDetermination();
      this.workspaceFile = new WorkspaceFile(storageDirectory, pascalCasedName + ".json");
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
      footstepPlanner = FootstepPlanningModuleLauncher.createModule(robotModel);
      this.syncedRobot = syncedRobot;
      this.referenceFrameLibrary = referenceFrameLibrary;
      ros2ControllerHelper = new ROS2ControllerHelper(ros2Node, robotModel);
   }

   public void loadNameFromFile()
   {
      JSONFileTools.load(workspaceFile, jsonNode -> name = jsonNode.get("name").asText());
   }

   public void loadActionsFromFile()
   {
      actionSequence.clear();
      excecutionNextIndex = 0;
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
               case "RDXHandConfigurationAction" -> new RDXHandConfigurationAction(ros2ControllerHelper);
               case "RDXHandPoseAction" -> newHandPoseAction();
               case "RDXHandWrenchAction" -> new RDXHandWrenchAction(ros2ControllerHelper);
               case "RDXPelvisHeightAction" -> new RDXPelvisHeightAction(ros2ControllerHelper);
               case "RDXWaitDurationAction" -> new RDXWaitDurationAction();
               case "RDXWalkAction" -> newWalkAction();
               default -> null;
            };

            action.loadFromFile(actionNode);
            insertNewAction(action);
         });
      });
      loading = false;
      excecutionNextIndex = 0;
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
               behaviorAction.saveToFile(actionNode);
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

      if (ImGui.button(labels.get("<")))
      {
         if (excecutionNextIndex > 0)
            excecutionNextIndex--;
      }
      ImGuiTools.previousWidgetTooltip("Go to previous action");
      ImGui.sameLine();
      ImGui.text("Index: " + String.format("%03d", excecutionNextIndex));
      ImGui.sameLine();
      if (ImGui.button(labels.get(">")))
      {
         if (excecutionNextIndex < actionSequence.size())
            excecutionNextIndex++;
      }
      ImGuiTools.previousWidgetTooltip("Go to next action");

      boolean endOfSequence = excecutionNextIndex >= actionSequence.size();
      if (!endOfSequence)
      {
         ImGui.sameLine();
         ImGui.text("Execute");
         ImGui.sameLine();
         ImGui.checkbox(labels.get("Autonomously"), automaticExecution);
         ImGuiTools.previousWidgetTooltip("Enables autonomous execution. Will immediately start executing when checked.");
         if (!automaticExecution.get())
         {
            ImGui.sameLine();
            if (ImGui.button(labels.get("Manually")))
            {
               // TODO: Send a message
            }
            ImGuiTools.previousWidgetTooltip("Executes the next action.");
         }
      }

      ImGui.text("Current action:");
      ImGui.sameLine();
      endOfSequence = excecutionNextIndex >= actionSequence.size();
      if (endOfSequence)
         ImGui.text("End of sequence.");
      else
         ImGui.text(actionSequence.get(excecutionNextIndex).getNameForDisplay());

      ImGui.separator();

      // This, paired with the endChild call after, allows this area to scroll separately
      // from the rest, so the top controls are still available while editing later parts
      // of the sequence.
      ImGui.beginChild(labels.get("childRegion"));

      reorderRequest.setLeft(-1);
      for (int i = 0; i < actionSequence.size(); i++)
      {
         RDXBehaviorAction action = actionSequence.get(i);
         if (ImGui.radioButton(labels.get("", "playbackNextIndex", i), excecutionNextIndex == i))
         {
            excecutionNextIndex = i;
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
            excecutionNextIndex = actionSequence.size();
//            removedAction.destroy();
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
            RDXHandWrenchAction handWrenchAction = new RDXHandWrenchAction(ros2ControllerHelper);
            handWrenchAction.setSide(side);
            newAction = handWrenchAction;
         }
         if (side.ordinal() < 1)
            ImGui.sameLine();
      }
      if (ImGui.button(labels.get("Add Hand Configuration")))
      {
         newAction = new RDXHandConfigurationAction(ros2ControllerHelper);
      }
      if (ImGui.button(labels.get("Add Chest Orientation")))
      {
         newAction = new RDXChestOrientationAction();
      }
      if (ImGui.button(labels.get("Add Pelvis Height")))
      {
         newAction = new RDXPelvisHeightAction(ros2ControllerHelper);
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

   private RDXFootstepAction findNextPreviousFootstepAction()
   {
      RDXFootstepAction previousAction = null;
      for (int i = 0; i < excecutionNextIndex; i++)
         if (actionSequence.get(i) instanceof RDXFootstepAction)
            previousAction = (RDXFootstepAction) actionSequence.get(i);
      return previousAction;
   }

   private RDXHandPoseAction findNextPreviousHandPoseAction(RobotSide side)
   {
      RDXHandPoseAction previousAction = null;
      for (int i = 0; i < excecutionNextIndex - 1; i++)
      {
         if (actionSequence.get(i) instanceof RDXHandPoseAction
             && ((RDXHandPoseAction) actionSequence.get(i)).getSide() == side)
         {
            previousAction = (RDXHandPoseAction) actionSequence.get(i);
         }
      }
      return previousAction;
   }

   private void insertNewAction(RDXBehaviorAction action)
   {
      actionSequence.add(excecutionNextIndex, action);
      for (int i = 0; i < actionSequence.size(); i++)
      {
         // When loading, we want to deselect all the actions, otherwise the last one ends up being selected.
         actionSequence.get(i).getSelected().set(!loading && i == excecutionNextIndex);
      }
      excecutionNextIndex++;
   }

   private RDXFootstepAction newFootstepAction(RDXFootstepAction possiblyNullPreviousFootstepAction)
   {
      return new RDXFootstepAction(panel3D, robotModel, syncedRobot, referenceFrameLibrary, possiblyNullPreviousFootstepAction);
   }

   private RDXWalkAction newWalkAction()
   {
      return new RDXWalkAction(panel3D, robotModel, footstepPlanner, syncedRobot, ros2ControllerHelper, referenceFrameLibrary);
   }

   private RDXHandPoseAction newHandPoseAction()
   {
      return new RDXHandPoseAction(panel3D, robotModel, syncedRobot, syncedRobot.getFullRobotModel(), ros2ControllerHelper, referenceFrameLibrary);
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

   public WorkspaceFile getWorkspaceFile()
   {
      return workspaceFile;
   }
}
