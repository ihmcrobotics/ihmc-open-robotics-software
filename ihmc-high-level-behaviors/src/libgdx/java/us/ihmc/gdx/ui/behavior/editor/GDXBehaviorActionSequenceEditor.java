package us.ihmc.gdx.ui.behavior.editor;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.ImGui;
import imgui.type.ImBoolean;
import org.apache.commons.lang3.tuple.MutablePair;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.commons.FormattingTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.gdx.GDXFocusBasedCamera;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.io.WorkspaceDirectory;
import us.ihmc.tools.io.WorkspaceFile;

import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

public class GDXBehaviorActionSequenceEditor
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private ImGuiPanel panel;
   private final ImBoolean enabled = new ImBoolean(false);
   private String name;
   private final WorkspaceFile workspaceFile;
   private final LinkedList<GDXBehaviorAction> actionSequence = new LinkedList<>();
   private String pascalCasedName;
   private GDXFocusBasedCamera camera3D;
   private DRCRobotModel robotModel;
   private int playbackNextIndex = 0;
   private FootstepPlanningModule footstepPlanner;
   private ROS2SyncedRobotModel syncedRobot;
   private List<ReferenceFrame> referenceFrameLibrary;
   private ROS2ControllerHelper ros2ControllerHelper;
   private final MutablePair<Integer, Integer> reorderRequest = MutablePair.of(-1, 0);

   public GDXBehaviorActionSequenceEditor(WorkspaceFile fileToLoadFrom)
   {
      this.workspaceFile = fileToLoadFrom;
      loadNameFromFile();
      afterNameDetermination();
   }

   public GDXBehaviorActionSequenceEditor(String name, WorkspaceDirectory storageDirectory)
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

   public void create(GDXFocusBasedCamera camera3D,
                      DRCRobotModel robotModel,
                      ROS2Node ros2Node,
                      ROS2SyncedRobotModel syncedRobot,
                      List<ReferenceFrame> referenceFrameLibrary)
   {
      this.camera3D = camera3D;
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
      playbackNextIndex = 0;
      LogTools.info("Loading from {}", workspaceFile.getClasspathResource().toString());
      JSONFileTools.load(workspaceFile, jsonNode ->
      {
         for (Iterator<JsonNode> actionNodeIterator = jsonNode.withArray("actions").elements(); actionNodeIterator.hasNext(); )
         {
            JsonNode actionNode = actionNodeIterator.next();
            String actionType = actionNode.get("type").asText();
            if (actionType.equals(GDXWalkAction.class.getSimpleName()))
            {
               GDXWalkAction walkAction = addWalkAction();
               walkAction.loadFromFile(actionNode);
            }
            else if (actionType.equals(GDXHandPoseAction.class.getSimpleName()))
            {
               GDXHandPoseAction handPoseAction = addHandPoseAction();
               handPoseAction.loadFromFile(actionNode);
            }
            else if (actionType.equals(GDXHandWrenchAction.class.getSimpleName()))
            {
               GDXHandWrenchAction handWrenchAction = addHandWrenchAction();
               handWrenchAction.loadFromFile(actionNode);
            }
            else if (actionType.equals(GDXHandConfigurationAction.class.getSimpleName()))
            {
               GDXHandConfigurationAction action = addHandConfigurationAction();
               action.loadFromFile(actionNode);
            }
            else if (actionType.equals(GDXChestOrientationAction.class.getSimpleName()))
            {
               GDXChestOrientationAction action = addChestOrientationAction();
               action.loadFromFile(actionNode);
            }
            else if (actionType.equals(GDXPelvisHeightAction.class.getSimpleName()))
            {
               GDXPelvisHeightAction action = addPelvisHeightAction();
               action.loadFromFile(actionNode);
            }
            else if (actionType.equals(GDXArmJointAnglesAction.class.getSimpleName()))
            {
               GDXArmJointAnglesAction action = addArmJointAnglesAction();
               action.loadFromFile(actionNode);
            }
            else if (actionType.equals(GDXFootstepAction.class.getSimpleName()))
            {
               GDXFootstepAction action = addFootstepAction();
               action.loadFromFile(actionNode);
            }
         }
      });
      playbackNextIndex = 0;
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
            for (GDXBehaviorAction behaviorAction : actionSequence)
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
      for (GDXBehaviorAction action : actionSequence)
      {
         action.calculate3DViewPick(input);
      }
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      for (GDXBehaviorAction action : actionSequence)
      {
         action.process3DViewInput(input);
      }
   }

   public void update()
   {
      for (GDXBehaviorAction action : actionSequence)
      {
         action.update();
      }
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

      ImGui.endMenuBar();

      if (ImGui.button(labels.get("<")))
      {
         if (playbackNextIndex > 0)
            playbackNextIndex--;
      }
      ImGui.sameLine();
      ImGui.text("Index: " + String.format("%03d", playbackNextIndex));
      ImGui.sameLine();
      if (ImGui.button(labels.get(">")))
      {
         if (playbackNextIndex < actionSequence.size())
            playbackNextIndex++;
      }
      ImGui.sameLine();
      boolean endOfSequence = playbackNextIndex >= actionSequence.size();
      if (!endOfSequence)
      {
         if (ImGui.button(labels.get("Execute")))
         {
            GDXBehaviorAction action = actionSequence.get(playbackNextIndex);
            action.performAction();
            playbackNextIndex++;
         }
      }
      ImGui.sameLine();
      endOfSequence = playbackNextIndex >= actionSequence.size();
      if (endOfSequence)
         ImGui.text("No actions left.");
      else
         ImGui.text(actionSequence.get(playbackNextIndex).getNameForDisplay());

      ImGui.separator();

      ImGui.beginChild(labels.get("childRegion"));


      reorderRequest.setLeft(-1);
      for (int i = 0; i < actionSequence.size(); i++)
      {
         GDXBehaviorAction action = actionSequence.get(i);
         ImGui.checkbox(labels.get("", "Selected", i), action.getSelected());
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
            ImGui.sameLine();
         }
         if (i < actionSequence.size() - 1)
         {
            if (ImGui.button(labels.get("v", i)))
            {
               reorderRequest.setLeft(i);
               reorderRequest.setRight(1);
            }
            ImGui.sameLine();
         }
         if (ImGui.button(labels.get("X", i)))
         {
            GDXBehaviorAction removedAction = actionSequence.remove(i);
            playbackNextIndex = actionSequence.size();
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

      if (ImGui.button(labels.get("Add Walk")))
      {
         addWalkAction();
      }
      ImGui.text("Add Hand Pose:");
      ImGui.sameLine();
      for (RobotSide side : RobotSide.values)
      {
         if (ImGui.button(labels.get(side.getPascalCaseName(), "HandPose")))
         {
            GDXHandPoseAction handPoseAction = addHandPoseAction();
            // Set the new action to where the last one was for faster authoring
            GDXHandPoseAction previousAction = null;
            for (int i = 0; i < playbackNextIndex - 1; i++)
            {
               if (actionSequence.get(i) instanceof GDXHandPoseAction
               && ((GDXHandPoseAction) actionSequence.get(i)).getSide() == side)
               {
                  previousAction = (GDXHandPoseAction) actionSequence.get(i);
               }
            }
            handPoseAction.setSide(side, true, previousAction);
         }
         if (side.ordinal() < 1)
            ImGui.sameLine();
      }
      ImGui.text("Add Hand Wrench:");
      ImGui.sameLine();
      for (RobotSide side : RobotSide.values)
      {
         if (ImGui.button(labels.get(side.getPascalCaseName(), "HandWrench")))
         {
            GDXHandWrenchAction handWrenchAction = addHandWrenchAction();
            handWrenchAction.setSide(side);
         }
         if (side.ordinal() < 1)
            ImGui.sameLine();
      }
      if (ImGui.button(labels.get("Add Hand Configuration")))
      {
         addHandConfigurationAction();
      }
      if (ImGui.button(labels.get("Add Chest Orientation")))
      {
         addChestOrientationAction();
      }
      if (ImGui.button(labels.get("Add Pelvis Height")))
      {
         addPelvisHeightAction();
      }
      if (ImGui.button(labels.get("Add Arm Joint Angles")))
      {
         addArmJointAnglesAction();
      }
      ImGui.text("Add Footstep:");
      ImGui.sameLine();
      for (RobotSide side : RobotSide.values)
      {
         if (ImGui.button(labels.get(side.getPascalCaseName(), 1)))
         {
            GDXFootstepAction footstepAction = addFootstepAction();
            footstepAction.setSide(side, true);
         }
         if (side.ordinal() < 1)
            ImGui.sameLine();
      }

      ImGui.endChild();
   }

   private GDXHandPoseAction addHandPoseAction()
   {
      GDXHandPoseAction handPoseAction = new GDXHandPoseAction();
      handPoseAction.create(camera3D, robotModel, syncedRobot, syncedRobot.getFullRobotModel(), ros2ControllerHelper, referenceFrameLibrary);
      insertNewAction(handPoseAction);
      return handPoseAction;
   }

   private GDXHandWrenchAction addHandWrenchAction()
   {
      GDXHandWrenchAction handWrenchAction = new GDXHandWrenchAction();
      handWrenchAction.create(ros2ControllerHelper);
      insertNewAction(handWrenchAction);
      return handWrenchAction;
   }

   private GDXHandConfigurationAction addHandConfigurationAction()
   {
      GDXHandConfigurationAction handConfigurationAction = new GDXHandConfigurationAction();
      handConfigurationAction.create(ros2ControllerHelper);
      insertNewAction(handConfigurationAction);
      return handConfigurationAction;
   }

   private GDXChestOrientationAction addChestOrientationAction()
   {
      GDXChestOrientationAction chestOrientationAction = new GDXChestOrientationAction();
      chestOrientationAction.create(ros2ControllerHelper, syncedRobot);
      insertNewAction(chestOrientationAction);
      return chestOrientationAction;
   }

   private GDXPelvisHeightAction addPelvisHeightAction()
   {
      GDXPelvisHeightAction pelvisHeightAction = new GDXPelvisHeightAction();
      pelvisHeightAction.create(ros2ControllerHelper);
      insertNewAction(pelvisHeightAction);
      return pelvisHeightAction;
   }

   private GDXArmJointAnglesAction addArmJointAnglesAction()
   {
      GDXArmJointAnglesAction armJointAnglesAction = new GDXArmJointAnglesAction();
      armJointAnglesAction.create(ros2ControllerHelper);
      insertNewAction(armJointAnglesAction);
      return armJointAnglesAction;
   }

   private GDXWalkAction addWalkAction()
   {
      GDXWalkAction walkAction = new GDXWalkAction();
      walkAction.create(camera3D, robotModel, footstepPlanner, syncedRobot, ros2ControllerHelper, referenceFrameLibrary);
      insertNewAction(walkAction);
      return walkAction;
   }

   private GDXFootstepAction addFootstepAction()
   {
      GDXFootstepAction footstepAction = new GDXFootstepAction();

      // Set the new action to where the last one was for faster authoring
      GDXFootstepAction previousAction = null;
      for (int i = 0; i < playbackNextIndex; i++)
      {
         if (actionSequence.get(i) instanceof GDXFootstepAction)
         {
            previousAction = (GDXFootstepAction) actionSequence.get(i);
         }
      }

      footstepAction.create(camera3D, robotModel, syncedRobot, ros2ControllerHelper, referenceFrameLibrary, previousAction);
      insertNewAction(footstepAction);
      return footstepAction;
   }

   private void insertNewAction(GDXBehaviorAction action)
   {
      actionSequence.add(playbackNextIndex, action);
      for (int i = 0; i < actionSequence.size(); i++)
      {
         actionSequence.get(i).getSelected().set(i == playbackNextIndex);
      }
      playbackNextIndex++;
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (panel.getIsShowing().get())
      {
         for (GDXBehaviorAction action : actionSequence)
         {
            action.getRenderables(renderables, pool);
         }
      }
   }

   public ImGuiPanel getPanel()
   {
      return panel;
   }

   public String getName()
   {
      return name;
   }

   public ImBoolean getEnabled()
   {
      return enabled;
   }

   public WorkspaceFile getWorkspaceFile()
   {
      return workspaceFile;
   }
}
