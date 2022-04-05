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
import us.ihmc.gdx.FocusBasedGDXCamera;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.io.WorkspaceDirectory;
import us.ihmc.tools.io.WorkspaceFile;

import java.util.Iterator;
import java.util.LinkedList;

public class GDXBehaviorActionSequenceEditor
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private ImGuiPanel panel;
   private final ImBoolean enabled = new ImBoolean(false);
   private String name;
   private final WorkspaceFile workspaceFile;
   private final LinkedList<GDXBehaviorAction> actionSequence = new LinkedList<>();
   private String pascalCasedName;
   private FocusBasedGDXCamera camera3D;
   private DRCRobotModel robotModel;
   private int playbackNextIndex = 0;
   private FootstepPlanningModule footstepPlanner;
   private ROS2SyncedRobotModel syncedRobot;
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

   public void create(FocusBasedGDXCamera camera3D, DRCRobotModel robotModel, ROS2Node ros2Node, ROS2SyncedRobotModel syncedRobot)
   {
      this.camera3D = camera3D;
      this.robotModel = robotModel;
      footstepPlanner = FootstepPlanningModuleLauncher.createModule(robotModel);
      this.syncedRobot = syncedRobot;
      ros2ControllerHelper = new ROS2ControllerHelper(ros2Node, robotModel);
   }

   public void loadNameFromFile()
   {
      JSONFileTools.load(workspaceFile, jsonNode -> name = jsonNode.get("name").asText());
   }

   public void loadActionsFromFile()
   {
      actionSequence.clear();
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
         }
      });
   }

   public void saveToFile()
   {
      if (workspaceFile.isFileAccessAvailable())
      {
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
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      for (GDXBehaviorAction action : actionSequence)
      {
         action.process3DViewInput(input);
      }
   }

   public void renderImGuiWidgets()
   {
      ImGui.beginMenuBar();
      if (ImGui.beginMenu(labels.get("File")))
      {
         if (ImGui.menuItem("Save to JSON"))
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
      if (playbackNextIndex < actionSequence.size())
      {
         ImGui.text("Index: " + playbackNextIndex);
         ImGui.sameLine();
         if (ImGui.button(labels.get("Execute")))
         {
            GDXBehaviorAction action = actionSequence.get(playbackNextIndex);
            if (action instanceof GDXWalkAction)
            {
               ((GDXWalkAction) action).walk(ReferenceFrame.getWorldFrame(), ros2ControllerHelper, syncedRobot);
            }
            else if (action instanceof GDXHandPoseAction)
            {
               ((GDXHandPoseAction) action).moveHand(4.0);
            }
            playbackNextIndex++;
         }
      }
      else
      {
         ImGui.text("No actions left.");
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get(">")))
      {
         if (playbackNextIndex < actionSequence.size())
            playbackNextIndex++;
      }

      ImGui.separator();

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
//            removedAction.destroy();
         }
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
      ImGui.text("Add Hand Pose");
      ImGui.sameLine();
      for (RobotSide side : RobotSide.values)
      {
         if (ImGui.button(labels.get(side.getPascalCaseName())))
         {
            GDXHandPoseAction handPoseAction = addHandPoseAction();
            handPoseAction.setSide(side);
         }
         if (side.ordinal() < 1)
            ImGui.sameLine();
      }
   }

   private GDXHandPoseAction addHandPoseAction()
   {
      GDXHandPoseAction handPoseAction = new GDXHandPoseAction();
      handPoseAction.create(camera3D, robotModel, syncedRobot.getFullRobotModel(), ros2ControllerHelper);
      actionSequence.addLast(handPoseAction);
      return handPoseAction;
   }

   private GDXWalkAction addWalkAction()
   {
      GDXWalkAction walkAction = new GDXWalkAction();
      walkAction.create(camera3D, robotModel, footstepPlanner);
      actionSequence.addLast(walkAction);
      return walkAction;
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (GDXBehaviorAction action : actionSequence)
      {
         action.getRenderables(renderables, pool);
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
