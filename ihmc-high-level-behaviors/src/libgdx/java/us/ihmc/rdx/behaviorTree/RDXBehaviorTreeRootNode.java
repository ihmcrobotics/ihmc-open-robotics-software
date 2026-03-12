package us.ihmc.rdx.behaviorTree;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import gnu.trove.map.hash.TLongObjectHashMap;
import imgui.ImGui;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeDefinition;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeTools;
import us.ihmc.behaviors.behaviorTree.condition.BehaviorTreeLLMEncoding;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.behaviorTree.scene.RDXBehaviorTreeScene;
import us.ihmc.rdx.imgui.ImBooleanWrapper;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.behaviorTree.actions.RDXActionNode;
import us.ihmc.rdx.behaviorTree.actions.RDXActionProgressWidgetsManager;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXMultiBodyGraphic;
import us.ihmc.rdx.ui.widgets.ImGuiRootIconWidget;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import java.util.ArrayList;
import java.util.List;

public class RDXBehaviorTreeRootNode extends RDXBehaviorTreeNode<BehaviorTreeRootNodeState, BehaviorTreeRootNodeDefinition>
   implements BehaviorTreeRootNode<RDXBehaviorTreeNode<?, ?>>
{
   private final RDXBehaviorTree tree;
   private ROS2Node previewROS2Node;
   private ROS2SyncedRobotModel realSyncedRobot;
   private ROS2SyncedRobotModel previewSyncedRobot;
   private FullHumanoidRobotModel previewRobotModel;
   private RDXMultiBodyGraphic previewRobot;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBooleanWrapper automaticExecutionCheckbox;
   private final ImBooleanWrapper concurrencyEnabledCheckbox;
   private final ImBooleanWrapper previewModeCheckbox;
   private final TLongObjectHashMap<RDXBehaviorTreeNode<?, ?>> idToNodeMap = new TLongObjectHashMap<>();
   private final List<RDXLeafNode<?, ?>> orderedLeaves = new ArrayList<>();
   private final List<RDXActionNode<?, ?>> orderedActions = new ArrayList<>();
   private final List<RDXLeafNode<?, ?>> nextForExecutionLeaves = new ArrayList<>();
   private final List<RDXLeafNode<?, ?>> currentlyExecutingLeaves = new ArrayList<>();
   private RDXBehaviorTreeNode<?, ?> selectedNode = null;
   private final RDXActionProgressWidgetsManager progressWidgetsManager = new RDXActionProgressWidgetsManager();
   private final ImGuiRootIconWidget rootIconWidget = new ImGuiRootIconWidget();

   public RDXBehaviorTreeRootNode(long id,
                                  RDXBehaviorTree tree,
                                  WorkspaceResourceDirectory saveFileDirectory,
                                  ROS2SyncedRobotModel syncedRobot,
                                  RDXBehaviorTreeScene scene,
                                  RobotCollisionModel selectionCollisionModel,
                                  RDXBaseUI baseUI,
                                  RDX3DPanel panel3D)
   {
      super(new BehaviorTreeRootNodeState(id, tree.getCRDTInfo(), saveFileDirectory, syncedRobot.getRobotModel(), scene),
            syncedRobot,
            scene,
            selectionCollisionModel,
            baseUI,
            panel3D);

      this.tree = tree;
      this.realSyncedRobot = syncedRobot;

      automaticExecutionCheckbox = new ImBooleanWrapper(state::getAutomaticExecution,
                                                        state::setAutomaticExecution,
                                                        imBoolean -> ImGui.checkbox(labels.get("Autonomously"), imBoolean));
      concurrencyEnabledCheckbox = new ImBooleanWrapper(state::getConcurrencyEnabled,
                                                        state::setConcurrencyEnabled,
                                                        imBoolean -> ImGui.checkbox(labels.get("Concurrency Enabled"), imBoolean));
      previewModeCheckbox = new ImBooleanWrapper(state::getPreviewModeEnabled,
                                                 state::setPreviewModeEnabled,
                                                 imBoolean -> ImGui.checkbox(labels.get("Preview Mode"), imBoolean));
      previewModeCheckbox.set(false);
   }

   @Override
   public void update()
   {
      super.update();

      if (state.getPreviewModeEnabled())
      {
         if (previewRobot == null)
         {
            ROS2NodeBuilder ros2NodeBuilder = new ROS2NodeBuilder().domainId(165); // TODO: Decide what domain is better
            previewROS2Node = ros2NodeBuilder.build("behavior_preview");
            previewSyncedRobot = new ROS2SyncedRobotModel(rootNode.robotModel, previewROS2Node);
            previewRobotModel = robotModel.createFullRobotModel();
            previewRobot = new RDXMultiBodyGraphic(robotModel.getSimpleRobotName() + " (Behavior Preview)");
            RobotDefinition previewRobotDefinition = new RobotDefinition(robotModel.getRobotDefinition());
            ColorDefinition diffuseColor = ColorDefinitions.GreenYellow();
            diffuseColor.setAlpha(0.5);
            MaterialDefinition material = new MaterialDefinition(diffuseColor);
            RobotDefinition.forEachRigidBodyDefinition(previewRobotDefinition.getRootBodyDefinition(),
                                                       body -> body.getVisualDefinitions().forEach(visual -> visual.setMaterialDefinition(material)));
            previewRobot.loadRobotModelAndGraphics(previewRobotDefinition, previewSyncedRobot.getFullRobotModel().getElevator());
            previewRobot.setActive(true);
            previewRobot.create();
         }

         previewSyncedRobot.update();
         previewRobot.update();
      }

      scene.setSyncedRobot(state.getPreviewModeEnabled() ? previewSyncedRobot : realSyncedRobot);
      scene.update();

      BehaviorTreeTools.runForSubtreeNodes(this, node -> node.syncedRobot = state.getPreviewModeEnabled() ? previewSyncedRobot : realSyncedRobot);

      idToNodeMap.clear();
      orderedLeaves.clear();
      orderedActions.clear();
      nextForExecutionLeaves.clear();
      currentlyExecutingLeaves.clear();
      selectedNode = null;
      updateNodeListsRecursive(this);

      for (RDXLeafNode<?, ?> leaf : orderedLeaves)
         leaf.getState().validateDefinition(state.getOrderedNodes());
   }

   public void updateNodeListsRecursive(RDXBehaviorTreeNode<?, ?> node)
   {
      idToNodeMap.put(node.getState().getID(), node);
      if (node.getSelected())
         selectedNode = node;

      for (RDXBehaviorTreeNode<?, ?> child : node.getChildren())
      {
         if (child instanceof RDXLeafNode<?, ?> leaf)
         {
            orderedLeaves.add(leaf);

            if (leaf.getState().getIsNextForExecution())
               nextForExecutionLeaves.add(leaf);
            if (leaf.getState().getIsExecuting())
               currentlyExecutingLeaves.add(leaf);
            if (child instanceof RDXActionNode<?, ?> action)
               orderedActions.add(action);
         }

         updateNodeListsRecursive(child);
      }
   }

   @Override
   public void renderTreeViewRow()
   {
      super.renderRowBeginning();

      rootIconWidget.render();

      ImGui.sameLine();
      super.renderEditableName();
   }

   @Override
   public void renderContextMenuItems()
   {
      super.renderContextMenuItems();

      if (ImGui.menuItem(labels.get("Print LLM Encoding")))
         LogTools.info("LLM Encoding:%n%s".formatted(BehaviorTreeLLMEncoding.encode(state)));
   }

   public void renderExecutionControlAndProgressWidgets()
   {
      if (ImGui.button(labels.get("<")))
         state.stepBackNextExecutionIndex();
      ImGuiTools.previousWidgetTooltip("Go to previous leaf");
      ImGui.sameLine();
      ImGui.text("Index: " + String.format("%03d", state.getExecutionNextIndex()));
      ImGui.sameLine();
      if (ImGui.button(labels.get(">")))
         state.stepForwardNextExecutionIndex();
      ImGuiTools.previousWidgetTooltip("Go to next leaf");

      ImGui.sameLine();
      ImGui.text("Execute");
      ImGui.sameLine();

      automaticExecutionCheckbox.renderImGuiWidget();
      if (automaticExecutionCheckbox.changed())
         definition.modify();

      ImGuiTools.previousWidgetTooltip("Enables autonomous execution. Will immediately start executing when checked.");

      boolean endOfSequence = state.getExecutionNextIndex() >= state.getOrderedLeaves().size();
      ImGui.beginDisabled(endOfSequence || state.getAutomaticExecution());
      {
         ImGui.sameLine();

         boolean disableManuallyExecuteButton = state.getManualExecutionRequested();
         ImGui.beginDisabled(disableManuallyExecuteButton);
         {
            if (ImGui.button(labels.get("Manually")))
            {
               state.setManualExecutionRequested();
            }
         }
         ImGui.endDisabled();
         ImGuiTools.previousWidgetTooltip("Executes the next leaf.");
      }
      ImGui.endDisabled();
      if (ImGui.button(labels.get("Reset Failures")))
      {
         state.setFailureResetRequested();
      }

      ImGui.sameLine();
      concurrencyEnabledCheckbox.renderImGuiWidget();
      ImGui.sameLine();
      previewModeCheckbox.renderImGuiWidget();

      if (currentlyExecutingLeaves.isEmpty())
      {
         ImGui.text("Nothing executing.");
      }
      else
      {
         ImGui.text("Executing:");
         for (RDXLeafNode<?, ?> currentlyExecutingLeaf : currentlyExecutingLeaves)
         {
            ImGui.sameLine();
            ImGui.text("%s (%s)".formatted(currentlyExecutingLeaf.getDefinition().getName(),
                                           currentlyExecutingLeaf.getLeafTypeTitle()));
         }
      }

      progressWidgetsManager.getActionNodesToRender().clear();
      int lastIndex = 0;
      for (RDXLeafNode<?, ?> currentlyExecutingLeaf : currentlyExecutingLeaves)
         if (currentlyExecutingLeaf instanceof RDXActionNode<?, ?> currentlyExecutingAction)
         {
            progressWidgetsManager.getActionNodesToRender().add(currentlyExecutingAction);
            lastIndex = Math.max(lastIndex, currentlyExecutingAction.getState().getLeafIndex());
         }
      if (currentlyExecutingLeaves.isEmpty() && selectedNode instanceof RDXActionNode<?, ?> selectedAction)
         progressWidgetsManager.getActionNodesToRender().add(selectedAction);
      progressWidgetsManager.render();
   }

   @Override
   public void renderNodeSettingsWidgets()
   {
      ImGui.text("Type: %s   ID: %d".formatted(definition.getClass().getSimpleName(), state.getID()));

      super.renderNodeSettingsWidgets();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (previewRobot != null)
      {
         previewRobot.setActive(state.getPreviewModeEnabled());
         previewRobot.getRenderables(renderables, pool, baseUI.getPrimaryScene().getSceneLevelsToRender());
      }
   }

   @Override
   public void destroy()
   {
      super.destroy();
      scene.destroy();

      if (previewROS2Node != null)
         previewROS2Node.destroy();
      if (previewSyncedRobot != null)
         previewSyncedRobot.destroy();
      if (previewRobot != null)
         previewRobot.destroy();
   }

   public TLongObjectHashMap<RDXBehaviorTreeNode<?, ?>> getIDToNodeMap()
   {
      return idToNodeMap;
   }

   public RDXActionProgressWidgetsManager getProgressWidgetsManager()
   {
      return progressWidgetsManager;
   }

   // Getters are in here so there's not getters in base node for root stuff

   public RDXBehaviorTree getTree()
   {
      return tree;
   }

   public ROS2SyncedRobotModel getSyncedRobot()
   {
      return syncedRobot;
   }

   public RDXBehaviorTreeScene getScene()
   {
      return scene;
   }

   public RobotCollisionModel getSelectionCollisionModel()
   {
      return selectionCollisionModel;
   }

   public RDXBaseUI getBaseUI()
   {
      return baseUI;
   }

   public RDX3DPanel get3DPanel()
   {
      return panel3D;
   }
}