package us.ihmc.rdx.behaviorTree;

import gnu.trove.map.hash.TLongObjectHashMap;
import imgui.ImGui;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeDefinition;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.behaviors.behaviorTree.condition.BehaviorTreeLLMEncoding;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.behaviorTree.scene.RDXBehaviorTreeScene;
import us.ihmc.rdx.imgui.ImBooleanWrapper;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.behaviorTree.actions.RDXActionNode;
import us.ihmc.rdx.behaviorTree.actions.RDXActionProgressWidgetsManager;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.widgets.ImGuiRootIconWidget;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import java.util.ArrayList;
import java.util.List;

public class RDXBehaviorTreeRootNode extends RDXBehaviorTreeNode<BehaviorTreeRootNodeState, BehaviorTreeRootNodeDefinition>
   implements BehaviorTreeRootNode<RDXBehaviorTreeNode<?, ?>>
{
   private final RDXBehaviorTree tree;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBooleanWrapper automaticExecutionCheckbox;
   private final ImBooleanWrapper concurrencyEnabledCheckbox;
   private final TLongObjectHashMap<RDXBehaviorTreeNode<?, ?>> idToNodeMap = new TLongObjectHashMap<>();
   private final List<RDXLeafNode<?, ?>> orderedLeaves = new ArrayList<>();
   private final List<RDXActionNode<?, ?>> orderedActions = new ArrayList<>();
   private final List<RDXLeafNode<?, ?>> nextForExecutionLeaves = new ArrayList<>();
   private final List<RDXLeafNode<?, ?>> currentlyExecutingLeaves = new ArrayList<>();
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

      automaticExecutionCheckbox = new ImBooleanWrapper(state::getAutomaticExecution,
                                                        state::setAutomaticExecution,
                                                        imBoolean -> ImGui.checkbox(labels.get("Autonomously"), imBoolean));
      concurrencyEnabledCheckbox = new ImBooleanWrapper(state::getConcurrencyEnabled,
                                                        state::setConcurrencyEnabled,
                                                        imBoolean -> ImGui.checkbox(labels.get("Concurrency Enabled"), imBoolean));
   }

   @Override
   public void update()
   {
      super.update();

      idToNodeMap.clear();
      orderedLeaves.clear();
      orderedActions.clear();
      nextForExecutionLeaves.clear();
      currentlyExecutingLeaves.clear();
      updateNodeListsRecursive(this);

      for (RDXLeafNode<?, ?> leaf : orderedLeaves)
         leaf.getState().validateDefinition(state.getOrderedLeaves());
   }

   public void updateNodeListsRecursive(RDXBehaviorTreeNode<?, ?> node)
   {
      idToNodeMap.put(node.getState().getID(), node);

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
      {
         if (currentlyExecutingLeaf instanceof RDXActionNode<?, ?> currentlyExecutingAction)
         {
            progressWidgetsManager.getActionNodesToRender().add(currentlyExecutingAction);
            lastIndex = Math.max(lastIndex, currentlyExecutingAction.getState().getLeafIndex());
         }
      }
      for (RDXLeafNode<?, ?> nextForExecutionLeaf : nextForExecutionLeaves)
      {
         if (nextForExecutionLeaf instanceof RDXActionNode<?, ?> nextForExecutionAction)
            if (currentlyExecutingLeaves.isEmpty() || nextForExecutionAction.getState().getLeafIndex() < lastIndex)
               progressWidgetsManager.getActionNodesToRender().add(nextForExecutionAction);
      }
      progressWidgetsManager.render();
   }

   @Override
   public void renderNodeSettingsWidgets()
   {
      ImGui.text("Type: %s   ID: %d".formatted(definition.getClass().getSimpleName(), state.getID()));

      super.renderNodeSettingsWidgets();
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