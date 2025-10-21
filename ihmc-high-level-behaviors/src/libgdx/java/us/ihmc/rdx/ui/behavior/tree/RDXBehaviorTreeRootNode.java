package us.ihmc.rdx.ui.behavior.tree;

import gnu.trove.map.hash.TLongObjectHashMap;
import imgui.ImGui;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeDefinition;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.behaviors.behaviorTree.condition.BehaviorTreeLLMEncoding;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.imgui.ImBooleanWrapper;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.behavior.sequence.RDXActionNode;
import us.ihmc.rdx.ui.behavior.sequence.RDXActionProgressWidgetsManager;
import us.ihmc.rdx.ui.behavior.sequence.RDXLeafNode;
import us.ihmc.rdx.ui.widgets.ImGuiRootIconWidget;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import java.util.ArrayList;
import java.util.List;

public class RDXBehaviorTreeRootNode extends RDXBehaviorTreeNode<BehaviorTreeRootNodeState, BehaviorTreeRootNodeDefinition>
{
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

   public RDXBehaviorTreeRootNode(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(new BehaviorTreeRootNodeState(id, crdtInfo, saveFileDirectory));

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
      updateSubtree(this);

      for (RDXLeafNode<?, ?> leaf : orderedLeaves)
      {
         leaf.getState().validateFields(state.getOrderedLeaves());
      }
   }

   public void updateSubtree(RDXBehaviorTreeNode<?, ?> node)
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

         updateSubtree(child);
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
      {
         state.stepBackNextExecutionIndex();
      }
      ImGuiTools.previousWidgetTooltip("Go to previous leaf");
      ImGui.sameLine();
      ImGui.text("Index: " + String.format("%03d", state.getExecutionNextIndex()));
      ImGui.sameLine();
      if (ImGui.button(labels.get(">")))
      {
         state.stepForwardNextExecutionIndex();
      }
      ImGuiTools.previousWidgetTooltip("Go to next leaf");

      boolean endOfSequence = state.getExecutionNextIndex() >= state.getOrderedLeaves().size();
      ImGui.beginDisabled(endOfSequence); // Use disabled so stuff doesn't glitch around
      {
         ImGui.sameLine();
         ImGui.text("Execute");
         ImGui.sameLine();

         automaticExecutionCheckbox.renderImGuiWidget();
         if (automaticExecutionCheckbox.changed())
            definition.modify();

         ImGuiTools.previousWidgetTooltip("Enables autonomous execution. Will immediately start executing when checked.");

         ImGui.beginDisabled(state.getAutomaticExecution());
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
}