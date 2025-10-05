package us.ihmc.rdx.ui.behavior.sequence;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeTools;
import us.ihmc.behaviors.sequence.LeafNodeDefinition;
import us.ihmc.behaviors.sequence.LeafNodeState;
import us.ihmc.rdx.imgui.ImGuiExpandCollapseRenderer;
import us.ihmc.rdx.imgui.ImGuiFlashingColors;
import us.ihmc.rdx.imgui.ImGuiFlashingText;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.behavior.tree.RDXBehaviorTreeNode;
import us.ihmc.rdx.ui.behavior.tree.RDXBehaviorTreeRootNode;
import us.ihmc.rdx.ui.behavior.tree.RDXBehaviorTreeTools;
import us.ihmc.rdx.ui.widgets.ImGuiHollowArrowRenderer;

/**
 * RDX common implementation of all leaf nodes.
 */
public abstract class RDXLeafNode<S extends LeafNodeState<D>,
                                  D extends LeafNodeDefinition>
      extends RDXBehaviorTreeNode<S, D>
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiFlashingColors isExecutingFlashingColor = new ImGuiFlashingColors(0.1, ImGuiTools.PURPLE, ImGuiTools.DARK_PURPLE);
   private final ImGuiHollowArrowRenderer hollowArrowRenderer = new ImGuiHollowArrowRenderer();
   private final ImGuiFlashingText flashingDescriptionColor = new ImGuiFlashingText(ImGuiTools.RED);
   /** Used to trigger a UI notification when the action goes from !failed -> failed. */
   private boolean wasFailed = false;

   public RDXLeafNode(S state)
   {
      super(state);
   }

   @Override
   public void update()
   {
      super.update();

      if (!wasFailed && state.getFailed())
      {
         RDXBaseUI.pushNotification("%s failed".formatted(definition.getName()));
      }
      wasFailed = state.getFailed();
   }

   @Override
   public void renderTreeViewIconArea()
   {
      RDXBehaviorTreeRootNode actionSequence = RDXBehaviorTreeTools.findRootNode(this);
      if (actionSequence != null)
      {
         // Give the arrow a little space to the left, like the other icons
         ImGui.setCursorPosX(ImGui.getCursorPosX() + ImGui.getStyle().getItemSpacingX());

         // Not displaying this now until we calculate it correctly. @dcalvert
         if (state.getConcurrencyRank() != 1)
         {
            ImGui.pushStyleColor(ImGuiCol.Text, ImGui.getColorU32(ImGuiCol.TextDisabled));
            String text = state.getConcurrencyRank() == 1 ? " " : String.valueOf(state.getConcurrencyRank());
            ImGui.setCursorPosX(ImGui.getCursorPosX() - ImGuiTools.calcTextSizeX(text) - ImGui.getStyle().getItemSpacingX());
            ImGui.text(text);
            ImGui.popStyleColor();
            ImGui.sameLine();
         }

         boolean colorArrow = state.getIsNextForExecution() || state.getIsExecuting();
         int arrowColor = state.getIsNextForExecution() ? ImGuiTools.GREEN : isExecutingFlashingColor.getColor(state.getIsExecuting());
         if (hollowArrowRenderer.render(colorArrow, arrowColor, ImGui.getFrameHeight()))
         {
            setSpecificWidgetOnRowClicked();
            actionSequence.getState().setExecutionNextIndex(state.getLeafIndex());
         }
         ImGui.sameLine();
      }
   }

   @Override
   public void renderNodeSettingsWidgets()
   {
      ImGui.text("Type: %s   Index: %d".formatted(getLeafTypeTitle(), state.getLeafIndex()));

      BehaviorTreeRootNodeState actionSequence = BehaviorTreeTools.findRootNode(state);

      if (actionSequence != null)
      {
         // Validate state in case something earlier in this UI tick messed with things.
         // This happens with the Undo non-topological changes button.
         state.validateFields(actionSequence.getOrderedLeaves());

         if (ImGui.beginCombo(labels.get("Execute after"), definition.getExecuteAfterLeafName()))
         {
            if (ImGui.selectable(labels.get("Previous"), definition.getExecuteAfterPrevious()))
            {
               definition.setExecuteAfterPrevious();
            }
            if (ImGui.selectable(labels.get("Beginning"), definition.getExecuteAfterBeginning()))
            {
               definition.setExecuteAfterBeginning();
            }

            for (LeafNodeState<?> leafNode : actionSequence.getOrderedLeaves())
            {
               if (leafNode.getLeafIndex() < state.getLeafIndex())
               {
                  if (ImGui.selectable(labels.get(leafNode.getDefinition().getName()), definition.getExecuteAfterNodeID() == leafNode.getID()))
                  {
                     definition.setExecuteAfterLeaf(leafNode.getID(), leafNode.getDefinition().getName());
                  }
               }
            }

            ImGui.endCombo();
         }
      }

      renderImGuiWidgetsInternal();

      super.renderNodeSettingsWidgets();
   }

   protected void renderImGuiWidgetsInternal()
   {

   }

   @Override
   public void clearSelections()
   {
      super.clearSelections();

      if (!ImGui.getIO().getKeyCtrl())
         deselectGizmos();
   }

   public void deselectGizmos()
   {

   }

   public abstract String getLeafTypeTitle();

   @Override
   public int getNameColor()
   {
      return flashingDescriptionColor.getTextColor(state.getFailed());
   }
}
