package us.ihmc.rdx.behaviorTree;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import us.ihmc.behaviors.behaviorTree.LeafNodeDefinition;
import us.ihmc.behaviors.behaviorTree.LeafNodeState;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.imgui.ImGuiFlashingColors;
import us.ihmc.rdx.imgui.ImGuiFlashingText;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;
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
   private float offsetY = 0.0f;

   public RDXLeafNode(S state, RDXBehaviorTreeRootNode rootNode)
   {
      super(state, rootNode);
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
   public void renderTreeViewRow()
   {
      renderRowBeginning();
      renderEditableName();
      renderConcurrencyGraph();
   }

   @Override
   public void renderRowBeginning()
   {
      super.renderRowBeginning();

      // Give the arrow a little space to the left, like the other icons
      ImGui.setCursorPosX(ImGui.getCursorPosX() + ImGui.getStyle().getItemSpacingX());

      boolean colorArrow = state.getIsNextForExecution() || state.getIsExecuting();
      int arrowColor = state.getIsNextForExecution() ? ImGuiTools.GREEN : isExecutingFlashingColor.getColor(state.getIsExecuting());
      if (hollowArrowRenderer.render(colorArrow, arrowColor, ImGui.getFrameHeight()))
      {
         anySpecificWidgetOnLineClicked = true;
         rootNode.getState().setExecutionNextIndex(state.getLeafIndex());
      }
      ImGui.sameLine();
   }

   public void renderConcurrencyGraph()
   {
      float frameHeight = ImGui.getFrameHeight();
      ImGui.sameLine(ImGui.getColumnWidth(), 0.0f);
      offsetY = ImGui.getCursorScreenPosY();

      if (!definition.getExecuteAfterPrevious())
      {
         RDXLeafNode<?, ?> executeAfterLeaf = getExecuteAfterLeaf();
         if (executeAfterLeaf != null)
         {
            int color = ImGui.getColorU32(getSelected() ? ImGuiCol.Text : ImGuiCol.TextDisabled);
            int scale = ImGui.getFontSize();
            float thickness = mouseHoveringNodeLine ? 2.0f : 1.0f;
            float offsetX = ImGui.getCursorScreenPosX();
            float executeAfterY = executeAfterLeaf.offsetY + frameHeight * 0.5f;
            ImGui.getWindowDrawList().addLine(offsetX, offsetY + frameHeight * 0.5f, offsetX, executeAfterY, color, thickness);
            ImGui.getWindowDrawList().addLine(offsetX - scale * 0.4f, executeAfterY + scale * 0.5f, offsetX, executeAfterY, color, thickness);
            ImGui.getWindowDrawList().addLine(offsetX + scale * 0.4f, executeAfterY + scale * 0.5f, offsetX, executeAfterY, color, thickness);
            ImGui.getWindowDrawList().addCircle(offsetX, offsetY + frameHeight * 0.5f, scale * 0.15f, color, 16, thickness);
         }
      }
      ImGui.dummy(0.0f, frameHeight);
   }

   public void renderConcurrencyRank()
   {
      // Probably better to display some parallel bars like a Git log view maybe. @dcalvert
      if (state.getConcurrencyRank() != 1)
      {
         ImGui.pushStyleColor(ImGuiCol.Text, ImGui.getColorU32(ImGuiCol.TextDisabled));
         String text = state.getConcurrencyRank() == 1 ? " " : String.valueOf(state.getConcurrencyRank());
         ImGui.setCursorPosX(ImGui.getCursorPosX() - ImGuiTools.calcTextSizeX(text) - ImGui.getStyle().getItemSpacingX());
         ImGui.text(text);
         ImGui.popStyleColor();
         ImGui.sameLine();
      }
   }

   @Override
   public void renderNodeSettingsWidgets()
   {
      ImGui.text("Type: %s   Index: %d".formatted(getLeafTypeTitle(), state.getLeafIndex()));

      // Validate state in case something earlier in this UI tick messed with things.
      // This happens with the Undo non-topological changes button.
      state.validateDefinition(rootNode.getState().getOrderedLeaves());

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

         for (LeafNodeState<?> leafNode : rootNode.getState().getOrderedLeaves())
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



   /** @return the leaf to execute after as part of the concurrency system */
   public RDXLeafNode getExecuteAfterLeaf()
   {
      if (rootNode.getIDToNodeMap().get(definition.getExecuteAfterNodeID()) instanceof RDXLeafNode executeAfterNode)
      {
         return executeAfterNode;
      }

      LogTools.error("Node ID not found: {}", definition.getExecuteAfterNodeID());

      return null;
   }
}
