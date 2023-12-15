package us.ihmc.rdx.ui.behavior.sequence;

import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImString;
import us.ihmc.behaviors.sequence.ActionNodeDefinition;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.rdx.imgui.ImGuiHollowArrowRenderer;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.behavior.tree.RDXBehaviorTreeNode;

/**
 * The UI representation of a robot behavior action. It provides a base
 * template for implementing an interactable action.
 */
public abstract class RDXActionNode<S extends ActionNodeState<D>,
                                    D extends ActionNodeDefinition>
      extends RDXBehaviorTreeNode<S, D>
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean selected = new ImBoolean();
   private final ImBoolean expanded = new ImBoolean(true);
   private final ImString rejectionTooltip = new ImString();
   private final ImGuiHollowArrowRenderer hollowArrowRenderer = new ImGuiHollowArrowRenderer();

   public RDXActionNode(S state)
   {
      super(state);
   }

   @Override
   public void renderTreeViewIconArea()
   {
      if (hollowArrowRenderer.render(getState().getIsNextForExecution()))
      {
         RDXActionSequence actionSequence = findSequenceParent(this);
         if (actionSequence != null)
         {
            actionSequence.getState().setExecutionNextIndex(getState().getActionIndex());
         }
      }
   }

   private RDXActionSequence findSequenceParent(RDXBehaviorTreeNode<?, ?> node)
   {
      if (node.getParent() == null)
      {
         return null;
      }
      else if (node.getParent() instanceof RDXActionSequence actionSequence)
      {
         return actionSequence;
      }
      else
      {
         return findSequenceParent(node.getParent());
      }
   }

   @Override
   public void renderImGuiWidgets()
   {
      if (expanded.get())
      {
         ImGui.checkbox(labels.get("Selected"), selected);
         ImGuiTools.previousWidgetTooltip("(Show gizmo)");
         ImGui.sameLine();
         ImGui.text("Type: %s   Index: %d".formatted(getActionTypeTitle(), getState().getActionIndex()));
         renderImGuiWidgetsInternal();
      }
   }

   protected void renderImGuiWidgetsInternal()
   {

   }

   public abstract String getActionTypeTitle();

   public ImBoolean getSelected()
   {
      return selected;
   }

   public ImBoolean getExpanded()
   {
      return expanded;
   }

   public ImString getRejectionTooltip()
   {
      return rejectionTooltip;
   }
}
