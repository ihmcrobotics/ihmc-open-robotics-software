package us.ihmc.rdx.ui.behavior.sequence;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.type.ImString;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeTools;
import us.ihmc.behaviors.sequence.ActionNodeDefinition;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.behaviors.sequence.ActionSequenceState;
import us.ihmc.rdx.imgui.ImGuiFlashingText;
import us.ihmc.rdx.imgui.ImGuiHollowArrowRenderer;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.behavior.tree.RDXBehaviorTreeNode;
import us.ihmc.rdx.ui.behavior.tree.RDXBehaviorTreeTools;

/**
 * The UI representation of a robot behavior action. It provides a base
 * template for implementing an interactable action.
 */
public abstract class RDXActionNode<S extends ActionNodeState<D>,
                                    D extends ActionNodeDefinition>
      extends RDXBehaviorTreeNode<S, D>
{
   private final S state;
   private final D definition;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImString rejectionTooltip = new ImString();
   private final RDXActionProgressWidgets progressWidgets = new RDXActionProgressWidgets(this);
   private final ImGuiHollowArrowRenderer hollowArrowRenderer = new ImGuiHollowArrowRenderer();
   private final ImGuiFlashingText flashingDescriptionColor = new ImGuiFlashingText(ImGuiTools.RED);
   private boolean wasFailed = false;

   public RDXActionNode(S state)
   {
      super(state);

      this.state = state;
      definition = getDefinition();
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
      RDXActionSequence actionSequence = RDXBehaviorTreeTools.findActionSequenceAncestor(this);
      if (actionSequence != null)
      {
         ImGui.pushStyleColor(ImGuiCol.Text, ImGui.getColorU32(ImGuiCol.TextDisabled));
         ImGui.text(state.getConcurrencyRank() > 1 ? String.valueOf(state.getConcurrencyRank()) : " ");
         ImGui.popStyleColor();
         ImGui.sameLine();

         if (hollowArrowRenderer.render(state.getIsNextForExecution(), ImGui.getFrameHeight()))
         {
            setSpecificWidgetOnRowClicked();
            actionSequence.getState().setExecutionNextIndex(state.getActionIndex());
         }
         ImGui.sameLine();
      }
   }

   @Override
   public void renderNodeSettingsWidgets()
   {
      ImGui.text("Type: %s   Index: %d".formatted(getActionTypeTitle(), state.getActionIndex()));

      ActionSequenceState actionSequence = BehaviorTreeTools.findActionSequenceAncestor(state);

      if (ImGui.beginCombo(labels.get("Execute after"), definition.getExecuteAfterAction()))
      {
         if (ImGui.selectable(labels.get("Previous"), definition.getExecuteAfterPrevious()))
         {
            definition.setExecuteAfterAction(ActionNodeDefinition.EXECUTE_AFTER_PREVIOUS);

            if (actionSequence != null)
               actionSequence.updateExecuteAfterNodeReferences();
         }
         if (ImGui.selectable(labels.get("Beginning"), definition.getExecuteAfterBeginning()))
         {
            definition.setExecuteAfterAction(ActionNodeDefinition.EXECUTE_AFTER_BEGINNING);

            if (actionSequence != null)
               actionSequence.updateExecuteAfterNodeReferences();
         }

         if (actionSequence != null)
         {
            for (ActionNodeState<?> actionChild : actionSequence.getActionChildren())
            {
               if (ImGui.selectable(labels.get(actionChild.getDefinition().getName()), actionChild == state.getExecuteAfterNode()))
               {
                  definition.setExecuteAfterAction(actionChild.getDefinition().getName());
                  actionSequence.updateExecuteAfterNodeReferences();
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

   public abstract String getActionTypeTitle();

   @Override
   public int getNameColor()
   {
      return flashingDescriptionColor.getTextColor(state.getFailed());
   }

   public ImString getRejectionTooltip()
   {
      return rejectionTooltip;
   }

   public RDXActionProgressWidgets getProgressWidgets()
   {
      return progressWidgets;
   }
}
