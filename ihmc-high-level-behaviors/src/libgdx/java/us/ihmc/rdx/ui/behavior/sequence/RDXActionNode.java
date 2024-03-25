package us.ihmc.rdx.ui.behavior.sequence;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.type.ImString;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeTools;
import us.ihmc.behaviors.sequence.ActionNodeDefinition;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.behaviors.sequence.ActionSequenceState;
import us.ihmc.rdx.imgui.ImGuiFlashingColors;
import us.ihmc.rdx.imgui.ImGuiFlashingText;
import us.ihmc.rdx.imgui.ImGuiHollowArrowRenderer;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.behavior.tree.RDXBehaviorTreeNode;
import us.ihmc.rdx.ui.behavior.tree.RDXBehaviorTreeTools;

import java.util.List;

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
   private final ImGuiFlashingColors isExecutingFlashingColor = new ImGuiFlashingColors(0.1, ImGuiTools.PURPLE, ImGuiTools.DARK_PURPLE);
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
         // Not displaying this now until we calculate it correctly. @dcalvert
         // if (state.getConcurrencyRank() != 1)
         // {
         //    ImGui.pushStyleColor(ImGuiCol.Text, ImGui.getColorU32(ImGuiCol.TextDisabled));
         //    ImGui.text(state.getConcurrencyRank() == 1 ? " " : String.valueOf(state.getConcurrencyRank()));
         //    ImGui.popStyleColor();
         //    ImGui.sameLine();
         // }
         // else
         // {
         //    ImGui.setCursorPosX(ImGui.getCursorPosX() + ImGuiTools.calcTextSizeX("2") + ImGui.getStyle().getItemSpacingX());
         // }

         boolean colorArrow = state.getIsNextForExecution() || state.getIsExecuting();
         int arrowColor = state.getIsNextForExecution() ? ImGuiTools.GREEN : isExecutingFlashingColor.getColor(state.getIsExecuting());
         if (hollowArrowRenderer.render(colorArrow, arrowColor, ImGui.getFrameHeight()))
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

      if (actionSequence != null)
      {
         // Validate state case something earlier in this UI tick messed with things.
         // This happens with the Undo non-topological changes button.
         updateAndValidateExecuteAfter(actionSequence.getActionChildren());

         String selectedText;
         if (definition.getExecuteAfterPrevious().getValue())
         {
            selectedText = ActionNodeDefinition.EXECUTE_AFTER_PREVIOUS;
         }
         else if (definition.getExecuteAfterBeginning().getValue())
         {
            selectedText = ActionNodeDefinition.EXECUTE_AFTER_BEGINNING;
         }
         else
         {
            int executeAfterActionIndex = state.calculateExecuteAfterActionIndex(actionSequence.getActionChildren());
            ActionNodeState<?> executeAfterAction = actionSequence.getActionChildren().get(executeAfterActionIndex);
            selectedText = executeAfterAction.getDefinition().getName();
         }

         if (ImGui.beginCombo(labels.get("Execute after"), selectedText))
         {
            if (ImGui.selectable(labels.get("Previous"), definition.getExecuteAfterPrevious().getValue()))
            {
               definition.getExecuteAfterPrevious().setValue(true);
               definition.getExecuteAfterBeginning().setValue(false);
               definition.updateAndSanitizeExecuteAfterFields(null);
            }
            if (ImGui.selectable(labels.get("Beginning"), definition.getExecuteAfterBeginning().getValue()))
            {
               definition.getExecuteAfterPrevious().setValue(false);
               definition.getExecuteAfterBeginning().setValue(true);
               definition.updateAndSanitizeExecuteAfterFields(null);
            }

            for (ActionNodeState<?> actionChild : actionSequence.getActionChildren())
            {
               if (actionChild.getActionIndex() < state.getActionIndex())
               {
                  if (ImGui.selectable(labels.get(actionChild.getDefinition().getName()), definition.getExecuteAfterNodeID().getValue() == actionChild.getID()))
                  {
                     definition.getExecuteAfterPrevious().setValue(false);
                     definition.getExecuteAfterBeginning().setValue(false);
                     definition.getExecuteAfterNodeID().setValue((int) actionChild.getID());
                     definition.updateAndSanitizeExecuteAfterFields(actionChild.getDefinition().getName());
                  }
               }
            }

            ImGui.endCombo();
         }
      }

      renderImGuiWidgetsInternal();

      super.renderNodeSettingsWidgets();
   }

   /**
    * Updates the definition executeAfterActionName string for
    * saving an up to date human readable name in the JSON.
    * It also finds the correct node upon loading the name from JSON.
    * We do it here because only the operator side is allowed to change the definition.
    */
   public void updateAndValidateExecuteAfter(List<ActionNodeState<?>> actionStateChildren)
   {
      String executeAfterActionName = null;

      if (!definition.getExecuteAfterPrevious().getValue() && !definition.getExecuteAfterBeginning().getValue())
      {
         // We need to find the node by name
         // This happens when we load from JSON
         if (definition.getExecuteAfterNodeID().getValue() == 0)
         {
            for (int j = state.getActionIndex() - 1; j >= 0; j--)
            {
               ActionNodeState<?> actionStateToCompare = actionStateChildren.get(j);
               if (actionStateToCompare.getDefinition().getName().equals(definition.getExecuteAfterActionName()))
               {
                  executeAfterActionName = actionStateToCompare.getDefinition().getName();
                  definition.getExecuteAfterNodeID().setValue(actionStateToCompare.getID());
               }
            }
         }
         else // Update the node's name for saving in human readable format
         {
            long executeAfterID = definition.getExecuteAfterNodeID().getValue();
            for (int j = state.getActionIndex() - 1; j >= 0; j--)
            {
               ActionNodeState<?> actionStateToCompare = actionStateChildren.get(j);
               if (actionStateToCompare.getID() == executeAfterID)
               {
                  executeAfterActionName = actionStateToCompare.getDefinition().getName();
               }
            }
         }
      }

      definition.updateAndSanitizeExecuteAfterFields(executeAfterActionName);
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
