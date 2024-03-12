package us.ihmc.rdx.ui.behavior.sequence;

import com.badlogic.gdx.graphics.Color;
import imgui.ImGui;
import imgui.flag.ImGuiCol;
import us.ihmc.behaviors.sequence.ActionNodeDefinition;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.behaviors.sequence.ActionSequenceDefinition;
import us.ihmc.behaviors.sequence.ActionSequenceState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.rdx.imgui.ImBooleanWrapper;
import us.ihmc.rdx.imgui.ImGuiFlashingText;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.behavior.tree.RDXBehaviorTreeNode;
import us.ihmc.tools.Timer;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import java.util.ArrayList;
import java.util.List;

public class RDXActionSequence extends RDXBehaviorTreeNode<ActionSequenceState, ActionSequenceDefinition>
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ActionSequenceState state;
   private final ImBooleanWrapper automaticExecutionCheckbox;
   private final Timer manualExecutionOverrideTimer = new Timer();
   private final ImGuiFlashingText executionRejectionTooltipText = new ImGuiFlashingText(Color.RED.toIntBits());
   private final List<RDXActionNode<?, ?>> nextForExecutionActions = new ArrayList<>();
   private final List<RDXActionNode<?, ?>> currentlyExecutingActions = new ArrayList<>();
   private final RDXActionProgressWidgetsManager progressWidgetsManager = new RDXActionProgressWidgetsManager();

   public RDXActionSequence(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(new ActionSequenceState(id, crdtInfo, saveFileDirectory));

      state = getState();

      getDefinition().setName("Action sequence");

      automaticExecutionCheckbox = new ImBooleanWrapper(state::getAutomaticExecution,
                                                        state::setAutomaticExecution,
                                                        imBoolean -> ImGui.checkbox(labels.get("Autonomously"), imBoolean));
   }

   @Override
   public void update()
   {
      super.update();

      nextForExecutionActions.clear();
      currentlyExecutingActions.clear();
      updateActionSubtree(this);

      // This block updates the definition executeAfterActionName string for
      // saving an up to date human readable name in the JSON.
      // It also finds the correct node upon loading the name from JSON.
      // We do it here because only the operator side is allowed to change the definition.
      for (int i = 0; i < state.getActionChildren().size(); i++)
      {
         ActionNodeState<?> actionState = state.getActionChildren().get(i);
         ActionNodeDefinition actionDefinition = actionState.getDefinition();

         String executeAfterActionName = null;

         if (!actionDefinition.getExecuteAfterPrevious().getValue()
          && !actionDefinition.getExecuteAfterBeginning().getValue())
         {
            // We need to find the node by name
            // This happens when we load from JSON
            if (actionDefinition.getExecuteAfterNodeID().getValue() == 0)
            {
               for (int j = i - 1; j >= 0; j--)
               {
                  ActionNodeState<?> actionStateToCompare = state.getActionChildren().get(j);
                  if (actionStateToCompare.getDefinition().getName().equals(actionDefinition.getExecuteAfterActionName()))
                  {
                     executeAfterActionName = actionStateToCompare.getDefinition().getName();
                     actionDefinition.getExecuteAfterNodeID().setValue(actionStateToCompare.getID());
                  }
               }
            }
            else // Update the node's name for saving in human readable format
            {
               long executeAfterID = actionDefinition.getExecuteAfterNodeID().getValue();
               for (int j = i - 1; j >= 0; j--)
               {
                  ActionNodeState<?> actionStateToCompare = state.getActionChildren().get(j);
                  if (actionStateToCompare.getID() == executeAfterID)
                  {
                     executeAfterActionName = actionStateToCompare.getDefinition().getName();
                  }
               }
            }
         }

         actionDefinition.updateAndSanitizeExecuteAfterFields(executeAfterActionName);
      }
   }

   public void updateActionSubtree(RDXBehaviorTreeNode<?, ?> node)
   {
      for (RDXBehaviorTreeNode<?, ?> child : node.getChildren())
      {
         if (child instanceof RDXActionNode<?, ?> actionNode)
         {
            if (actionNode.getState().getIsNextForExecution())
            {
               nextForExecutionActions.add(actionNode);
            }
            if (actionNode.getState().getIsExecuting())
            {
               currentlyExecutingActions.add(actionNode);
            }
         }
         else
         {
            updateActionSubtree(child);
         }
      }
   }

   @Override
   public void renderContextMenuItems()
   {
      super.renderContextMenuItems();

      if (ImGui.menuItem(labels.get("Render Progress Using Plots"), null, progressWidgetsManager.getRenderAsPlots()))
         progressWidgetsManager.setRenderAsPlots(!progressWidgetsManager.getRenderAsPlots());
   }

   public void renderExecutionControlAndProgressWidgets()
   {
      if (ImGui.button(labels.get("<")))
      {
         getState().stepBackNextExecutionIndex();
      }
      ImGuiTools.previousWidgetTooltip("Go to previous action");
      ImGui.sameLine();
      ImGui.text("Index: " + String.format("%03d", getState().getExecutionNextIndex()));
      ImGui.sameLine();
      if (ImGui.button(labels.get(">")))
      {
         getState().stepForwardNextExecutionIndex();
      }
      ImGuiTools.previousWidgetTooltip("Go to next action");

      boolean endOfSequence = getState().getExecutionNextIndex() >= getState().getActionChildren().size();
      if (!endOfSequence)
      {
         String nextActionRejectionTooltip = getState().getNextActionRejectionTooltip();
         boolean canExecuteNextAction = nextActionRejectionTooltip.isEmpty();

         ImGui.sameLine();
         ImGui.text("Execute");
         ImGui.sameLine();

         if (!canExecuteNextAction)
            ImGui.beginDisabled();
         automaticExecutionCheckbox.renderImGuiWidget();
         if (automaticExecutionCheckbox.changed())
            getState().freeze();
         if (!canExecuteNextAction)
            ImGui.endDisabled();

         ImGuiTools.previousWidgetTooltip("Enables autonomous execution. Will immediately start executing when checked.");
         if (!getState().getAutomaticExecution())
         {
            ImGui.sameLine();

            if (!canExecuteNextAction)
               ImGui.pushStyleColor(ImGuiCol.Button, Color.RED.toIntBits());
            boolean confirmationState = manualExecutionOverrideTimer.isRunning(5.0);
            boolean disableManuallyExecuteButton = getState().getManualExecutionRequested();
            if (disableManuallyExecuteButton)
               ImGui.beginDisabled();
            if (ImGui.button(labels.get(confirmationState ? "Manually (confirm)" : "Manually")))
            {
               if (canExecuteNextAction || confirmationState)
               {
                  getState().setManualExecutionRequested();
               }
               else
               {
                  manualExecutionOverrideTimer.reset();
               }
            }
            if (disableManuallyExecuteButton)
               ImGui.endDisabled();
            if (!canExecuteNextAction)
               ImGui.popStyleColor();
            ImGuiTools.previousWidgetTooltip("Executes the next action.");

            if (!getState().getNextActionRejectionTooltip().isEmpty())
            {
               executionRejectionTooltipText.renderText(getState().getNextActionRejectionTooltip(), true);
            }
         }

         if (endOfSequence)
         {
            ImGui.text("End of sequence.");
         }
      }

      if (currentlyExecutingActions.isEmpty())
      {
         ImGui.text("Nothing executing.");
      }
      else
      {
         ImGui.text("Executing:");
         for (RDXActionNode<?, ?> currentlyExecutingAction : currentlyExecutingActions)
         {
            ImGui.sameLine();
            ImGui.text("%s (%s)".formatted(currentlyExecutingAction.getDefinition().getName(),
                                           currentlyExecutingAction.getActionTypeTitle()));
         }
      }

      progressWidgetsManager.getActionNodesToRender().clear();
      int lastIndex = 0;
      for (RDXActionNode<?, ?> currentlyExecutingAction : currentlyExecutingActions)
      {
         progressWidgetsManager.getActionNodesToRender().add(currentlyExecutingAction);
         lastIndex = Math.max(lastIndex, currentlyExecutingAction.getState().getActionIndex());
      }
      for (RDXActionNode<?, ?> nextForExecutionAction : nextForExecutionActions)
      {
         if (currentlyExecutingActions.isEmpty() || nextForExecutionAction.getState().getActionIndex() < lastIndex)
            progressWidgetsManager.getActionNodesToRender().add(nextForExecutionAction);
      }
      progressWidgetsManager.render();
   }

   @Override
   public void renderNodeSettingsWidgets()
   {
      ImGui.text("Type: %s   ID: %d".formatted(getDefinition().getClass().getSimpleName(), getState().getID()));

      super.renderNodeSettingsWidgets();
   }
}