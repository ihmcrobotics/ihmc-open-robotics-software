package us.ihmc.rdx.ui.behavior.sequence;

import com.badlogic.gdx.graphics.Color;
import imgui.ImGui;
import imgui.ImVec2;
import imgui.flag.ImGuiCol;
import org.apache.commons.lang3.tuple.MutablePair;
import us.ihmc.behaviors.sequence.ActionSequenceDefinition;
import us.ihmc.behaviors.sequence.ActionSequenceState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.log.LogTools;
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
   private final ImVec2 calcDescriptionTextSize = new ImVec2();
   private final ImVec2 expandButtonSize = new ImVec2();
   private float longestDescriptionLength;
   private final MutablePair<Integer, Integer> reorderRequest = MutablePair.of(-1, 0);
   private final Timer manualExecutionOverrideTimer = new Timer();
   private final ImGuiFlashingText executionRejectionTooltipText = new ImGuiFlashingText(Color.RED.toIntBits());
   private final List<RDXActionNode<?, ?>> currentlyExecutingActions = new ArrayList<>();
   private final RDXMultipleActionProgressBars multipleActionProgressBars = new RDXMultipleActionProgressBars();

   public RDXActionSequence(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(new ActionSequenceState(id, crdtInfo, saveFileDirectory));

      state = getState();

      getDefinition().setDescription("ActionSequence");

      automaticExecutionCheckbox = new ImBooleanWrapper(state::getAutomaticExecution,
                                                        state::setAutomaticExecution,
                                                        imBoolean -> ImGui.checkbox(labels.get("Autonomously"), imBoolean));
   }

   @Override
   public void update()
   {
      super.update();

      currentlyExecutingActions.clear();
      updateActionSubtree(this);
   }

   public void updateActionSubtree(RDXBehaviorTreeNode<?, ?> node)
   {
      for (RDXBehaviorTreeNode<?, ?> child : node.getChildren())
      {
         if (child instanceof RDXActionNode<?, ?> actionNode)
         {
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
   public void renderImGuiWidgets()
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
      
      boolean endOfSequence = getState().getExecutionNextIndex() >= getIndexEndOfSequence(getChildren());
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

         if (currentlyExecutingActions.isEmpty())
         {
            ImGui.text("Nothing executing.");
         }
         else
         {
            for (RDXActionNode<?, ?> currentlyExecutingAction : currentlyExecutingActions)
            {
               ImGui.text("Currently executing: " + currentlyExecutingAction.getDefinition().getDescription());
            }
         }
      }

      multipleActionProgressBars.getActionProgressBars().clear();
      for (RDXActionNode<?, ?> currentlyExecutingAction : currentlyExecutingActions)
      {
         RDXSingleActionProgressBars actionProgressBars = multipleActionProgressBars.getActionProgressBars().add();
         actionProgressBars.setAction(currentlyExecutingAction);
      }
      multipleActionProgressBars.render();
   }

   private int getIndexEndOfSequence(List<RDXBehaviorTreeNode<?, ?>> children) {
      int endIndex = children.size() - 1; // Count the immediate children
      for (var child : children)
      {
         if (child.getChildren().size() > 0)
         {
            // Add the count of child's children recursively
            endIndex += getIndexEndOfSequence(child.getChildren());
         }
      }
      return endIndex;
   }
}
