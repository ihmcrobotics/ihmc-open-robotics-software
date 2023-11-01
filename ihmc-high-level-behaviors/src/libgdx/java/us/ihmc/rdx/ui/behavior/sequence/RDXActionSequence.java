package us.ihmc.rdx.ui.behavior.sequence;

import com.badlogic.gdx.graphics.Color;
import imgui.ImGui;
import imgui.ImVec2;
import imgui.flag.ImGuiCol;
import org.apache.commons.lang3.tuple.MutablePair;
import us.ihmc.behaviors.sequence.ActionSequenceDefinition;
import us.ihmc.behaviors.sequence.ActionSequenceState;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.rdx.imgui.ImBooleanWrapper;
import us.ihmc.rdx.imgui.ImGuiFlashingText;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.behavior.tree.RDXBehaviorTreeNode;
import us.ihmc.tools.Timer;

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

   public RDXActionSequence(long id)
   {
      state = new ActionSequenceState(id, ROS2ActorDesignation.OPERATOR);

      automaticExecutionCheckbox = new ImBooleanWrapper(state::getAutomaticExecution,
                                                        state::setAutomaticExecution,
                                                        imBoolean -> ImGui.checkbox(labels.get("Autonomously"), imBoolean));
   }

   @Override
   public void update()
   {
      super.update();
   }

   @Override
   public void renderImGuiWidgets()
   {
      if (ImGui.button(labels.get("<")))
      {
         getState().stepBackNextExecutionIndex();
         getState().freezeFromModification();
      }
      ImGuiTools.previousWidgetTooltip("Go to previous action");
      ImGui.sameLine();
      ImGui.text("Index: " + String.format("%03d", getState().getExecutionNextIndex()));
      ImGui.sameLine();
      if (ImGui.button(labels.get(">")))
      {
         getState().stepForwardNextExecutionIndex();
         getState().freezeFromModification();
      }
      ImGuiTools.previousWidgetTooltip("Go to next action");

      boolean endOfSequence = getState().getExecutionNextIndex() >= getChildren().size();
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
            getState().freezeFromModification();
         if (!canExecuteNextAction)
            ImGui.endDisabled();

         ImGuiTools.previousWidgetTooltip("Enables autonomous execution. Will immediately start executing when checked.");
         if (!getState().getAutomaticExecution())
         {
            ImGui.sameLine();

            if (!canExecuteNextAction)
            ImGui.pushStyleColor(ImGuiCol.Button, Color.RED.toIntBits());
            boolean confirmationState = manualExecutionOverrideTimer.isRunning(5.0);
            boolean disableManuallyExecuteButton = getState().getManualExecutionRequested().peek();
            if (disableManuallyExecuteButton)
               ImGui.beginDisabled();
            if (ImGui.button(labels.get(confirmationState ? "Manually (confirm)" : "Manually")))
            {
               if (canExecuteNextAction || confirmationState)
               {
                  getState().getManualExecutionRequested().set();
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
         else if (getState().getCurrentlyExecutingActions().isEmpty())
         {
            ImGui.text("Nothing executing.");
         }
      }

   }

   @Override
   public ActionSequenceState getState()
   {
      return state;
   }
}
