package us.ihmc.rdx.ui.behavior.actions;

import imgui.ImGui;
import us.ihmc.behaviors.sequence.actions.HandWrenchActionDefinition;
import us.ihmc.behaviors.sequence.actions.HandWrenchActionState;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.behavior.sequence.RDXBehaviorAction;
import us.ihmc.rdx.ui.behavior.sequence.RDXBehaviorActionSequenceEditor;

public class RDXHandWrenchAction extends RDXBehaviorAction
{
   private final HandWrenchActionDefinition definition = new HandWrenchActionDefinition();
   private final HandWrenchActionState state;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImDoubleWrapper trajectoryDurationWidget = new ImDoubleWrapper(definition::getTrajectoryDuration,
                                                                                definition::setTrajectoryDuration,
                                                                                imDouble -> ImGui.inputDouble(labels.get("Trajectory duration"), imDouble));
   private final ImDoubleWrapper forceWidget = new ImDoubleWrapper(definition::getForce,
                                                                   definition::setForce,
                                                                   imDouble -> ImGui.inputDouble(labels.get("Force"), imDouble));

   public RDXHandWrenchAction(long id, RDXBehaviorActionSequenceEditor editor)
   {
      super(editor);

      state = new HandWrenchActionState(id, definition);
   }

   @Override
   protected void renderImGuiWidgetsInternal()
   {
      ImGui.pushItemWidth(80.0f);
      trajectoryDurationWidget.renderImGuiWidget();
      forceWidget.renderImGuiWidget();
      ImGui.popItemWidth();
   }

   @Override
   public String getActionTypeTitle()
   {
      return definition.getSide().getPascalCaseName() + " Hand Wrench";
   }

   @Override
   public HandWrenchActionState getState()
   {
      return state;
   }

   @Override
   public HandWrenchActionDefinition getDefinition()
   {
      return definition;
   }
}
