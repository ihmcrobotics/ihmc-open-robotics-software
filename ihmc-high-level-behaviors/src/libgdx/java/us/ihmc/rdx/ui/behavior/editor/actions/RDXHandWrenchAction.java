package us.ihmc.rdx.ui.behavior.editor.actions;

import imgui.ImGui;
import us.ihmc.behaviors.sequence.actions.HandWrenchActionDefinition;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.behavior.editor.RDXBehaviorAction;

public class RDXHandWrenchAction extends RDXBehaviorAction
{
   private final HandWrenchActionDefinition actionDefinition = new HandWrenchActionDefinition();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImDoubleWrapper trajectoryDurationWidget = new ImDoubleWrapper(actionDefinition::getTrajectoryDuration,
                                                                                actionDefinition::setTrajectoryDuration,
                                                                                imDouble -> ImGui.inputDouble(labels.get("Trajectory duration"), imDouble));
   private final ImDoubleWrapper forceWidget = new ImDoubleWrapper(actionDefinition::getForce,
                                                                   actionDefinition::setForce,
                                                                   imDouble -> ImGui.inputDouble(labels.get("Force"), imDouble));

   @Override
   public void renderImGuiSettingWidgets()
   {
      ImGui.pushItemWidth(80.0f);
      trajectoryDurationWidget.renderImGuiWidget();
      forceWidget.renderImGuiWidget();
      ImGui.popItemWidth();
   }

   @Override
   public String getActionTypeTitle()
   {
      return actionDefinition.getSide().getPascalCaseName() + " Hand Wrench";
   }

   public HandWrenchActionDefinition getActionDefinition()
   {
      return actionDefinition;
   }
}
