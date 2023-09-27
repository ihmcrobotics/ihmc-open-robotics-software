package us.ihmc.rdx.ui.behavior.editor.actions;

import imgui.ImGui;
import us.ihmc.behaviors.sequence.actions.HandWrenchActionDescription;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.behavior.editor.RDXBehaviorAction;

public class RDXHandWrenchAction extends RDXBehaviorAction
{
   private final HandWrenchActionDescription actionDescription = new HandWrenchActionDescription();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImDoubleWrapper trajectoryDurationWidget = new ImDoubleWrapper(actionDescription::getTrajectoryDuration,
                                                                                actionDescription::setTrajectoryDuration,
                                                                                imDouble -> ImGui.inputDouble(labels.get("Trajectory duration"), imDouble));
   private final ImDoubleWrapper forceWidget = new ImDoubleWrapper(actionDescription::getForce,
                                                                   actionDescription::setForce,
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
      return actionDescription.getSide().getPascalCaseName() + " Hand Wrench";
   }

   public HandWrenchActionDescription getActionDescription()
   {
      return actionDescription;
   }
}
