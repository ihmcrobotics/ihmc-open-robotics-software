package us.ihmc.rdx.ui.behavior.editor.actions;

import imgui.ImGui;
import us.ihmc.behaviors.sequence.actions.WaitDurationActionDescription;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.behavior.editor.RDXBehaviorAction;

public class RDXWaitDurationAction extends RDXBehaviorAction
{
   private final WaitDurationActionDescription actionDescription = new WaitDurationActionDescription();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImDoubleWrapper waitDurationWidget = new ImDoubleWrapper(actionDescription::getWaitDuration,
                                                                          actionDescription::setWaitDuration,
                                                                          imDouble -> ImGui.inputDouble(labels.get("Wait duration"), imDouble));

   @Override
   public void renderImGuiSettingWidgets()
   {
      ImGui.pushItemWidth(80.0f);
      waitDurationWidget.renderImGuiWidget();
      ImGui.popItemWidth();
   }

   @Override
   public WaitDurationActionDescription getActionDescription()
   {
      return actionDescription;
   }

   @Override
   public String getActionTypeTitle()
   {
      return String.format("Wait %.1f s", actionDescription.getWaitDuration());
   }
}
