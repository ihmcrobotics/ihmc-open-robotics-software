package us.ihmc.rdx.ui.behavior.editor.actions;

import imgui.ImGui;
import us.ihmc.behaviors.sequence.actions.WaitDurationActionDefinition;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.behavior.editor.RDXBehaviorAction;

public class RDXWaitDurationAction extends RDXBehaviorAction
{
   private final WaitDurationActionDefinition actionDefinition = new WaitDurationActionDefinition();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImDoubleWrapper waitDurationWidget = new ImDoubleWrapper(actionDefinition::getWaitDuration,
                                                                          actionDefinition::setWaitDuration,
                                                                          imDouble -> ImGui.inputDouble(labels.get("Wait duration"), imDouble));

   @Override
   public void renderImGuiSettingWidgets()
   {
      ImGui.pushItemWidth(80.0f);
      waitDurationWidget.renderImGuiWidget();
      ImGui.popItemWidth();
   }

   @Override
   public WaitDurationActionDefinition getActionDefinition()
   {
      return actionDefinition;
   }

   @Override
   public String getActionTypeTitle()
   {
      return String.format("Wait %.1f s", actionDefinition.getWaitDuration());
   }
}
