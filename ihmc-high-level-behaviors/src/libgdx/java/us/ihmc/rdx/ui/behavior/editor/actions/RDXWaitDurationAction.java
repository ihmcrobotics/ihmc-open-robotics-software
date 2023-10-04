package us.ihmc.rdx.ui.behavior.editor.actions;

import imgui.ImGui;
import us.ihmc.behaviors.sequence.actions.WaitDurationActionDefinition;
import us.ihmc.behaviors.sequence.actions.WaitDurationActionState;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.behavior.editor.RDXBehaviorAction;
import us.ihmc.rdx.ui.behavior.editor.RDXBehaviorActionSequenceEditor;

public class RDXWaitDurationAction extends RDXBehaviorAction
{
   private final WaitDurationActionState state = new WaitDurationActionState();
   private final WaitDurationActionDefinition definition = state.getDefinition();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImDoubleWrapper waitDurationWidget = new ImDoubleWrapper(definition::getWaitDuration,
                                                                          definition::setWaitDuration,
                                                                          imDouble -> ImGui.inputDouble(labels.get("Wait duration"), imDouble));

   public RDXWaitDurationAction(RDXBehaviorActionSequenceEditor editor)
   {
      super(editor);
   }

   @Override
   protected void renderImGuiWidgetsInternal()
   {
      ImGui.pushItemWidth(80.0f);
      waitDurationWidget.renderImGuiWidget();
      ImGui.popItemWidth();
   }

   @Override
   public String getActionTypeTitle()
   {
      return String.format("Wait %.1f s", definition.getWaitDuration());
   }

   @Override
   public WaitDurationActionState getState()
   {
      return state;
   }

   @Override
   public WaitDurationActionDefinition getDefinition()
   {
      return definition;
   }
}
