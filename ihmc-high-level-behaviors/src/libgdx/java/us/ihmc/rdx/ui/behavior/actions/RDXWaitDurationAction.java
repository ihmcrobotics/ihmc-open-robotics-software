package us.ihmc.rdx.ui.behavior.actions;

import imgui.ImGui;
import us.ihmc.behaviors.sequence.actions.WaitDurationActionDefinition;
import us.ihmc.behaviors.sequence.actions.WaitDurationActionState;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.behavior.sequence.RDXActionNode;

public class RDXWaitDurationAction extends RDXActionNode<WaitDurationActionState, WaitDurationActionDefinition>
{
   private final WaitDurationActionState state;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImDoubleWrapper waitDurationWidget;

   public RDXWaitDurationAction(long id)
   {
      state = new WaitDurationActionState(id, ROS2ActorDesignation.OPERATOR);

      waitDurationWidget = new ImDoubleWrapper(getDefinition()::getWaitDuration,
                                               getDefinition()::setWaitDuration,
                                               imDouble -> ImGui.inputDouble(labels.get("Wait duration"), imDouble));
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
      return String.format("Wait %.1f s", getDefinition().getWaitDuration());
   }

   @Override
   public WaitDurationActionState getState()
   {
      return state;
   }
}
