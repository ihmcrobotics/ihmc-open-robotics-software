package us.ihmc.rdx.behaviorTree.actions;

import imgui.ImGui;
import us.ihmc.behaviors.behaviorTree.action.actions.WaitDurationActionDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.WaitDurationActionState;
import us.ihmc.rdx.behaviorTree.RDXBehaviorTreeRootNode;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;

public class RDXWaitDurationAction extends RDXActionNode<WaitDurationActionState, WaitDurationActionDefinition>
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImDoubleWrapper waitDurationWidget;

   public RDXWaitDurationAction(long id, RDXBehaviorTreeRootNode rootNode)
   {
      super(new WaitDurationActionState(id, rootNode.getState()), rootNode);

      waitDurationWidget = new ImDoubleWrapper(definition::getWaitDuration,
                                               definition::setWaitDuration,
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
   public String getLeafTypeTitle()
   {
      return String.format("Wait %.1f s", definition.getWaitDuration());
   }
}
