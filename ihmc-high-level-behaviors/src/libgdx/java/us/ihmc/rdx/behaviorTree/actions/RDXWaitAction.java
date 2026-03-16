package us.ihmc.rdx.behaviorTree.actions;

import imgui.ImGui;
import us.ihmc.behaviors.behaviorTree.action.actions.WaitActionDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.WaitActionState;
import us.ihmc.rdx.behaviorTree.RDXBehaviorTreeRootNode;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;

public class RDXWaitAction extends RDXActionNode<WaitActionState, WaitActionDefinition>
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImDoubleWrapper waitDurationWidget;

   public RDXWaitAction(long id, RDXBehaviorTreeRootNode rootNode)
   {
      super(new WaitActionState(id, rootNode.getState()), rootNode);

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
