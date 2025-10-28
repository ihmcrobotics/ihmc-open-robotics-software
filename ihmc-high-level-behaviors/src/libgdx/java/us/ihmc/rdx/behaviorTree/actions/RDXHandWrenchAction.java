package us.ihmc.rdx.behaviorTree.actions;

import imgui.ImGui;
import us.ihmc.behaviors.behaviorTree.action.actions.HandWrenchActionDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.HandWrenchActionState;
import us.ihmc.rdx.behaviorTree.RDXBehaviorTreeRootNode;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;

public class RDXHandWrenchAction extends RDXActionNode<HandWrenchActionState, HandWrenchActionDefinition>
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImDoubleWrapper trajectoryDurationWidget;
   private final ImDoubleWrapper forceWidget;

   public RDXHandWrenchAction(long id, RDXBehaviorTreeRootNode rootNode)
   {
      super(new HandWrenchActionState(id, rootNode.getState()), rootNode);

      trajectoryDurationWidget = new ImDoubleWrapper(definition::getTrajectoryDuration,
                                                     definition::setTrajectoryDuration,
                                                     imDouble -> ImGui.inputDouble(labels.get("Trajectory duration"), imDouble));
      forceWidget = new ImDoubleWrapper(definition::getForce,
                                        definition::setForce,
                                        imDouble -> ImGui.inputDouble(labels.get("Force"), imDouble));
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
   public String getLeafTypeTitle()
   {
      return definition.getSide().getPascalCaseName() + " Hand Wrench";
   }
}
