package us.ihmc.rdx.behaviorTree.actions;

import imgui.ImGui;
import us.ihmc.behaviors.behaviorTree.action.actions.NeckActionDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.NeckActionState;
import us.ihmc.rdx.behaviorTree.RDXBehaviorTreeRootNode;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiLabelledWidgetAligner;
import us.ihmc.rdx.imgui.ImGuiSliderDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.robotics.EuclidCoreMissingTools;

public class RDXNeckAction extends RDXActionNode<NeckActionState, NeckActionDefinition>
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiSliderDoubleWrapper pitchWidget;
   private final ImGuiSliderDoubleWrapper yawWidget;
   private final ImDoubleWrapper trajectoryDurationWidget;

   public RDXNeckAction(long id, RDXBehaviorTreeRootNode rootNode)
   {
      super(new NeckActionState(id, rootNode.getState()), rootNode);

      ImGuiLabelledWidgetAligner widgetAligner = new ImGuiLabelledWidgetAligner();
      pitchWidget = new ImGuiSliderDoubleWrapper("Pitch", "%.2f " + EuclidCoreMissingTools.DEGREE_SYMBOL,
                                                 -28.0,
                                                 28.0,
                                                 () -> Math.toDegrees(definition.getPitch()),
                                                 pitchDegrees -> definition.setPitch(Math.toRadians(pitchDegrees)));
      pitchWidget.addWidgetAligner(widgetAligner);
      yawWidget = new ImGuiSliderDoubleWrapper("Yaw", "%.2f " + EuclidCoreMissingTools.DEGREE_SYMBOL,
                                               -90.0,
                                               90.0,
                                               () -> Math.toDegrees(definition.getYaw()),
                                               yawDegrees -> definition.setYaw(Math.toRadians(yawDegrees)));
      yawWidget.addWidgetAligner(widgetAligner);
      trajectoryDurationWidget = new ImDoubleWrapper(definition::getTrajectoryDuration,
                                                     definition::setTrajectoryDuration,
                                                     imDouble -> ImGuiTools.volatileInputDouble(labels.get("Trajectory duration"), imDouble));
   }

   @Override
   public void update()
   {
      super.update();
   }

   @Override
   public void renderTreeViewRow()
   {
      super.renderRowBeginning();
      super.renderEditableName();

      ImGui.sameLine();
      ImGui.textDisabled("Neck");

      renderRowEnd();
   }

   @Override
   protected void renderImGuiWidgetsInternal()
   {
      pitchWidget.renderImGuiWidget();
      yawWidget.renderImGuiWidget();
      trajectoryDurationWidget.renderImGuiWidget();
   }

   @Override
   public String getLeafTypeTitle()
   {
      return "Neck";
   }
}
