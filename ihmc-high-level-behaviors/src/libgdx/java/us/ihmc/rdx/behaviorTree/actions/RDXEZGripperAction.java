package us.ihmc.rdx.behaviorTree.actions;

import imgui.ImGui;
import us.ihmc.avatar.sakeGripper.SakeHandParameters;
import us.ihmc.avatar.sakeGripper.SakeHandPreset;
import us.ihmc.behaviors.behaviorTree.action.actions.EZGripperActionDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.EZGripperActionState;
import us.ihmc.rdx.behaviorTree.RDXBehaviorTreeRootNode;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiLabelledWidgetAligner;
import us.ihmc.rdx.imgui.ImGuiSliderDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.ImIntegerWrapper;
import us.ihmc.rdx.ui.widgets.ImGuiGripperWidget;
import us.ihmc.robotics.EuclidCoreMissingTools;

public class RDXEZGripperAction extends RDXActionNode<EZGripperActionState, EZGripperActionDefinition>
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImIntegerWrapper sideWidget;
   private final ImGuiSliderDoubleWrapper handOpenAngleSlider;
   private final ImDoubleWrapper initialSatisfactionHandAngleToleranceInput;
   private final ImDoubleWrapper completionHandAngleToleranceInput;
   private final ImGuiSliderDoubleWrapper fingertipGripForceSlider;
   private final ImGuiGripperWidget gripperWidget = new ImGuiGripperWidget();

   public RDXEZGripperAction(long id, RDXBehaviorTreeRootNode rootNode)
   {
      super(new EZGripperActionState(id, rootNode.getState()), rootNode);

      sideWidget = new ImIntegerWrapper(definition::getSide, definition::setSide, labels.get("Side"));
      ImGuiLabelledWidgetAligner widgetAligner = new ImGuiLabelledWidgetAligner();
      handOpenAngleSlider = new ImGuiSliderDoubleWrapper("Hand Open Angle", "", 0.0, Math.toRadians(SakeHandParameters.MAX_DESIRED_HAND_OPEN_ANGLE_DEGREES),
                                                         definition::getHandOpenAngle,
                                                         definition::setHandOpenAngle);
      handOpenAngleSlider.addWidgetAligner(widgetAligner);
      initialSatisfactionHandAngleToleranceInput = new ImDoubleWrapper(
            () -> Math.toDegrees(definition.getInitialSatisfactionHandAngleTolerance()),
            initialSatisfactionHandAngleToleranceDegrees ->
                  definition.setInitialSatisfactionHandAngleTolerance(Math.toRadians(initialSatisfactionHandAngleToleranceDegrees)),
            imDouble -> ImGui.inputDouble(labels.get("Initial Satisfaction Hand Angle Tolerance (%s)".formatted(EuclidCoreMissingTools.DEGREE_SYMBOL)),
                                          imDouble));
      completionHandAngleToleranceInput = new ImDoubleWrapper(
            () -> Math.toDegrees(definition.getCompletionHandAngleTolerance()),
            completionHandAngleToleranceDegrees -> definition.setCompletionHandAngleTolerance(Math.toRadians(completionHandAngleToleranceDegrees)),
            imDouble -> ImGui.inputDouble(labels.get("Completion Hand Angle Tolerance (%s)".formatted(EuclidCoreMissingTools.DEGREE_SYMBOL)), imDouble));
      fingertipGripForceSlider = new ImGuiSliderDoubleWrapper("Fingertip Torque Limit", "%.1f N", 0.0, SakeHandParameters.FINGERTIP_GRIP_FORCE_HARDWARE_LIMIT,
                                                              definition::getFingertipGripForceLimit,
                                                              definition::setFingertipGripForceLimit);
      fingertipGripForceSlider.addWidgetAligner(widgetAligner);
   }

   @Override
   public void update()
   {
      super.update();
   }

   @Override
   protected void renderImGuiWidgetsInternal()
   {
      ImGui.pushItemWidth(100.0f);
      sideWidget.renderImGuiWidget();

      for (SakeHandPreset preset : SakeHandPreset.values)
      {
         ImGui.sameLine();
         if (ImGui.button(labels.get(preset.getPascalCasedName())))
         {
            definition.setHandOpenAngle(preset.getHandOpenAngle());
            definition.setFingertipGripForceLimit(preset.getFingertipGripForceLimit());
         }
      }

      handOpenAngleSlider.setWidgetText("%.1f%s".formatted(Math.toDegrees(definition.getHandOpenAngle()), EuclidCoreMissingTools.DEGREE_SYMBOL));

      handOpenAngleSlider.renderImGuiWidget();
      initialSatisfactionHandAngleToleranceInput.renderImGuiWidget();
      completionHandAngleToleranceInput.renderImGuiWidget();
      fingertipGripForceSlider.renderImGuiWidget();
   }

   @Override
   public void renderTreeViewRow()
   {
      super.renderRowBeginning();
      super.renderEditableName();

      ImGui.sameLine();
      gripperWidget.render(definition.getSide(), ImGui.getFrameHeight());

      renderRowEnd();
   }

   @Override
   public String getLeafTypeTitle()
   {
      return "Hand Configuration";
   }
}
