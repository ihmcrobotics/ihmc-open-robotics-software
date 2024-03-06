package us.ihmc.rdx.ui.behavior.actions;

import imgui.ImGui;
import us.ihmc.avatar.sakeGripper.SakeHandParameters;
import us.ihmc.avatar.sakeGripper.SakeHandPreset;
import us.ihmc.behaviors.sequence.actions.SakeHandCommandActionDefinition;
import us.ihmc.behaviors.sequence.actions.SakeHandCommandActionState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.rdx.imgui.ImBooleanWrapper;
import us.ihmc.rdx.imgui.ImGuiLabelledWidgetAligner;
import us.ihmc.rdx.imgui.ImGuiSliderDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.ImIntegerWrapper;
import us.ihmc.rdx.ui.behavior.sequence.RDXActionNode;
import us.ihmc.rdx.ui.widgets.ImGuiGripperWidget;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class RDXSakeHandCommandAction extends RDXActionNode<SakeHandCommandActionState, SakeHandCommandActionDefinition>
{
   private final SakeHandCommandActionDefinition definition;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImIntegerWrapper sideWidget;
   private final ImGuiSliderDoubleWrapper handOpenAngleSlider;
   private final ImGuiSliderDoubleWrapper fingertipGripForceSlider;
   private final ImGuiGripperWidget gripperWidget = new ImGuiGripperWidget();

   public RDXSakeHandCommandAction(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(new SakeHandCommandActionState(id, crdtInfo, saveFileDirectory));

      definition = getDefinition();

      definition.setName("Hand configuration");

      sideWidget = new ImIntegerWrapper(definition::getSide, definition::setSide, labels.get("Side"));
      ImGuiLabelledWidgetAligner widgetAligner = new ImGuiLabelledWidgetAligner();
      handOpenAngleSlider = new ImGuiSliderDoubleWrapper("Hand Open Angle", "", 0.0, Math.toRadians(SakeHandParameters.MAX_DESIRED_HAND_OPEN_ANGLE_DEGREES),
                                                         definition::getHandOpenAngle,
                                                         definition::setHandOpenAngle);
      handOpenAngleSlider.addWidgetAligner(widgetAligner);
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
      fingertipGripForceSlider.renderImGuiWidget();
   }

   @Override
   public void renderTreeViewIconArea()
   {
      super.renderTreeViewIconArea();

      gripperWidget.render(definition.getSide(), ImGui.getFrameHeight());
      ImGui.sameLine();
   }

   @Override
   public String getActionTypeTitle()
   {
      return "Hand Configuration";
   }
}
