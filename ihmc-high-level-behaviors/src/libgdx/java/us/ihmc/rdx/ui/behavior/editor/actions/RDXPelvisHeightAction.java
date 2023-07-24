package us.ihmc.rdx.ui.behavior.editor.actions;

import imgui.ImGui;
import us.ihmc.behaviors.sequence.actions.PelvisHeightActionData;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.behavior.editor.RDXBehaviorAction;

public class RDXPelvisHeightAction extends RDXBehaviorAction
{
   private final PelvisHeightActionData actionData = new PelvisHeightActionData();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImDoubleWrapper heightInWorldWidget = new ImDoubleWrapper(actionData::getHeightInWorld,
                                                                           actionData::setHeightInWorld,
                                                                           imDouble -> ImGui.inputDouble(labels.get("Height in world"), imDouble));
   private final ImDoubleWrapper trajectoryDurationWidget = new ImDoubleWrapper(actionData::getTrajectoryDuration,
                                                                                actionData::setTrajectoryDuration,
                                                                                imDouble -> ImGui.inputDouble(labels.get("Trajectory duration"), imDouble));

   public RDXPelvisHeightAction()
   {

   }

   @Override
   public void renderImGuiSettingWidgets()
   {
      ImGui.pushItemWidth(80.0f);
      heightInWorldWidget.renderImGuiWidget();
      trajectoryDurationWidget.renderImGuiWidget();
      ImGui.popItemWidth();
   }

   @Override
   public PelvisHeightActionData getActionData()
   {
      return actionData;
   }

   @Override
   public String getActionTypeTitle()
   {
      return "Pelvis Height";
   }
}
