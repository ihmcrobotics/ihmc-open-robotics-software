package us.ihmc.rdx.ui.behavior.actions;

import imgui.ImGui;
import us.ihmc.behaviors.sequence.actions.HandWrenchActionDefinition;
import us.ihmc.behaviors.sequence.actions.HandWrenchActionState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.rdx.imgui.*;
import us.ihmc.rdx.ui.behavior.sequence.RDXActionNode;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class RDXHandWrenchAction extends RDXActionNode<HandWrenchActionState, HandWrenchActionDefinition>
{
   private final HandWrenchActionState state;
   private final HandWrenchActionDefinition definition;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImDoubleWrapper trajectoryDurationWidget;
//   private final ImGuiSliderDoubleWrapper forceXWidget;
//   private final ImGuiSliderDoubleWrapper forceYWidget;
//   private final ImGuiSliderDoubleWrapper forceZWidget;
//   private final ImGuiSliderDoubleWrapper torqueXWidget;
//   private final ImGuiSliderDoubleWrapper torqueYWidget;
//   private final ImGuiSliderDoubleWrapper torqueZWidget;
   private final ImBooleanWrapper executeWithNextActionWrapper;
   private final ImDoubleWrapper forceWidget;

   public RDXHandWrenchAction(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(new HandWrenchActionState(id, crdtInfo, saveFileDirectory));

      state = getState();
      definition = getDefinition();

      definition.setName("Hand wrench");


      executeWithNextActionWrapper = new ImBooleanWrapper(definition::getExecuteWithNextAction,
                                                          definition::setExecuteWithNextAction,
                                                          imBoolean -> ImGui.checkbox(labels.get("Execute with next action"), imBoolean));

      forceWidget = new ImDoubleWrapper(getDefinition()::getForceY,
                                        getDefinition()::setForceY,
                                        imDouble -> ImGui.inputDouble(labels.get("Force"), imDouble));

//      ImGuiLabelledWidgetAligner widgetAligner = new ImGuiLabelledWidgetAligner();
//
//      forceXWidget = new ImGuiSliderDoubleWrapper("Force along X: ", "%.2f", -50.0, 50.0,
//                                                  definition::getForceX,
//                                                  definition::setForceX);
//      forceXWidget.addWidgetAligner(widgetAligner);
//
//      forceYWidget = new ImGuiSliderDoubleWrapper("Force along Y: ", "%.2f", -50.0, 50.0,
//                                                  definition::getForceY,
//                                                  definition::setForceY);
//      forceYWidget.addWidgetAligner(widgetAligner);
//
//      forceZWidget = new ImGuiSliderDoubleWrapper("Force along Z: ", "%.2f", -50.0, 50.0,
//                                                  definition::getForceZ,
//                                                  definition::setForceZ);
//      forceZWidget.addWidgetAligner(widgetAligner);
//
//      torqueXWidget = new ImGuiSliderDoubleWrapper("Torque along X: ", "%.2f", -50.0, 50.0,
//                                                   definition::getTorqueX,
//                                                   definition::setTorqueX);
//      torqueXWidget.addWidgetAligner(widgetAligner);
//
//      torqueYWidget = new ImGuiSliderDoubleWrapper("Torque along Y: ", "%.2f", -50.0, 50.0,
//                                                   definition::getTorqueY,
//                                                   definition::setTorqueY);
//      torqueYWidget.addWidgetAligner(widgetAligner);
//
//      torqueZWidget = new ImGuiSliderDoubleWrapper("Torque along Z: ", "%.2f", -50.0, 50.0,
//                                                   definition::getTorqueZ,
//                                                   definition::setTorqueZ);
//      torqueZWidget.addWidgetAligner(widgetAligner);

      trajectoryDurationWidget = new ImDoubleWrapper(getDefinition()::getTrajectoryDuration,
                                                     getDefinition()::setTrajectoryDuration,
                                                     imDouble -> ImGui.inputDouble(labels.get("Trajectory duration"), imDouble));
   }

   @Override
   protected void renderImGuiWidgetsInternal()
   {
      ImGui.pushItemWidth(80.0f);
      executeWithNextActionWrapper.renderImGuiWidget();
//      forceXWidget.renderImGuiWidget();
//      forceYWidget.renderImGuiWidget();
//      forceZWidget.renderImGuiWidget();
//      torqueXWidget.renderImGuiWidget();
//      torqueYWidget.renderImGuiWidget();
//      torqueZWidget.renderImGuiWidget();
      forceWidget.renderImGuiWidget();
      trajectoryDurationWidget.renderImGuiWidget();
      ImGui.popItemWidth();
   }

   @Override
   public String getActionTypeTitle()
   {
      return getDefinition().getSide().getPascalCaseName() + " Hand Wrench";
   }
}
