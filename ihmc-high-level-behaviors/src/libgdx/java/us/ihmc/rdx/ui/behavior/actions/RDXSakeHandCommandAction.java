package us.ihmc.rdx.ui.behavior.actions;

import imgui.internal.ImGui;
import us.ihmc.avatar.sakeGripper.SakeHandCommandOption;
import us.ihmc.behaviors.sequence.actions.SakeHandCommandActionDefinition;
import us.ihmc.behaviors.sequence.actions.SakeHandCommandActionState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.rdx.imgui.*;
import us.ihmc.rdx.ui.behavior.sequence.RDXActionNode;
import us.ihmc.rdx.ui.widgets.ImGuiGripperWidget;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import static us.ihmc.avatar.sakeGripper.SakeHandParameters.MAX_ANGLE_BETWEEN_FINGERS;
import static us.ihmc.avatar.sakeGripper.SakeHandParameters.MAX_TORQUE_NEWTONS;

public class RDXSakeHandCommandAction extends RDXActionNode<SakeHandCommandActionState, SakeHandCommandActionDefinition>
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImIntegerWrapper sideWidget;
   private final String[] handConfigurationNames = new String[SakeHandCommandOption.values.length];
   private final ImIntegerWrapper handCommandEnumWidget;
   private final ImDoubleWrapper positionWidget;
   private final ImDoubleWrapper torqueWidget;
   private final ImBooleanWrapper executeWithNextActionWrapper;
   private final ImGuiGripperWidget gripperWidget = new ImGuiGripperWidget();

   public RDXSakeHandCommandAction(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(new SakeHandCommandActionState(id, crdtInfo, saveFileDirectory));

      getDefinition().setDescription("Hand configuration");

      sideWidget = new ImIntegerWrapper(getDefinition()::getSide, getDefinition()::setSide, labels.get("Side"));
      handCommandEnumWidget = new ImIntegerWrapper(getDefinition()::getHandConfigurationIndex,
                                                   getDefinition()::setHandConfigurationIndex,
                                                    imInt -> ImGui.combo(labels.get("Predefined Options"),
                                                                         imInt,
                                                                         handConfigurationNames));
      positionWidget = new ImDoubleWrapper(getDefinition()::getGoalPosition,
                                           getDefinition()::setGoalPosition,
                                           imDouble -> ImGuiTools.sliderDouble(labels.get("Position"), imDouble, 0.0, 1.0,
                                                          "%.1f deg".formatted(getDefinition().getGoalPosition() * MAX_ANGLE_BETWEEN_FINGERS)));
      torqueWidget = new ImDoubleWrapper(getDefinition()::getGoalTorque,
                                           getDefinition()::setGoalTorque,
                                           imDouble -> ImGuiTools.sliderDouble(labels.get("Torque"), imDouble, 0.0, 1.0,
                                                          "%.1f N".formatted(getDefinition().getGoalTorque() * MAX_TORQUE_NEWTONS)));
      executeWithNextActionWrapper = new ImBooleanWrapper(getDefinition()::getExecuteWithNextAction,
                                                          getDefinition()::setExecuteWithNextAction,
                                                          imBoolean -> imgui.ImGui.checkbox(labels.get("Execute with next action"),
                                                                                            imBoolean));

      for (int i = 0; i < SakeHandCommandOption.values.length; ++i)
      {
         handConfigurationNames[i] = SakeHandCommandOption.values[i].name();
      }
   }

   @Override
   public void update()
   {
      super.update();
   }

   @Override
   protected void renderImGuiWidgetsInternal()
   {
      imgui.ImGui.sameLine();
      executeWithNextActionWrapper.renderImGuiWidget();
      
      ImGui.pushItemWidth(100.0f);
      sideWidget.renderImGuiWidget();
      ImGui.sameLine();
      handCommandEnumWidget.renderImGuiWidget();
      ImGui.popItemWidth();

      SakeHandCommandOption sakeCommandOption = getDefinition().getSakeCommandOption();
      if (sakeCommandOption != SakeHandCommandOption.GOTO)
      {
         getDefinition().setGoalPosition(sakeCommandOption.getGoalPosition());
         getDefinition().setGoalTorque(sakeCommandOption.getGoalTorque());
      }

      positionWidget.renderImGuiWidget();
      torqueWidget.renderImGuiWidget();

      if (positionWidget.changed() || torqueWidget.changed())
      {
         getDefinition().setHandConfigurationIndex(SakeHandCommandOption.GOTO.ordinal());
      }
   }

   @Override
   public void renderTreeViewIconArea()
   {
      super.renderTreeViewIconArea();

      imgui.ImGui.sameLine();
      gripperWidget.render(getDefinition().getSide());
   }

   @Override
   public String getActionTypeTitle()
   {
      return "Hand Configuration";
   }
}
