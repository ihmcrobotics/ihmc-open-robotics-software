package us.ihmc.rdx.ui.behavior.actions;

import imgui.ImGui;
import us.ihmc.avatar.sakeGripper.SakeHandCommandOption;
import us.ihmc.behaviors.sequence.actions.SakeHandCommandActionDefinition;
import us.ihmc.behaviors.sequence.actions.SakeHandCommandActionState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.rdx.imgui.ImBooleanWrapper;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.ImIntegerWrapper;
import us.ihmc.rdx.ui.behavior.sequence.RDXActionNode;
import us.ihmc.rdx.ui.widgets.ImGuiGripperWidget;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import static us.ihmc.avatar.sakeGripper.SakeHandParameters.MAX_ANGLE_BETWEEN_FINGERS;
import static us.ihmc.avatar.sakeGripper.SakeHandParameters.FINGERTIP_GRIP_FORCE_HARDWARE_LIMIT;

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
      positionWidget = new ImDoubleWrapper(getDefinition()::getHandOpenAngle,
                                           getDefinition()::setHandOpenAngle,
                                           imDouble -> ImGuiTools.sliderDouble(labels.get("Position"), imDouble, 0.0, 1.0,
                                                          "%.1f deg".formatted(getDefinition().getHandOpenAngle() * MAX_ANGLE_BETWEEN_FINGERS)));
      torqueWidget = new ImDoubleWrapper(getDefinition()::getMaxTorque,
                                           getDefinition()::setMaxTorque,
                                           imDouble -> ImGuiTools.sliderDouble(labels.get("Torque"), imDouble, 0.0, 1.0,
                                                                               "%.1f N".formatted(getDefinition().getMaxTorque() * FINGERTIP_GRIP_FORCE_HARDWARE_LIMIT)));
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

      SakeHandCommandOption sakeCommandOption = getDefinition().getSakeCommandOption();
      if (sakeCommandOption != SakeHandCommandOption.GOTO)
      {
         getDefinition().setHandOpenAngle(sakeCommandOption.getNormalizedHandOpenAngle());
         getDefinition().setMaxTorque(sakeCommandOption.getMaxTorque());
      }
   }

   @Override
   protected void renderImGuiWidgetsInternal()
   {
      ImGui.sameLine();
      executeWithNextActionWrapper.renderImGuiWidget();
      
      ImGui.pushItemWidth(100.0f);
      sideWidget.renderImGuiWidget();
      ImGui.sameLine();
      handCommandEnumWidget.renderImGuiWidget();
      ImGui.popItemWidth();

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

      gripperWidget.render(getDefinition().getSide(), ImGui.getFrameHeight());
      ImGui.sameLine();
   }

   @Override
   public String getActionTypeTitle()
   {
      return "Hand Configuration";
   }
}
