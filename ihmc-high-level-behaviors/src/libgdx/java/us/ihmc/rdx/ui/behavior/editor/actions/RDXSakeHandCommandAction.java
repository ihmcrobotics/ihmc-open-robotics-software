package us.ihmc.rdx.ui.behavior.editor.actions;

import imgui.flag.ImGuiCol;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImString;
import us.ihmc.avatar.sakeGripper.SakeHandCommandOption;
import us.ihmc.behaviors.sequence.actions.SakeHandCommandActionState;
import us.ihmc.rdx.imgui.ImBooleanWrapper;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.ImIntegerWrapper;
import us.ihmc.rdx.ui.behavior.editor.RDXBehaviorAction;
import us.ihmc.rdx.ui.behavior.editor.RDXBehaviorActionBasics;

import static us.ihmc.avatar.sakeGripper.SakeHandParameters.MAX_ANGLE_BETWEEN_FINGERS;
import static us.ihmc.avatar.sakeGripper.SakeHandParameters.MAX_TORQUE_NEWTONS;

public class RDXSakeHandCommandAction extends SakeHandCommandActionState implements RDXBehaviorAction
{
   private final RDXBehaviorActionBasics rdxActionBasics = new RDXBehaviorActionBasics(this);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImIntegerWrapper sideWidget = new ImIntegerWrapper(this::getSide, this::setSide, labels.get("Side"));

   private final String[] handConfigurationNames = new String[SakeHandCommandOption.values.length];
   private final ImIntegerWrapper handCommandOptionIndex = new ImIntegerWrapper(this::getHandConfigurationIndex,
                                                                                this::setHandConfigurationIndex,
                                                                                imInt -> ImGui.combo(labels.get("Predefined Options"), imInt, handConfigurationNames));
   private final float[] positionValue = {0.0f};
   private final float[] torqueValue = {0.0f};

   private final ImBooleanWrapper executeWithNextActionWrapper = new ImBooleanWrapper(this::getExecuteWithNextAction,
                                                                                      this::setExecuteWithNextAction,
                                                                                      imBoolean -> imgui.ImGui.checkbox(labels.get("Execute with next action"), imBoolean));

   public RDXSakeHandCommandAction()
   {
      for (int i = 0; i < SakeHandCommandOption.values.length; ++i)
      {
         handConfigurationNames[i] = SakeHandCommandOption.values[i].name();
      }
   }

   @Override
   public void update()
   {
      setGoalPosition(positionValue[0] / Math.toRadians(MAX_ANGLE_BETWEEN_FINGERS));
      setGoalTorque(torqueValue[0]);
   }

   @Override
   public void update(boolean concurrentActionIsNextForExecution)
   {
      update();
   }

   @Override
   public void renderImGuiWidgets()
   {
      rdxActionBasics.renderImGuiWidgets();
   }

   @Override
   public void renderImGuiSettingWidgets()
   {
      imgui.ImGui.sameLine();
      executeWithNextActionWrapper.renderImGuiWidget();
      
      ImGui.pushItemWidth(100.0f);
      sideWidget.renderImGuiWidget();
      ImGui.sameLine();
      handCommandOptionIndex.renderImGuiWidget();
      ImGui.popItemWidth();

      // Set position and torque slider values to match selected command option
      SakeHandCommandOption command = SakeHandCommandOption.values[getHandConfigurationIndex()];
      if (command.getGoalPosition() >= 0)
      {
         positionValue[0] = (float) (command.getGoalPosition() * Math.toRadians(MAX_ANGLE_BETWEEN_FINGERS));
      }
      if (command.getGoalTorque() >= 0)
      {
         torqueValue[0] = (float) command.getGoalTorque();
      }

      float lastPositionValue = positionValue[0];
      float lastTorqueValue = torqueValue[0];

      ImGui.sliderAngle(labels.get("Angle Between Fingers"), positionValue, 0.0f, MAX_ANGLE_BETWEEN_FINGERS);
      ImGui.pushStyleColor(ImGuiCol.SliderGrab, ImGuiTools.greenToRedGradiatedColor(torqueValue[0], 0.5, 0.7, 0.9));
      ImGui.sliderFloat(labels.get("Goal Torque"), torqueValue, 0.0f, 1.0f, String.format("%.1f N", (torqueValue[0] * MAX_TORQUE_NEWTONS)));
      ImGui.popStyleColor();

      // if user moved slider, set command to GOTO
      if (positionValue[0] != lastPositionValue || torqueValue[0] != lastTorqueValue)
      {
         setHandConfigurationIndex(SakeHandCommandOption.GOTO.ordinal());
      }
   }

   @Override
   public String getActionTypeTitle()
   {
      return "Hand Configuration";
   }

   @Override
   public ImBooleanWrapper getSelected()
   {
      return rdxActionBasics.getSelected();
   }

   @Override
   public ImBoolean getExpanded()
   {
      return rdxActionBasics.getExpanded();
   }

   @Override
   public ImString getImDescription()
   {
      return rdxActionBasics.getDescription();
   }

   @Override
   public ImString getRejectionTooltip()
   {
      return rdxActionBasics.getRejectionTooltip();
   }

   @Override
   public int getActionIndex()
   {
      return rdxActionBasics.getActionIndex();
   }

   @Override
   public void setActionIndex(int actionIndex)
   {
      rdxActionBasics.setActionIndex(actionIndex);
   }

   @Override
   public int getActionNextExecutionIndex()
   {
      return rdxActionBasics.getActionNextExecutionIndex();
   }

   @Override
   public void setActionNextExecutionIndex(int actionNextExecutionIndex)
   {
      rdxActionBasics.setActionNextExecutionIndex(actionNextExecutionIndex);
   }
}
