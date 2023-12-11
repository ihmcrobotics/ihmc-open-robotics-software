package us.ihmc.rdx.ui.behavior.actions;

import imgui.flag.ImGuiCol;
import imgui.internal.ImGui;
import us.ihmc.avatar.sakeGripper.SakeHandCommandOption;
import us.ihmc.behaviors.sequence.actions.SakeHandCommandActionDefinition;
import us.ihmc.behaviors.sequence.actions.SakeHandCommandActionState;
import us.ihmc.rdx.imgui.ImBooleanWrapper;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.ImIntegerWrapper;
import us.ihmc.rdx.ui.behavior.sequence.RDXBehaviorAction;
import us.ihmc.rdx.ui.behavior.sequence.RDXBehaviorActionSequenceEditor;

import static us.ihmc.avatar.sakeGripper.SakeHandParameters.MAX_ANGLE_BETWEEN_FINGERS;
import static us.ihmc.avatar.sakeGripper.SakeHandParameters.MAX_TORQUE_NEWTONS;

public class RDXSakeHandCommandAction extends RDXBehaviorAction
{
   private final SakeHandCommandActionState state = new SakeHandCommandActionState();
   private final SakeHandCommandActionDefinition definition = state.getDefinition();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImIntegerWrapper sideWidget = new ImIntegerWrapper(definition::getSide, definition::setSide, labels.get("Side"));

   private final String[] handConfigurationNames = new String[SakeHandCommandOption.values.length];
   private final ImIntegerWrapper handCommandOptionIndex = new ImIntegerWrapper(definition::getHandConfigurationIndex,
                                                                                definition::setHandConfigurationIndex,
                                                                                imInt -> ImGui.combo(labels.get("Predefined Options"),
                                                                                                     imInt,
                                                                                                     handConfigurationNames));
   private final float[] positionValue = {0.0f};
   private final float[] torqueValue = {0.0f};

   private final ImBooleanWrapper executeWithNextActionWrapper = new ImBooleanWrapper(definition::getExecuteWithNextAction,
                                                                                      definition::setExecuteWithNextAction,
                                                                                      imBoolean -> imgui.ImGui.checkbox(labels.get("Execute with next action"),
                                                                                                                        imBoolean));

   public RDXSakeHandCommandAction(RDXBehaviorActionSequenceEditor editor)
   {
      super(editor);

      for (int i = 0; i < SakeHandCommandOption.values.length; ++i)
      {
         handConfigurationNames[i] = SakeHandCommandOption.values[i].name();
      }
   }

   @Override
   public void update()
   {
      super.update();

      definition.setGoalPosition(positionValue[0] / Math.toRadians(MAX_ANGLE_BETWEEN_FINGERS));
      definition.setGoalTorque(torqueValue[0]);
   }

   @Override
   protected void renderImGuiWidgetsInternal()
   {
      imgui.ImGui.sameLine();
      executeWithNextActionWrapper.renderImGuiWidget();
      
      ImGui.pushItemWidth(100.0f);
      sideWidget.renderImGuiWidget();
      ImGui.sameLine();
      handCommandOptionIndex.renderImGuiWidget();
      ImGui.popItemWidth();

      // Set position and torque slider values to match selected command option
      SakeHandCommandOption command = SakeHandCommandOption.values[definition.getHandConfigurationIndex()];
      if (command.getDesiredPosition() >= 0)
      {
         positionValue[0] = (float) (command.getDesiredPosition() * Math.toRadians(MAX_ANGLE_BETWEEN_FINGERS));
      }
      if (command.getDesiredTorque() >= 0)
      {
         torqueValue[0] = (float) command.getDesiredTorque();
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
         definition.setHandConfigurationIndex(SakeHandCommandOption.CUSTOM.ordinal());
      }
   }

   @Override
   public String getActionTypeTitle()
   {
      return "Hand Configuration";
   }

   @Override
   public SakeHandCommandActionState getState()
   {
      return state;
   }

   @Override
   public SakeHandCommandActionDefinition getDefinition()
   {
      return definition;
   }
}
