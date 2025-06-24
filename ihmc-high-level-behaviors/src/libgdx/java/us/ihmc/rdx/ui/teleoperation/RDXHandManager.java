package us.ihmc.rdx.ui.teleoperation;

import imgui.ImGui;
import us.ihmc.commons.UnitConversions;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.psyonicros2.AbilityHandCommandType;
import us.ihmc.psyonicros2.AbilityHandHardwareCommunication;
import us.ihmc.psyonicros2.AbilityHandInterface;
import us.ihmc.rdx.imgui.ImGuiLabelledWidgetAligner;
import us.ihmc.rdx.imgui.ImGuiSliderDouble;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.interactable.RDXAbilityHand;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.Timer;

public class RDXHandManager
{
   private static final double SEND_PERIOD = UnitConversions.hertzToSeconds(5.0);
   private static final double FREEZE_DURATION = 1.0;

   private final RobotSide handSide;
   private final AbilityHandHardwareCommunication communication;
   private final RDXAbilityHand hand;
   private final int actuatorCount;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiLabelledWidgetAligner widgetAligner = new ImGuiLabelledWidgetAligner();

   private final ImGuiSliderDouble[] cmdValueSliders;
   private final TypedNotification<AbilityHandCommandType> userChangedCommandType = new TypedNotification<>();
   private final Notification userChangedCommandValues = new Notification();

   private final Throttler sendThrottler = new Throttler();
   private final Timer freezeExpiration = new Timer();

   public RDXHandManager(RobotSide handSide)
   {
      this.handSide = handSide;
      communication = new AbilityHandHardwareCommunication(handSide.toString());
      this.hand = new RDXAbilityHand(handSide);
      this.actuatorCount = AbilityHandInterface.ACTUATOR_COUNT;

      cmdValueSliders = new ImGuiSliderDouble[actuatorCount];
      for (int i = 0; i < actuatorCount; i++)
      {
         String label = "Cmd Value " + i;
         cmdValueSliders[i] = new ImGuiSliderDouble(label, "%.1f", Double.NaN);
         cmdValueSliders[i].addWidgetAligner(widgetAligner);
      }
   }
   public void update()
   {
      communication.readState(hand);

      if (!freezeExpiration.isRunning(FREEZE_DURATION))
      {
         for (int i = 0; i < actuatorCount; i++)
         {
            cmdValueSliders[i].setDoubleValue(hand.getCommandValue(i));
         }
      }

      if (sendThrottler.run(SEND_PERIOD))
      {
         boolean typeChanged = userChangedCommandType.poll();
         AbilityHandCommandType newType = typeChanged ? userChangedCommandType.read() : null;

         boolean valuesChanged = userChangedCommandValues.poll();

         if (typeChanged)
            hand.setCommandType(newType);

         if (valuesChanged)
         {
            for (int i = 0; i < actuatorCount; i++)
               hand.setCommandValue(i, (float) cmdValueSliders[i].getDoubleValue());
         }

         if (typeChanged || valuesChanged)
            communication.publishCommand(hand);
      }
   }

   public void renderImGuiWidgets()
   {
      ImGui.text("Command Type:");
      if (ImGui.beginCombo(labels.get("Command Type"), hand.getCommandType().name()))
      {
         for (AbilityHandCommandType type : AbilityHandCommandType.values())
         {
            boolean selected = (type == hand.getCommandType());
            if (ImGui.selectable(labels.get(type.name()), selected))
            {
               userChangedCommandType.set(type);
               freezeExpiration.reset();
            }
            if (selected)
               ImGui.setItemDefaultFocus();
         }
         ImGui.endCombo();
      }
      for (int i = 0; i < actuatorCount; i++)
      {
         double min = -180.0, max = 180.0;
         if (cmdValueSliders[i].render(min, max))
         {
            userChangedCommandValues.set();
            freezeExpiration.reset();
         }

         ImGui.sameLine();
         ImGui.text("Pos: " + hand.getActuatorPosition(i));
      }
   }
}
