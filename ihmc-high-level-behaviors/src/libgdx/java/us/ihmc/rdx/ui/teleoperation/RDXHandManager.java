package us.ihmc.rdx.ui.teleoperation;

import imgui.ImGui;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.tools.CommunicationHelper;
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
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.interactable.RDXAbilityHand;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.Timer;

public class RDXHandManager
{
   private static final double SEND_PERIOD = UnitConversions.hertzToSeconds(5.0);
   private static final double FREEZE_DURATION = 1.0;
   private final SideDependentList<AbilityHandHardwareCommunication> communicationList = new SideDependentList<>();
   private final SideDependentList<RDXAbilityHand> hands =  new SideDependentList<>();
   private int actuatorCount;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiLabelledWidgetAligner widgetAligner = new ImGuiLabelledWidgetAligner();

   private ImGuiSliderDouble[] cmdValueSliders;
   private final TypedNotification<AbilityHandCommandType> userChangedCommandType = new TypedNotification<>();
   private final Notification userChangedCommandValues = new Notification();

   private final Throttler sendThrottler = new Throttler();
   private final Timer freezeExpiration = new Timer();

   public void create(RDXBaseUI baseUI, CommunicationHelper communicationHelper, ROS2SyncedRobotModel syncedRobotModel)
   {
      for (RobotSide side : RobotSide.values)
      {
         communicationList.put(side, new AbilityHandHardwareCommunication(side.toString()));
         hands.put(side, new RDXAbilityHand(side));
      }
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
      for(RobotSide side : RobotSide.values)
      {
         communicationList.get(side).readState(hands.get(side));

         if (!freezeExpiration.isRunning(FREEZE_DURATION))
         {
            for (int i = 0; i < actuatorCount; i++)
            {
               cmdValueSliders[i].setDoubleValue(hands.get(side).getCommandValue(i));
            }
         }

         if (sendThrottler.run(SEND_PERIOD))
         {
            boolean typeChanged = userChangedCommandType.poll();
            AbilityHandCommandType newType = typeChanged ? userChangedCommandType.read() : null;

            boolean valuesChanged = userChangedCommandValues.poll();

            if (typeChanged)
               hands.get(side).setCommandType(newType);

            if (valuesChanged)
            {
               for (int i = 0; i < actuatorCount; i++)
                  hands.get(side).setCommandValue(i, (float) cmdValueSliders[i].getDoubleValue());
            }

            if (typeChanged || valuesChanged)
               communicationList.get(side).publishCommand(hands.get(side));
         }
      }
   }

   public void renderImGuiWidgets()
   {
      ImGui.text("Command Type:");
      for (RobotSide side : RobotSide.values)
      {
         if (ImGui.beginCombo(labels.get("Command Type"), hands.get(side).getCommandType().name()))
         {
            for (AbilityHandCommandType type : AbilityHandCommandType.values())
            {
               boolean selected = (type == hands.get(side).getCommandType());
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
            ImGui.text("Pos: " + hands.get(side).getActuatorPosition(i));
         }
      }
   }
}
