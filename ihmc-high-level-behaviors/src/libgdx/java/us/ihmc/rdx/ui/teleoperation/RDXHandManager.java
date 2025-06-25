package us.ihmc.rdx.ui.teleoperation;

import imgui.ImGui;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.commons.UnitConversions;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.psyonicros2.AbilityHandCommandType;
import us.ihmc.psyonicros2.AbilityHandHardwareCommunication;
import us.ihmc.psyonicros2.AbilityHandInterface;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.interactable.RDXAbilityHand;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class RDXHandManager
{
   private final SideDependentList<AbilityHandHardwareCommunication> communicationList = new SideDependentList<>();
   private final SideDependentList<RDXAbilityHand> hands = new SideDependentList<>();
   // new: store one slider value per side
   private final SideDependentList<float[]> fingerSliders = new SideDependentList<>();
   private final int actuatorCount = AbilityHandInterface.ACTUATOR_COUNT;

   private static final float OPEN_POSITION   = 45.0f;
   private static final float CLOSED_POSITION = -45.0f;

   public void create(RDXBaseUI baseUI, CommunicationHelper helper, ROS2SyncedRobotModel syncedRobotModel)
   {
      for (RobotSide side : RobotSide.values)
      {
         communicationList.put(side, new AbilityHandHardwareCommunication(side.toString()));
         hands.put(side, new RDXAbilityHand(side));
         fingerSliders.put(side, new float[]{0.0f});
      }
   }

   public void update()
   {
      for (RobotSide side : RobotSide.values)
      {
         communicationList.get(side).readState(hands.get(side));
      }
   }

   public void renderImGuiWidgets()
   {
      ImGui.text("Hand Open/Close Controls:");

      for (RobotSide side : RobotSide.values)
      {
         ImGui.text(side.toString() + ":");
         if (ImGui.button("Open"))
         {
            setAllActuators(side, OPEN_POSITION);
            communicationList.get(side).publishCommand(hands.get(side));
         }
         ImGui.sameLine();
         if (ImGui.button("Close"))
         {
            setAllActuators(side, CLOSED_POSITION);
            communicationList.get(side).publishCommand(hands.get(side));
         }

         float[] slider = fingerSliders.get(side);
         if (ImGui.sliderFloat("Control Fingers", slider, 0.0f, 90.0f))
         {
            setSelectedFingers(side, slider[0]);
            communicationList.get(side).publishCommand(hands.get(side));
         }

         ImGui.separator();
      }
   }

   private void setAllActuators(RobotSide side, float position)
   {
      RDXAbilityHand hand = hands.get(side);
      hand.setCommandType(AbilityHandCommandType.POSITION);
      for (int i = 0; i < actuatorCount - 1; i++)
      {
         hand.setCommandValue(i, position);
      }
      hand.setCommandValue(actuatorCount - 1, -position);
   }
   private void setSelectedFingers(RobotSide side, float position)
   {
      RDXAbilityHand hand = hands.get(side);
      hand.setCommandType(AbilityHandCommandType.POSITION);
      for (int i = 0; i < actuatorCount; i++)
      {
         if (i != 4 && i != 5)
         {
            hand.setCommandValue(i, position);
         }
      }
   }
}

