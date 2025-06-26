package us.ihmc.rdx.ui.teleoperation;

import imgui.ImGui;
import us.ihmc.psyonicros2.AbilityHandHardwareCommunication;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.interactable.RDXAbilityHand;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class RDXHandManager
{
   private final AbilityHandHardwareCommunication communication = new AbilityHandHardwareCommunication("H1ROSHandManager");

   private final SideDependentList<RDXAbilityHand> hands = new SideDependentList<>();

   public void create(RDXBaseUI baseUI)
   {
      communication.start();
      for (RobotSide side : RobotSide.values)
      {
         hands.put(side, new RDXAbilityHand(side, communication, baseUI));
      }


   }

   public void update()
   {
      for (RobotSide side : RobotSide.values)
      {
         hands.get(side).update();
      }
   }

   public void renderImGuiWidgets()
   {
      ImGui.text("Hand Open/Close Controls:");
      for (RobotSide side : RobotSide.values)
      {
         hands.get(side).renderImGuiWidgets();
      }
   }
}
