package us.ihmc.rdx.ui.hands;

import imgui.ImGui;
import imgui.flag.ImGuiTreeNodeFlags;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotVersion;
import us.ihmc.handsros2.HandInterface;
import us.ihmc.handsros2.HandType;
import us.ihmc.handsros2.abilityHand.AbilityHandROS2HardwareCommunication;
import us.ihmc.handsros2.ezGripper.EZGripperROS2HardwareCommunication;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.hands.psyonicAbilityHand.RDXAbilityHand;
import us.ihmc.rdx.ui.hands.sakeEZGripper.RDXEZGripper;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

/**
 * Manages the UI for a humanoid robot's hands. A hand configuration is like "open", "closed", etc.
 */
public class RDXHandManager
{
   private final SideDependentList<RDXHandInterface> rdxHands = new SideDependentList<>();
   private final SideDependentList<RDXHandQuickAccessButtons> quickAccessButtons = new SideDependentList<>();

   private EZGripperROS2HardwareCommunication ezGripperCommunication = null;
   private AbilityHandROS2HardwareCommunication abilityHandCommunication = null;

   public RDXHandManager(DRCRobotModel robotModel)
   {
      RobotVersion robotVersion = robotModel.getRobotVersion();
      String robotName = robotModel.getSimpleRobotName();

      for (RobotSide side : RobotSide.values)
      {
         HandType handType = robotVersion.getHandType(side);
         if (!robotVersion.hasHandWithFingers(side) || handType == null)
            continue;

         switch (handType)
         {
            case EZ_GRIPPER ->
            {
               if (ezGripperCommunication == null)
                  ezGripperCommunication = new EZGripperROS2HardwareCommunication(getClass().getSimpleName() + "EZGripperCommunication");
            }
            case ABILITY_HAND ->
            {
               if (abilityHandCommunication == null)
                  abilityHandCommunication = new AbilityHandROS2HardwareCommunication(getClass().getSimpleName() + "AbilityHandCommunication");
            }
         }

         String handIdentifier = HandInterface.getSimpleIdentifier(robotName, side, handType);
         switch (handType)
         {
            case EZ_GRIPPER ->
                  rdxHands.put(side, new RDXEZGripper(handIdentifier, side, ezGripperCommunication));
            case ABILITY_HAND ->
                  rdxHands.put(side, new RDXAbilityHand(handIdentifier, side, abilityHandCommunication));
         }
      }
   }

   public void create(RDXBaseUI baseUI)
   {
      if (ezGripperCommunication != null)
         ezGripperCommunication.start();
      if (abilityHandCommunication != null)
         abilityHandCommunication.start();

      for (RobotSide side : rdxHands.sides())
         quickAccessButtons.put(side, new RDXHandQuickAccessButtons(baseUI, rdxHands.get(side)));
   }

   public void update()
   {
      // Update the hands
      for (RobotSide side : rdxHands.sides())
      {
         rdxHands.get(side).update();
         quickAccessButtons.get(side).update();
      }
   }

   public void renderImGuiWidgets()
   {
      if (ImGui.collapsingHeader("Hands", ImGuiTreeNodeFlags.DefaultOpen))
      {
         rdxHands.values().forEach(RDXHandInterface::renderImGuiWidgets);
      }
   }

   public SideDependentList<RDXHandInterface> getHands()
   {
      return rdxHands;
   }

   public RDXHandInterface getHand(RobotSide handSide)
   {
      return rdxHands.get(handSide);
   }

   public void destroy()
   {
      if (ezGripperCommunication != null)
         ezGripperCommunication.shutdown();
      if (abilityHandCommunication != null)
         abilityHandCommunication.shutdown();
   }
}
