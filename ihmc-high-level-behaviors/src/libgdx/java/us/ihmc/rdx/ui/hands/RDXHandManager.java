package us.ihmc.rdx.ui.hands;

import us.ihmc.psyonicros2.AbilityHandROS2HardwareCommunication;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.hands.psyonicAbilityHand.RDXAbilityHand;
import us.ihmc.rdx.ui.hands.sakeEZGripper.RDXEZGripper;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sakeros2.EZGripperROS2HardwareCommunication;

import java.util.HashMap;
import java.util.Map;

/**
 * Manages the UI for a humanoid robot's hands. A hand configuration is like "open", "closed", etc.
 */
public class RDXHandManager
{
   private final EZGripperROS2HardwareCommunication ezGripperCommunication = new EZGripperROS2HardwareCommunication(
         getClass().getSimpleName() + "EZGripperCommunication");
   private final AbilityHandROS2HardwareCommunication abilityHandCommunication = new AbilityHandROS2HardwareCommunication(
         getClass().getSimpleName() + "AbilityHandCommunication");

   private final Map<String, RDXHandInterface> rdxHands = new HashMap<>();
   private final Map<String, RDXHandQuickAccessButtons> quickAccessButtons = new HashMap<>();

   private final SideDependentList<RDXHandInterface> sideHands = new SideDependentList<>();
   private final SideDependentList<RDXEZGripper> ezGrippers = new SideDependentList<>();
   private final SideDependentList<RDXAbilityHand> abilityHands = new SideDependentList<>();

   private RDXBaseUI baseUI;

   public void create(RDXBaseUI baseUI)
   {
      this.baseUI = baseUI;
      ezGripperCommunication.start();
      abilityHandCommunication.start();
   }

   public void update()
   {
      // Update available EZGrippers
      for (RobotSide side : ezGripperCommunication.getAvailableGripperSides())
      {
         String key = side.name() + "EZGripper";
         if (!rdxHands.containsKey(key))
         {
            RDXEZGripper ezGripper = new RDXEZGripper(side, ezGripperCommunication);
            rdxHands.put(key, ezGripper);
            quickAccessButtons.put(key, new RDXHandQuickAccessButtons(baseUI, ezGripper));

            ezGrippers.put(side, ezGripper);
            sideHands.put(side, ezGripper);
            RDXBaseUI.pushNotification("Added " + key + " to " + getClass().getSimpleName());
         }
      }

      // Update available Ability hands
      for (String serialNumber : abilityHandCommunication.getAvailableHandSerialNumbers())
      {
         if (!rdxHands.containsKey(serialNumber))
         {
            RDXAbilityHand abilityHand = new RDXAbilityHand(serialNumber, abilityHandCommunication);
            rdxHands.put(serialNumber, abilityHand);
            quickAccessButtons.put(serialNumber, new RDXHandQuickAccessButtons(baseUI, abilityHand));

            RobotSide side = RobotSide.fromByte(abilityHandCommunication.readState(serialNumber).getHandSide());
            abilityHands.put(side, abilityHand);
            sideHands.put(side, abilityHand);
            RDXBaseUI.pushNotification("Added AbilityHand#" + serialNumber + " to " + getClass().getSimpleName());
         }
      }

      // Update the hands
      rdxHands.values().forEach(RDXHandInterface::update);
      quickAccessButtons.values().forEach(RDXHandQuickAccessButtons::update);
   }

   public void renderImGuiWidgets()
   {
      rdxHands.values().forEach(RDXHandInterface::renderImGuiWidgets);
   }

   public Map<String, RDXHandInterface> getHands()
   {
      return rdxHands;
   }

   public RDXHandInterface getHand(RobotSide handSide)
   {
      return sideHands.get(handSide);
   }

   public SideDependentList<RDXEZGripper> getEZGrippers()
   {
      return ezGrippers;
   }

   public SideDependentList<RDXAbilityHand> getAbilityHands()
   {
      return abilityHands;
   }

   public void destroy()
   {
      ezGripperCommunication.shutdown();
      abilityHandCommunication.shutdown();
   }
}
