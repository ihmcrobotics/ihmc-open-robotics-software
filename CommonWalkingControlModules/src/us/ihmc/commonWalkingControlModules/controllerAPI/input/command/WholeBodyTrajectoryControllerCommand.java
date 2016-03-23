package us.ihmc.commonWalkingControlModules.controllerAPI.input.command;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class WholeBodyTrajectoryControllerCommand implements ControllerCommand<WholeBodyTrajectoryControllerCommand, WholeBodyTrajectoryMessage>, MultipleControllerCommandHolder
{
   private final SideDependentList<HandTrajectoryControllerCommand> handTrajectoryControllerCommands = new SideDependentList<>();
   private final SideDependentList<ArmTrajectoryControllerCommand> armTrajectoryControllerCommands = new SideDependentList<>();
   private final ChestTrajectoryControllerCommand chestTrajectoryControllerCommand = new ChestTrajectoryControllerCommand();
   private final PelvisTrajectoryControllerCommand pelvisTrajectoryControllerCommand = new PelvisTrajectoryControllerCommand();
   private final SideDependentList<FootTrajectoryControllerCommand> footTrajectoryControllerCommands = new SideDependentList<>();

   private final ArrayList<ControllerCommand<?, ?>> allControllerCommands = new ArrayList<>();

   public WholeBodyTrajectoryControllerCommand()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         HandTrajectoryControllerCommand handTrajectoryControllerCommand = new HandTrajectoryControllerCommand();
         ArmTrajectoryControllerCommand armTrajectoryControllerCommand = new ArmTrajectoryControllerCommand();
         FootTrajectoryControllerCommand footTrajectoryControllerCommand = new FootTrajectoryControllerCommand();

         handTrajectoryControllerCommands.put(robotSide, handTrajectoryControllerCommand);
         armTrajectoryControllerCommands.put(robotSide, armTrajectoryControllerCommand);
         footTrajectoryControllerCommands.put(robotSide, footTrajectoryControllerCommand);

         allControllerCommands.add(handTrajectoryControllerCommand);
         allControllerCommands.add(armTrajectoryControllerCommand);
         allControllerCommands.add(footTrajectoryControllerCommand);
      }

      allControllerCommands.add(chestTrajectoryControllerCommand);
      allControllerCommands.add(pelvisTrajectoryControllerCommand);
   }

   @Override
   public void clear()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         handTrajectoryControllerCommands.get(robotSide).clear();
         armTrajectoryControllerCommands.get(robotSide).clear();
         footTrajectoryControllerCommands.get(robotSide).clear();
      }
      chestTrajectoryControllerCommand.clear();
      pelvisTrajectoryControllerCommand.clear();
   }

   @Override
   public void set(WholeBodyTrajectoryControllerCommand other)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         handTrajectoryControllerCommands.get(robotSide).set(other.handTrajectoryControllerCommands.get(robotSide));
         armTrajectoryControllerCommands.get(robotSide).set(other.armTrajectoryControllerCommands.get(robotSide));
         footTrajectoryControllerCommands.get(robotSide).set(other.footTrajectoryControllerCommands.get(robotSide));
      }
      chestTrajectoryControllerCommand.set(other.chestTrajectoryControllerCommand);
      pelvisTrajectoryControllerCommand.set(other.pelvisTrajectoryControllerCommand);
   }

   @Override
   public void set(WholeBodyTrajectoryMessage message)
   {
      clear();

      for (RobotSide robotside : RobotSide.values)
      {
         HandTrajectoryMessage handTrajectoryMessage = message.getHandTrajectoryMessage(robotside);
         if (handTrajectoryMessage.getUniqueId() != Packet.INVALID_MESSAGE_ID)
            handTrajectoryControllerCommands.get(robotside).set(handTrajectoryMessage);
         ArmTrajectoryMessage armTrajectoryMessage = message.getArmTrajectoryMessage(robotside);
         if (armTrajectoryMessage.getUniqueId() != Packet.INVALID_MESSAGE_ID)
            armTrajectoryControllerCommands.get(robotside).set(armTrajectoryMessage);
         FootTrajectoryMessage footTrajectoryMessage = message.getFootTrajectoryMessage(robotside);
         if (footTrajectoryMessage.getUniqueId() != Packet.INVALID_MESSAGE_ID)
            footTrajectoryControllerCommands.get(robotside).set(footTrajectoryMessage);
      }
      ChestTrajectoryMessage chestTrajectoryMessage = message.getChestTrajectoryMessage();
      if (chestTrajectoryMessage.getUniqueId() != Packet.INVALID_MESSAGE_ID)
         chestTrajectoryControllerCommand.set(chestTrajectoryMessage);
      PelvisTrajectoryMessage pelvisTrajectoryMessage = message.getPelvisTrajectoryMessage();
      if (pelvisTrajectoryMessage.getUniqueId() != Packet.INVALID_MESSAGE_ID)
         pelvisTrajectoryControllerCommand.set(pelvisTrajectoryMessage);
   }

   @Override
   public Class<WholeBodyTrajectoryMessage> getMessageClass()
   {
      return WholeBodyTrajectoryMessage.class;
   }

   @Override
   public List<ControllerCommand<?, ?>> getControllerCommands()
   {
      return allControllerCommands;
   }

   @Override
   public boolean isCommandValid()
   {
      return true;
   }
}
