package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.controllerAPI.command.MultipleCommandHolder;
import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class WholeBodyTrajectoryCommand implements MultipleCommandHolder<WholeBodyTrajectoryCommand, WholeBodyTrajectoryMessage>
{
   private final SideDependentList<HandTrajectoryCommand> handTrajectoryControllerCommands = new SideDependentList<>();
   private final SideDependentList<ArmTrajectoryCommand> armTrajectoryControllerCommands = new SideDependentList<>();
   private final ChestTrajectoryCommand chestTrajectoryControllerCommand = new ChestTrajectoryCommand();
   private final PelvisTrajectoryCommand pelvisTrajectoryControllerCommand = new PelvisTrajectoryCommand();
   private final SideDependentList<FootTrajectoryCommand> footTrajectoryControllerCommands = new SideDependentList<>();

   private final ArrayList<Command<?, ?>> allControllerCommands = new ArrayList<>();
   
   /** the time to delay this command on the controller side before being executed **/
   private double executionDelayTime;
   /** the execution time. This number is set if the execution delay is non zero**/
   public double adjustedExecutionTime;

   public WholeBodyTrajectoryCommand()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         HandTrajectoryCommand handTrajectoryControllerCommand = new HandTrajectoryCommand();
         ArmTrajectoryCommand armTrajectoryControllerCommand = new ArmTrajectoryCommand();
         FootTrajectoryCommand footTrajectoryControllerCommand = new FootTrajectoryCommand();

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
   public void set(WholeBodyTrajectoryCommand other)
   {
      executionDelayTime = other.getExecutionDelayTime();
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

      executionDelayTime = message.executionDelayTime;
      
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
   public void parseCommandIdToChildren()
   {
      // TODO implement commandId.
   }

   @Override
   public List<Command<?, ?>> getControllerCommands()
   {
      return allControllerCommands;
   }

   @Override
   public boolean isCommandValid()
   {
      return true;
   }
   
   /**
    * returns the amount of time this command is delayed on the controller side before executing
    * @return the time to delay this command in seconds
    */
   @Override
   public double getExecutionDelayTime()
   {
      return executionDelayTime;
   }
   
   /**
    * sets the amount of time this command is delayed on the controller side before executing
    * @param delayTime the time in seconds to delay after receiving the command before executing
    */
   @Override
   public void setExecutionDelayTime(double delayTime)
   {
      this.executionDelayTime = delayTime;
   }

   /**
    * returns the expected execution time of this command. The execution time will be computed when the controller 
    * receives the command using the controllers time plus the execution delay time.
    * This is used when {@code getExecutionDelayTime} is non-zero
    */
   @Override
   public double getExecutionTime()
   {
      return adjustedExecutionTime;
   }

   /**
    * sets the execution time for this command. This is called by the controller when the command is received.
    */
   @Override
   public void setExecutionTime(double adjustedExecutionTime)
   {
      this.adjustedExecutionTime = adjustedExecutionTime;
   }
   
   /**
    * tells the controller if this command supports delayed execution
    * (Spoiler alert: It does)
    * @return
    */
   @Override
   public boolean isDelayedExecutionSupported()
   {
      return true;
   }
}
