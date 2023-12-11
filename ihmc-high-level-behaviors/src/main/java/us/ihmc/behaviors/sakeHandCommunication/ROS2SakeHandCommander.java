package us.ihmc.behaviors.sakeHandCommunication;

import controller_msgs.msg.dds.SakeHandDesiredCommandMessage;
import us.ihmc.avatar.sakeGripper.SakeHandCommandOption;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

// TODO: Use this class
public class ROS2SakeHandCommander
{
   private static ROS2SakeHandCommander commander;

   private final SideDependentList<Integer> commandNumbers = new SideDependentList<>(SakeHandCommandOption.CUSTOM.getCommandNumber(),
                                                                                    SakeHandCommandOption.CUSTOM.getCommandNumber());
   private final SideDependentList<Boolean> errorConfirmations = new SideDependentList<>(false, false);
   private final SideDependentList<Double> desiredPositions = new SideDependentList<>(-1.0, -1.0);
   private final SideDependentList<Double> desiredTorques = new SideDependentList<>(-1.0, -1.0);

   private final CommunicationHelper communicationHelper;

   private ROS2SakeHandCommander(CommunicationHelper communicationHelper)
   {
      this.communicationHelper = communicationHelper;
   }

   public static ROS2SakeHandCommander getSakeHandCommander(CommunicationHelper communicationHelper)
   {
      if (commander == null)
         commander = new ROS2SakeHandCommander(communicationHelper);

      return commander;
   }

   public void sendPredefinedCommand(RobotSide commandSide, SakeHandCommandOption commandOption)
   {
      commandNumbers.put(commandSide, commandOption.getCommandNumber());
      errorConfirmations.put(commandSide, commandOption.getErrorConfirmation());
      desiredPositions.put(commandSide, commandOption.getDesiredPosition());
      desiredTorques.put(commandSide, commandOption.getDesiredTorque());
      sendCommandMessage(commandSide);

      if (errorConfirmations.get(commandSide))
         errorConfirmations.put(commandSide, false);
   }

   public void sendErrorConfirmation(RobotSide commandSide)
   {
      errorConfirmations.put(commandSide, true);
      sendCommandMessage(commandSide);
      errorConfirmations.put(commandSide, false);
   }

   public void setDesiredPosition(RobotSide commandSide, double desiredPosition)
   {
      desiredPositions.put(commandSide, desiredPosition);
      sendCommandMessage(commandSide);
   }

   public void setDesiredTorque(RobotSide commandSide, double desiredTorque)
   {
      desiredTorques.put(commandSide, desiredTorque);
      sendCommandMessage(commandSide);
   }

   private void sendCommandMessage(RobotSide commandSide)
   {
      SakeHandDesiredCommandMessage commandMessage = new SakeHandDesiredCommandMessage();
      commandMessage.setDesiredCommandOption((byte) commandNumbers.get(commandSide).intValue());
      commandMessage.setErrorConfirmation(errorConfirmations.get(commandSide));
      commandMessage.setPositionRatio(desiredPositions.get(commandSide));
      commandMessage.setTorqueRatio(desiredTorques.get(commandSide));
      commandMessage.setRobotSide(commandSide.toByte());

      communicationHelper.publish(ROS2Tools::getSakeHandCommandTopic, commandMessage);
   }
}