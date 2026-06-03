package us.ihmc.behaviors.behaviorTree.action.actions;

import ihmc_hands_ros2.AbilityHandCommand;
import ihmc_hands_ros2.AbilityHandState;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.handsros2.abilityHand.AbilityHandROS2API;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.tools.Timer;

public class AbilityHandActionComms
{
   private final TypedNotification<AbilityHandState> stateNotification = new TypedNotification<>();
   private AbilityHandState latestState = null;
   private final AbilityHandCommand command = new AbilityHandCommand();
   private final ROS2Publisher<AbilityHandCommand> commandPublisher;
   private final Timer connectedTimer = new Timer();

   public AbilityHandActionComms(RobotSide handSide, ROS2Node ros2Node)
   {
      ros2Node.createSubscription(AbilityHandROS2API.STATE_TOPICS.get(handSide), reader -> ROS2Tools.readIfPresent(reader, stateNotification::set));
      commandPublisher = ros2Node.createPublisher(AbilityHandROS2API.COMMAND_TOPICS.get(handSide));
   }

   public void update()
   {
      if (stateNotification.poll())
      {
         latestState = stateNotification.read();
         connectedTimer.reset();
      }

      if (!connectedTimer.isRunning(0.5))
         latestState = null;
   }

   public AbilityHandCommand getCommand()
   {
      return command;
   }

   public void publishCommand()
   {
      commandPublisher.publish(command);
   }

   public boolean isConnected()
   {
      return latestState != null;
   }

   public AbilityHandState getLatestState()
   {
      return latestState;
   }
}
