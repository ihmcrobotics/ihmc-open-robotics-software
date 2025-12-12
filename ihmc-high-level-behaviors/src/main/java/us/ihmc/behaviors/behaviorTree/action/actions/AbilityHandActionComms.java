package us.ihmc.behaviors.behaviorTree.action.actions;

import ihmc_hands_ros2.msg.dds.AbilityHandCommand;
import ihmc_hands_ros2.msg.dds.AbilityHandState;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.handsros2.abilityHand.AbilityHandROS2API;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.tools.Timer;

public class AbilityHandActionComms
{
   private final RobotSide handSide;
   private final TypedNotification<AbilityHandState> stateNotification = new TypedNotification<>();
   private AbilityHandState latestState = null;
   private final AbilityHandCommand command = new AbilityHandCommand();
   private final ROS2Publisher<AbilityHandCommand> commandPublisher;
   private final Throttler commandThrottler = new Throttler().setFrequency(30.0);
   private final Timer connectedTimer = new Timer();

   public AbilityHandActionComms(RobotSide handSide, ROS2Node ros2Node)
   {
      this.handSide = handSide;

      ros2Node.createSubscription2(AbilityHandROS2API.STATE_TOPIC, stateNotification::set);
      commandPublisher = ros2Node.createPublisher(AbilityHandROS2API.COMMAND_TOPIC);
   }

   public void update()
   {
      if (stateNotification.poll())
      {
         AbilityHandState read = stateNotification.read();
         if (read.getHandSide() == handSide.toByte())
         {
            latestState = read;
            command.setIdentifier(latestState.getIdentifierAsString());
            connectedTimer.reset();
         }
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
