package us.ihmc.communication.ros2;

import std_msgs.msg.dds.Empty;
import us.ihmc.commons.thread.Notification;
import us.ihmc.ros2.ROS2Callback;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.ROS2NodeInterface;

import java.util.ArrayList;
import java.util.List;

/**
 * TODO: This should become ROS2Input and ROS2Input<T> should become ROS2TypedInput
 */
public class ROS2TypelessInput
{
   private boolean hasReceivedFirstMessage = false;
   private Notification messageNotification = new Notification();
   private List<Runnable> userCallbacks = new ArrayList<>();
   private ROS2Callback<Empty> ros2Callback;

   public ROS2TypelessInput(ROS2NodeInterface ros2Node, ROS2Topic<Empty> topic)
   {
      ros2Callback = new ROS2Callback<>(ros2Node, topic, message -> messageReceivedCallback());
   }

   private void messageReceivedCallback()
   {
      messageNotification.set();
      for (Runnable userCallback : userCallbacks)
      {
         userCallback.run();
      }
      hasReceivedFirstMessage = true;
   }

   public Notification getMessageNotification()
   {
      return messageNotification;
   }

   public boolean hasReceivedFirstMessage()
   {
      return hasReceivedFirstMessage;
   }

   public void setEnabled(boolean enabled)
   {
      ros2Callback.setEnabled(enabled);
   }

   public void addCallback(Runnable messageReceivedCallback)
   {
      userCallbacks.add(messageReceivedCallback);
   }

   public void destroy()
   {
      ros2Callback.destroy();
   }
}
