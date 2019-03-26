package us.ihmc.communication;

import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.ros2.Ros2Node;

import java.util.concurrent.atomic.AtomicReference;

public class ROS2Input<T>
{
   private final AtomicReference<T> atomicReference;
   private final MessageFilter<T> messageFilter;

   public ROS2Input(Ros2Node ros2Node, Class<T> messageType, String robotName, String moduleName)
   {
      this(ros2Node, messageType, robotName, moduleName, message -> true);
   }

   public ROS2Input(Ros2Node ros2Node, Class<T> messageType, String robotName, String moduleName, MessageFilter<T> messageFilter)
   {
      atomicReference = new AtomicReference<>(ROS2Tools.newMessageInstance(messageType));
      this.messageFilter = messageFilter;
      ExceptionTools.handle(() -> ros2Node.createSubscription(ROS2Tools.newMessageTopicDataTypeInstance(messageType),
                                                              this::messageReceivedCallback,
                                                              ROS2Tools.generateDefaultTopicName(messageType,
                                                                                                 robotName,
                                                                                                 moduleName,
                                                                                                 ROS2Tools.ROS2TopicQualifier.OUTPUT)),
                            DefaultExceptionHandler.RUNTIME_EXCEPTION);
   }

   public interface MessageFilter<T>
   {
      boolean accept(T message);
   }

   private void messageReceivedCallback(Subscriber<T> subscriber)
   {
      T incomingData = subscriber.takeNextData();
      if (incomingData != null)
      {
         if (messageFilter.accept(incomingData))
         {
            atomicReference.set(incomingData);
         }
      }
   }

   public T getLatest()
   {
      return atomicReference.get();
   }
}
