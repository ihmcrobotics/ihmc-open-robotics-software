package us.ihmc.communication;

import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.ros2.Ros2Node;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

public class ROS2Callback<T>
{
   private final AtomicReference<T> atomicReference;
   private final Consumer<T> messageCallback;

   public ROS2Callback(Ros2Node ros2Node, Class<T> messageType, String robotName, String moduleName, Consumer<T> messageCallback)
   {
      atomicReference = new AtomicReference<>(ROS2Tools.newMessageInstance(messageType));
      this.messageCallback = messageCallback;
      ExceptionTools.handle(() -> ros2Node.createSubscription(ROS2Tools.newMessageTopicDataTypeInstance(messageType),
                                                              this::nullOmissionCallback,
                                                              ROS2Tools.generateDefaultTopicName(messageType,
                                                                                                 robotName,
                                                                                                 moduleName,
                                                                                                 ROS2Tools.ROS2TopicQualifier.OUTPUT)),
                            DefaultExceptionHandler.RUNTIME_EXCEPTION);
   }

   private void nullOmissionCallback(Subscriber<T> subscriber)
   {
      T incomingData = subscriber.takeNextData();
      if (incomingData != null)
      {
         messageCallback.accept(incomingData);
      }
      else
      {
         LogTools.warn("Received null from takeNextData()");
      }
   }

   public T getLatest()
   {
      return atomicReference.get();
   }
}
