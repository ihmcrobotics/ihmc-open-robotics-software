package us.ihmc.communication;

import geometry_msgs.msg.dds.PosePubSubType;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.log.LogTools;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.ROS2TopicNameTools;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.ros2.rosidl.geometry_msgs.msg.dds.Pose3DPubSubTypeImpl;

public class IHMCROS2Publisher<T>
{
   private static final int NUMBER_OF_EXCEPTIONS_TO_PRINT = 5;

   private ROS2PublisherBasics<T> publisher;

   private int numberOfExceptions = 0;

   private IHMCROS2Publisher()
   {
      // internal use only
   }

   IHMCROS2Publisher(ROS2PublisherBasics<T> ros2Publisher)
   {
      this.publisher = ros2Publisher;
   }

   @Deprecated
   public IHMCROS2Publisher(ROS2NodeInterface ros2Node, Class<T> messageType)
   {
      this(ros2Node, messageType, ROS2Tools.IHMC_ROOT);
   }

   public IHMCROS2Publisher(ROS2NodeInterface ros2Node, ROS2Topic<T> topicName)
   {
      this(ros2Node, topicName.getType(), topicName.getName());
   }

   public IHMCROS2Publisher(ROS2NodeInterface ros2Node, Class<T> messageType, ROS2Topic topicName)
   {
      this(ros2Node, messageType, topicName.withTypeName(messageType).toString());
   }

   public IHMCROS2Publisher(ROS2NodeInterface ros2Node, Class<T> messageType, String topicName)
   {
      ExceptionTools.handle(() -> publisher = ros2Node.createPublisher(ROS2TopicNameTools.newMessageTopicDataTypeInstance(messageType), topicName),
                            DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
   }

   public static IHMCROS2Publisher<Pose3D> newPose3DPublisher(ROS2NodeInterface ros2Node, ROS2Topic topicName)
   {
      PosePubSubType.setImplementation(new Pose3DPubSubTypeImpl());
      IHMCROS2Publisher<Pose3D> ihmcROS2Publisher = new IHMCROS2Publisher<>();
      ExceptionTools.handle(() -> ihmcROS2Publisher.publisher = ros2Node.createPublisher(new PosePubSubType(), topicName.getName()),
                            DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
      return ihmcROS2Publisher;
   }

   public synchronized void publish(T message)
   {
      try
      {
         if (publisher != null)
         {
            publisher.publish(message);
         }
         else
         {
            LogTools.warn(1, "Publisher already removed! Cannot publish " + message.getClass().getSimpleName());
         }
      }
      catch (Exception e)
      {
         if (numberOfExceptions <= NUMBER_OF_EXCEPTIONS_TO_PRINT)
         {
            e.printStackTrace();

            if (++numberOfExceptions > NUMBER_OF_EXCEPTIONS_TO_PRINT)
            {
               LogTools.error("Stopping to print exceptions after {}.", NUMBER_OF_EXCEPTIONS_TO_PRINT);
            }
         }
      }
   }

   public void destroy()
   {
      publisher.remove();
   }
}
