package us.ihmc.utilities.ros;

import org.ros.internal.message.Message;

import us.ihmc.utilities.ros.publisher.RosTopicPublisher;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

public class RosTopicEchoer<T extends Message> extends AbstractRosTopicSubscriber<T>
{
   private final EchoPublisher<T> echoPublisher;
   
   public RosTopicEchoer(RosMainNode rosMainNode, String messageType, String sourceTopic, String destinationTopic)
   {
      super(messageType);
      
      echoPublisher = new EchoPublisher<T>(messageType, true);
      rosMainNode.attachSubscriber(sourceTopic, this);
      rosMainNode.attachPublisher(destinationTopic, echoPublisher);
   }

   @Override
   public void onNewMessage(T message)
   {
      echoPublisher.echoMessage(message);
   }
   
   private class EchoPublisher<U extends Message> extends RosTopicPublisher<U>
   {

      public EchoPublisher(String messageType, boolean latched)
      {
         super(messageType, latched);
      }
      
      public void echoMessage(U message)
      {
         publish(message);
      }
   }
}
