package us.ihmc.utilities.ros;

import org.ros.internal.message.Message;
import org.ros.message.Time;
import org.ros.node.parameter.ParameterListener;
import us.ihmc.utilities.ros.publisher.RosTopicPublisher;
import us.ihmc.utilities.ros.subscriber.RosTopicSubscriberInterface;

import java.util.function.Consumer;

public interface RosNodeInterface
{
   boolean isStarted();

   void attachServiceClient(String topicName, RosServiceClient<? extends Message, ? extends Message> client);

   void attachServiceServer(String topicName, RosServiceServer<? extends Message, ? extends Message> server);

   void attachPublisher(String topicName, RosTopicPublisher<? extends Message> publisher);

   void attachSubscriber(String topicName, RosTopicSubscriberInterface<? extends Message> subscriber);

   <T extends Message> void attachSubscriber(String topicName, Class<T> type, Consumer<T> callback);

   void removeSubscriber(RosTopicSubscriberInterface<? extends Message> subscriber);

   void attachParameterListener(String topicName, ParameterListener listener);

   Time getCurrentTime();
}
