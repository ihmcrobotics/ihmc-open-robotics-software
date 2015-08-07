package us.ihmc.utilities.ros;

import static org.junit.Assert.assertTrue;

import java.net.URISyntaxException;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;

import org.junit.Test;

import std_msgs.String;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.tools.agileTesting.BambooAnnotations.BambooPlan;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.tools.agileTesting.BambooPlanType;
import us.ihmc.utilities.ros.publisher.RosStringPublisher;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;
import us.ihmc.utilities.ros.subscriber.RosTopicSubscriberInterface;

@BambooPlan(planType=BambooPlanType.Flaky)
public class RosTopicPublisherSubscriberTest extends IHMCRosTestWithRosCore
{
	@EstimatedDuration
	@Test(timeout=2000)
   public void testPubSubMultipleMessages() throws URISyntaxException, InterruptedException
   {
      int nPacket=10;
      final CountDownLatch latch =new CountDownLatch(nPacket);

      RosMainNode rosMainNode = new RosMainNode(rosMasterURI, "topicClientTestNode");
      final RosStringPublisher publisher = new RosStringPublisher(false);
      rosMainNode.attachPublisher("/chatter", publisher);

      RosTopicSubscriberInterface<std_msgs.String> subscriber = new AbstractRosTopicSubscriber<std_msgs.String>(std_msgs.String._TYPE)
      {
         @Override
         public void onNewMessage(String message)
         {
            latch.countDown();
         }

      };

      rosMainNode.attachSubscriber("/chatter", subscriber);
      rosMainNode.execute();

      subscriber.wailTillRegistered();
      publisher.waitTillRegistered();
      
      Thread publisherThread =new Thread()
      {
         public void run(){
              for(int seq=0; true;seq++)
              {
                 ThreadTools.sleep(1);
                 publisher.publish("hello "+seq);
              }
         }
      };
      publisherThread.start();

      assertTrue(latch.await(1, TimeUnit.SECONDS));
   }
}
