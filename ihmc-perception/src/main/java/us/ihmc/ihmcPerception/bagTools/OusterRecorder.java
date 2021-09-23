package us.ihmc.ihmcPerception.bagTools;

import org.ros.message.Time;
import sensor_msgs.PointCloud2;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.RosTools;
import us.ihmc.utilities.ros.publisher.RosPointCloudPublisher;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber;
import us.ihmc.utilities.ros.types.PointType;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.concurrent.atomic.AtomicReference;

public class OusterRecorder
{
   private final String name = getClass().getSimpleName();
   private final String subscriberName = "/os_cloud_node/points";
   private final String publisherName = "/os_cloud_node2/points";
   private final RosMainNode rosNode;

   private final RosPointCloudPublisher publisher;
   private final AtomicReference<sensor_msgs.PointCloud2> incomingMessage = new AtomicReference<>();

   public OusterRecorder() throws URISyntaxException
   {
      URI rosMasterURI = new URI("http://localhost:11311");
      rosNode = RosTools.createRosNode(rosMasterURI, name);

      RosPointCloudSubscriber pointCloudSubscriber = new RosPointCloudSubscriber()
      {
         @Override
         public void onNewMessage(sensor_msgs.PointCloud2 pointCloud)
         {
            System.out.println("received message " + receiveCount++);
            incomingMessage.set(pointCloud);
         }
      };

      publisher = new RosPointCloudPublisher(PointType.XYZ, false);
      rosNode.attachSubscriber(subscriberName, pointCloudSubscriber);
      rosNode.attachPublisher(publisherName, publisher);

      rosNode.execute();

      new Thread(this::run).start();
      ThreadTools.sleepForever();
   }

   private int publishCount, receiveCount;

   private void run()
   {
      while (true)
      {
         PointCloud2 message = incomingMessage.getAndSet(null);
         if (message != null)
         {
            Time currentTime = rosNode.getCurrentTime();
            message.getHeader().setStamp(currentTime);
            publisher.publish(message);
            System.out.println("publishing message " + publishCount++);
         }

         ThreadTools.sleep(100);
      }
   }

   public static void main(String[] args) throws URISyntaxException
   {
      new OusterRecorder();
   }
}
