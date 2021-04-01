package us.ihmc.utilities.ros.subscriber;

import lidar_obstacle_detection.GDXBoxMessage;
import us.ihmc.utilities.ros.RosMainNode;

import java.net.URI;
import java.net.URISyntaxException;

public class BoxSubscriber extends AbstractRosTopicSubscriber<GDXBoxMessage>
{
   private boolean DEBUG = false;

   private GDXBoxMessage Boxes;
   private boolean BoxIsAvailable = false;

   public boolean BoxIsAvailable()
   {
      return BoxIsAvailable;
   }

   public GDXBoxMessage getGDXBoxMessage()
   {
      this.BoxIsAvailable = false;
      return Boxes;
   }

   public BoxSubscriber()
   {
      super(GDXBoxMessage._TYPE);
   }

   @Override
   public void onNewMessage(GDXBoxMessage message)
   {
      this.Boxes = message;
      this.BoxIsAvailable = true;
      if (true)
      {
         System.out.println("Received Message:" + message.getZMax());
//         for (int i = 0; i < message.getNumOfRegions(); i++)
//         {
//            List<RawGPUPlanarRegion> regions = message.getRegions();
//            System.out.println("\tRegion:" + regions.get(i).getId());
//            System.out.println("\t\tNormal:" + "X:" + regions.get(i).getNormal().getX()
//                               + "\tY:" + regions.get(i).getNormal().getY() + "\tZ:" + regions.get(i));
//            System.out.println("\t\tCentroid:" + "X:" + regions.get(i).getCentroid().getX()
//                               + "\tY:" + regions.get(i).getCentroid().getY() + "\tZ:" + regions.get(i));
//            for (int j = 0; j < regions.get(i).getVertices().size(); j++)
//            {
//               geometry_msgs.Point point = regions.get(i).getVertices().get(j);
//               System.out.println("\t\tPoint:" + "X:" + point.getX() + "\tY:" + point.getY() + "\tZ:" + point.getZ());
//            }
//         }
      }
   }

   public static void main(String[] args) throws URISyntaxException
   {
      URI rosMasterURI = new URI("http://localhost:11311/");
      RosMainNode rosMainNode = new RosMainNode(rosMasterURI, "GPUBoxSubscriber");
      BoxSubscriber subscriber = new BoxSubscriber();
      rosMainNode.attachSubscriber("/box", subscriber);
      rosMainNode.execute();
   }
}
