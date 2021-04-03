package us.ihmc.utilities.ros.subscriber;

import lidar_obstacle_detection.GDXBoxesMessage;
import us.ihmc.utilities.ros.RosMainNode;

import java.net.URI;
import java.net.URISyntaxException;

public class BoxesSubscriber extends AbstractRosTopicSubscriber<GDXBoxesMessage>
{
   private boolean DEBUG = false;

   private GDXBoxesMessage Boxes;
   private boolean BoxIsAvailable = false;

   public boolean BoxIsAvailable()
   {
      return BoxIsAvailable;
   }

   public GDXBoxesMessage getGDXBoxesMessage()
   {
      this.BoxIsAvailable = false;
      return Boxes;
   }

   public BoxesSubscriber()
   {
      super(GDXBoxesMessage._TYPE);
   }

   @Override
   public void onNewMessage(GDXBoxesMessage message)
   {
      this.Boxes = message;
      this.BoxIsAvailable = true;
      if (true)
      {
         System.out.println("Received Message:"+message.getBoxes().toString());
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
      BoxesSubscriber subscriber = new BoxesSubscriber();
      rosMainNode.attachSubscriber("/boxes", subscriber);
      rosMainNode.execute();
   }
}
