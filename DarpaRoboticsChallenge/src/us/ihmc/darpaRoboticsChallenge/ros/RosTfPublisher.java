package us.ihmc.darpaRoboticsChallenge.ros;

import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.publisher.RosTf1Publisher;
import us.ihmc.utilities.ros.publisher.RosTf2Publisher;
import us.ihmc.utilities.ros.publisher.RosTfPublisherInterface;

public class RosTfPublisher implements RosTfPublisherInterface
{
   private final RosTfPublisherInterface tfPublisher;
   private final String tfPrefix;

   public RosTfPublisher(final RosMainNode rosMainNode, String tfPrefix)
   {
      if(tfPrefix == null)
      {
         this.tfPrefix = "";
      } 
      else
      {
         if(tfPrefix.length() > 1 && !tfPrefix.endsWith("/"))
         {
            tfPrefix = tfPrefix + "/";
         }
         this.tfPrefix = tfPrefix;
      }
      
      if (rosMainNode.isUseTf2())
      {
         tfPublisher = new RosTf2Publisher(false);
         rosMainNode.attachPublisher("/tf", (RosTf2Publisher) tfPublisher);
      }
      else
      {
         tfPublisher = new RosTf1Publisher(false);
         rosMainNode.attachPublisher("/tf", (RosTf1Publisher) tfPublisher);
      }
      System.out.println(tfPrefix);
   }

   @Override
   public void publish(RigidBodyTransform transform3d, long timeStamp,
         String parentFrame, String childFrame)
   {
      if(tfPrefix.length() > 0)
      {
         tfPublisher.publish(transform3d, timeStamp, tfPrefix + parentFrame, tfPrefix + childFrame);
      }
      else
      {
         tfPublisher.publish(transform3d, timeStamp, parentFrame, childFrame);
      }
   }
}
