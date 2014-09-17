package us.ihmc.darpaRoboticsChallenge.ros;

import us.ihmc.utilities.math.geometry.Transform3d;

import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.RosTf1Publisher;
import us.ihmc.utilities.ros.RosTf2Publisher;
import us.ihmc.utilities.ros.RosTfPublisherInterface;

public class RosTfPublisher implements RosTfPublisherInterface
{
   private final RosTfPublisherInterface tfPublisher;

   public RosTfPublisher(final RosMainNode rosMainNode)
   {
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
   }

   @Override
   public void publish(Transform3d transform3d, long timeStamp,
         String parentFrame, String childFrame)
   {
      tfPublisher.publish(transform3d, timeStamp, parentFrame, childFrame);
   }
}
