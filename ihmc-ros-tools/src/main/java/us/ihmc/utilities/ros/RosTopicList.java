package us.ihmc.utilities.ros;

//import org.ros.master.client.MasterStateClient;
//import org.ros.master.client.TopicType;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.log.LogTools;

import java.util.Objects;

public class RosTopicList
{
   /**
    * This doesn't work. It's not implemented.
    * See:
    * https://github.com/rosjava/rosjava_core/blob/111c1d109ebb9089ba5dc446de80319d82580e88/rosjava/src/main/java/org/ros/master/client/MasterStateClient.java#L86
    **/
   public static void main(String[] args)
   {
      RosMainNode node = RosTools.createRosNode(NetworkParameters.getROSURI(), "topic_list");
//      MasterStateClient masterStateClient = new MasterStateClient(node.getConnectedNode(), Objects.requireNonNull(NetworkParameters.getROSURI()));
      node.execute();

      ThreadTools.sleepSeconds(1.0);
//      for (TopicType topicType : masterStateClient.getTopicTypes())
//      {
//         LogTools.info("Topic type: {}", topicType);
//      }

      ThreadTools.sleepSeconds(1.0);
   }
}
