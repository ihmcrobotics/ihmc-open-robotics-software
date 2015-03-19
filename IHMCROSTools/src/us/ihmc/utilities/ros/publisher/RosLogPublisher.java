package us.ihmc.utilities.ros.publisher;

import us.ihmc.utilities.ros.RosMainNode;

public class RosLogPublisher extends RosTopicPublisher<rosgraph_msgs.Log>
{
   private final RosMainNode rosMainNode;
   private int sequenceID = 0;
   
   public RosLogPublisher(RosMainNode rosMainNode, boolean latched)
   {
      super(rosgraph_msgs.Log._TYPE, latched);
      this.rosMainNode = rosMainNode;
   }
   
   public void publish(byte level, String message)
   {
      rosgraph_msgs.Log logMessage = getMessage();

      std_msgs.Header header = logMessage.getHeader();
      header.setStamp(rosMainNode.getCurrentTime());
      header.setSeq(sequenceID++);
      logMessage.setHeader(header);
      
      logMessage.setLevel(level);
      logMessage.setMsg(message);
      
      publish(logMessage);
   }
}
