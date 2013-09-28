package us.ihmc.atlas;

import org.zeromq.ZMQ;
import std_msgs.Time;
import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.RosMultisensePPSSubscriber;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class RealRobotPPSTimestampOffsetProvider implements PPSTimestampOffsetProvider
{
   private static final String MULTISENSE_SL_PPS_TOPIC = "multisense_sl/pps";
   private RosMultisensePPSSubscriber ppsSubscriber;
   private long currentTimeStampOffset = 0;
   private ZMQ.Socket requester;

   private final byte[] requestPayload = {PPSRequestType.GET_NEW_PPS_TIMESTAMP};
   private final ByteBuffer responseBuffer = ByteBuffer.allocate(8);

   public RealRobotPPSTimestampOffsetProvider()
   {
      setupZMQSocket();

      setupPPSSubscriber();
   }

   private void setupPPSSubscriber()
   {
      ppsSubscriber = new RosMultisensePPSSubscriber()
      {
         @Override
         public void onNewMessage(Time message)
         {
            currentTimeStampOffset = requestNewestRobotTimestamp() - message.getData().totalNsecs();
         }
      };
   }

   private void setupZMQSocket()
   {
      ZMQ.Context context = ZMQ.context(1);
      requester = context.socket(ZMQ.REQ);
      requester.connect("tcp://" + DRCConfigParameters.SCS_MACHINE_IP_ADDRESS + ":" + DRCConfigParameters.PPS_PROVIDER_PORT);
   }

   public void attachToRosMainNode(RosMainNode rosMainNode)
   {
      rosMainNode.attachSubscriber(MULTISENSE_SL_PPS_TOPIC, ppsSubscriber);
   }

   public long getCurrentTimestampOffset()
   {
      return currentTimeStampOffset;
   }

   public long requestNewestRobotTimestamp()
   {
      requester.send(requestPayload, 0);
      responseBuffer.put(requester.recv());
      responseBuffer.rewind();
      responseBuffer.order(ByteOrder.BIG_ENDIAN);
      return responseBuffer.getLong();
   }
}