package us.ihmc.rdx.perception;

import perception_msgs.msg.dds.BigVideoPacket;
import us.ihmc.pubsub.common.SampleInfo;
import us.ihmc.pubsub.subscriber.Subscriber;

public class BigVideoSwapData
{
   private BigVideoPacket videoPacket = new BigVideoPacket();
   private SampleInfo sampleInfo = new SampleInfo();
   private boolean hasNewData = false;
   private int imageWidth = -1;
   private int imageHeight = -1;

   public void incomingDataMessage(Subscriber<BigVideoPacket> subscriber)
   {
      hasNewData = subscriber.takeNextData(videoPacket, sampleInfo);

      if (hasNewData)
      {
         imageWidth = videoPacket.getImageWidth();
         imageHeight = videoPacket.getImageHeight();
      }
   }

   public BigVideoPacket getVideoPacket()
   {
      return videoPacket;
   }
}
