package us.ihmc.rdx.perception;

import perception_msgs.BigVideoPacket;

/** Legacy pub/sub swap buffer — stubbed for jros2 migration (no current test references). */
public class BigVideoSwapData
{
   private final BigVideoPacket videoPacket = new BigVideoPacket();
   private boolean hasNewData = false;
   private int imageWidth = -1;
   private int imageHeight = -1;

   public BigVideoPacket getVideoPacket()
   {
      return videoPacket;
   }

   public boolean hasNewData()
   {
      return hasNewData;
   }

   public int getImageWidth()
   {
      return imageWidth;
   }

   public int getImageHeight()
   {
      return imageHeight;
   }
}
