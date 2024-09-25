package us.ihmc.perception.imageMessage;

import perception_msgs.msg.dds.ImageMessage;

public enum PixelFormat
{
   YUVI420, // YUV420 format
   BGR8,    // 24 bits per pixel, in BGR order
   BGRA8,   // 32 bits per pixel, in BGRA order
   RGB8,    // 24 bits per pixel, in RGB order
   RGBA8,   // 32 bits per pixel, in RGBA order
   GRAY8,   // monochrome
   GRAY16;  // aka depth

   public void packImageMessage(ImageMessage imageMessageToPack)
   {
      imageMessageToPack.setPixelFormat((byte) ordinal());
   }

   public static PixelFormat fromImageMessage(ImageMessage imageMessage)
   {
      return values()[imageMessage.getPixelFormat()];
   }
}
