package us.ihmc.utilities.ros.publisher;

import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.nio.ByteOrder;

import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBufferFactory;
import org.jboss.netty.buffer.HeapChannelBufferFactory;
import org.ros.message.Time;

import sensor_msgs.Image;
import std_msgs.Header;


public class RosImagePublisher extends RosTopicPublisher<Image>
{
   private int seq = 0;
   private static ChannelBufferFactory channelBufferFactory = HeapChannelBufferFactory.getInstance(ByteOrder.LITTLE_ENDIAN);

   public RosImagePublisher()
   {
       super(Image._TYPE, false);
   }

   @Override
   public void connected()
   {
   }

   public void publish(String frameID, BufferedImage img, Time t)
   {
        Image message = getMessage();
        Header header = message.getHeader();

        header.setStamp(t);
        header.setFrameId(frameID);
        header.setSeq(seq++);
        message.setHeader(header);

        message.setHeight(img.getHeight());
        message.setWidth(img.getWidth());
        message.setEncoding("bgr8");
        message.setIsBigendian((byte)0);
        message.setStep(3*img.getWidth());

        String[] props = img.getPropertyNames();
        /*if(props != null)
        {
            for (int i=0; i<props.length; i++)
            {
               System.out.println("prop name: " + props[i]);
            }
        }*/
        DataBufferByte dataBufferByte = (DataBufferByte)img.getData().getDataBuffer();
        byte[] bImg = dataBufferByte.getData();
        ChannelBuffer channelBuffer = channelBufferFactory.getBuffer(bImg, 0, bImg.length);

        message.setData(channelBuffer);
        publish(message);
   }

   public Image createMessage(int width, int height, int bytesPerValue, String encoding, ChannelBuffer channelBuffer)
   {
      Image message = getMessage();
      Header header = message.getHeader();

      header.setSeq(seq++);

      message.setIsBigendian((byte) 0);
      message.setStep(width * bytesPerValue);
      message.setHeight(height);
      message.setWidth(width);
      message.setEncoding(encoding);

      message.setData(channelBuffer);
      return message;
   }

   public void publish(Image message)
   {
      super.publish(message);
   }

   public static int floatTo16BitInt(float fval)
   {
      int fbits = Float.floatToIntBits(fval);
      int sign = fbits >>> 16 & 0x8000;          // sign only
      int val = (fbits & 0x7fffffff) + 0x1000; // rounded value

      if (val >= 0x47800000)               // might be or become NaN/Inf
      {                                     // avoid Inf due to rounding
         if ((fbits & 0x7fffffff) >= 0x47800000)
         {                                 // is or must become NaN/Inf
            if (val < 0x7f800000)        // was value but too large
               return sign | 0x7c00;     // make it +/-Inf
            return sign | 0x7c00 |        // remains +/-Inf or NaN
                   (fbits & 0x007fffff) >>> 13; // keep NaN (and Inf) bits
         }
         return sign | 0x7bff;             // unrounded not quite Inf
      }
      if (val >= 0x38800000)               // remains normalized value
         return sign | val - 0x38000000 >>> 13; // exp - 127 + 15
      if (val < 0x33000000)                // too small for subnormal
         return sign;                      // becomes +/-0
      val = (fbits & 0x7fffffff) >>> 23;  // tmp exp for subnormal calc
      return sign | ((fbits & 0x7fffff | 0x800000) // add subnormal bit
                     + (0x800000 >>> val - 102)     // round depending on cut off
                     >>> 126 - val);   // div by 2^(1-(exp-127+15)) and >> 13 | exp=0
   }

   public ChannelBufferFactory getChannelBufferFactory()
   {
      return channelBufferFactory;
   }
}
