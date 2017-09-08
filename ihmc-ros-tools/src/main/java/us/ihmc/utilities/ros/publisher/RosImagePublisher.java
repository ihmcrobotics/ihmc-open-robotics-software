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
   private static ChannelBufferFactory cbf = HeapChannelBufferFactory.getInstance(ByteOrder.LITTLE_ENDIAN);

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
        ChannelBuffer channelBuffer = cbf.getBuffer(bImg, 0, bImg.length);

        message.setData(channelBuffer);
        publish(message);
   }
}
