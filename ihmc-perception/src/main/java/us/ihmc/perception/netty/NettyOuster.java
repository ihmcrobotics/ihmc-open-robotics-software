package us.ihmc.perception.netty;

import io.netty.bootstrap.Bootstrap;
import io.netty.buffer.ByteBuf;
import io.netty.channel.*;
import io.netty.channel.nio.NioEventLoopGroup;
import io.netty.channel.socket.DatagramPacket;
import io.netty.channel.socket.nio.NioDatagramChannel;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.BytedecoOpenCVTools;

import java.nio.ByteOrder;

/**
 * Ouster Firmware User Manual: https://data.ouster.io/downloads/software-user-manual/firmware-user-manual-v2.3.0.pdf
 *
 * To test, use the GNU netcat command:
 * netcat -ul 7502
 */
public class NettyOuster
{
   public static final int PORT = 7502;

   private static final int HEIGHT = 64;
   private static final int WIDTH = 1024;
   private static final int LIDAR_DATA_PACKET_SIZE = 12608;
   private static final int MEASUREMENT_BLOCK_SIZE = 788;

   private EventLoopGroup group;
   private Bootstrap bootstrap;
   private final BytedecoImage image;

   public NettyOuster()
   {
      //TODO rows/cols should be determined by querying REST API of ouster instead
      image = new BytedecoImage(WIDTH, HEIGHT, opencv_core.CV_32FC1);
      image.getBytedecoOpenCVMat().setTo(new Mat(0.0f)); // should initialize it to 0
   }

   public void initialize()
   {
      group = new NioEventLoopGroup();
      bootstrap = new Bootstrap();
      bootstrap.group(group).channel(NioDatagramChannel.class).handler(new SimpleChannelInboundHandler<DatagramPacket>()
      {
         @Override
         protected void channelRead0(ChannelHandlerContext context, DatagramPacket packet)
         {
            //TODO buffer can only contain 2048 bytes right now, which needs to be at least 12608 to actually store lidar packet (we are missing lots of data)
            ByteBuf bufferedData = packet.content().readBytes(packet.content().capacity());
            bufferedData = bufferedData.order(ByteOrder.LITTLE_ENDIAN); //Ouster is little endian

            int i = 0;
            while (i + MEASUREMENT_BLOCK_SIZE <= packet.content().capacity()) //TODO you guessed it, REST API
            {
               i += 8; //Timestamp
               int measurementID = (int) extractValue(bufferedData, i, 2);
               i += 8; //Measurement ID (above), Frame ID, Encoder Count

               long[] range = new long[64];
               for (int blockID = 0; blockID < 64; blockID++)
               { //Note that blockID is useful data here
                  range[blockID] = extractValue(bufferedData, i, 4);
                  i += 12; //Range, and other values we don't care about
               }

               boolean dataOkay = extractValue(bufferedData, i, 4) == 0xFFFFFFFFL;
               i += 4;

               if (dataOkay)
               {
                  for (int k = 0; k < 64; k++)
                  {
                     float rangeScaled = range[k] / 1000.0F;
                     if (rangeScaled > 30.0)
                     {
                        rangeScaled = 0.0f;
                     }
                     image.getBytedecoOpenCVMat().ptr(k, measurementID).putFloat(rangeScaled);
                  }
               }
            }

            bufferedData.release();
         }
      });

      bootstrap.option(ChannelOption.RCVBUF_ALLOCATOR,
                       new FixedRecvByteBufAllocator(LIDAR_DATA_PACKET_SIZE)); //TODO this needs to also be done with the REST API
   }

   /**
    * Java doesn't have support for unsigned 32-bit integers* so we have to use a long instead
    */
   private long extractValue(ByteBuf data, int index, int num)
   {
      int shift = index % 4;
      int modIndex = index - (index % 4);
      long val = data.getUnsignedInt(modIndex);
      val >>= shift;
      switch (num)
      {
         case 1:
            val = val & 0xFF;
            break;
         case 2:
            val = val & 0xFFFF;
            break;
         case 4:
            break;
         case 8:
            val += data.getUnsignedInt(modIndex + 4) << 32;
            break;
         default:
            return -1;
      }

      return val;
   }

   public void start()
   {
      bootstrap.bind(PORT);
   }

   public void stop()
   {
      group.shutdownGracefully();
   }

   public int getImageWidth()
   {
      return WIDTH;
   }

   public int getImageHeight()
   {
      return HEIGHT;
   }

   public BytedecoImage getBytedecoImage()
   {
      return image;
   }
}
