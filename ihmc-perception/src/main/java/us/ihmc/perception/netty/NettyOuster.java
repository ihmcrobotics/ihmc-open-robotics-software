package us.ihmc.perception.netty;

import io.netty.bootstrap.Bootstrap;
import io.netty.buffer.ByteBuf;
import io.netty.channel.ChannelHandlerContext;
import io.netty.channel.EventLoopGroup;
import io.netty.channel.SimpleChannelInboundHandler;
import io.netty.channel.nio.NioEventLoopGroup;
import io.netty.channel.socket.DatagramPacket;
import io.netty.channel.socket.nio.NioDatagramChannel;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.perception.BytedecoImage;

import java.nio.ByteOrder;
import java.util.HashSet;

/**
 * Ouster Firmware User Manual: https://data.ouster.io/downloads/software-user-manual/firmware-user-manual-v2.3.0.pdf
 *
 * To test, use the GNU netcat command:
 * netcat -ul 7502
 */
public class NettyOuster
{
   public static final int PORT = 7502;

   private EventLoopGroup group;
   private Bootstrap bootstrap;
   private final Mat data;
   private final BytedecoImage image;

   public NettyOuster()
   {
      data = new Mat(64, 1024, opencv_core.CV_32FC1); //TODO rows/cols should be determined by querying REST API of ouster instead
      image = new BytedecoImage(data);
   }

   public void initialize()
   {
      group = new NioEventLoopGroup();
      bootstrap = new Bootstrap();
      bootstrap.group(group).channel(NioDatagramChannel.class).handler(new SimpleChannelInboundHandler<DatagramPacket>()
      {
         @Override
         protected void channelRead0(ChannelHandlerContext ctx, DatagramPacket msg)
         {
            //TODO buffer can only contain 2048 bytes right now, which needs to be at least 12608 to actually store lidar packet (we are missing lots of data)
            ByteBuf bufferedData = msg.content().readBytes(msg.content().capacity());
            bufferedData = bufferedData.order(ByteOrder.LITTLE_ENDIAN); //Ouster is little endian

            int i = 0;
            while (i + 788 < msg.content().capacity()) //SHOULD be 16 reads (but is actually 2 right now)
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

               if (dataOkay) {
                  for (int k = 0; k < 64; k++)
                     data.ptr(k, measurementID).putFloat(range[k] / 1000.0F);
               }
            }

            bufferedData.release();
         }
      });
   }

   /**
    * Java doesn't have support for unsigned 32-bit integers* so we have to use a long instead
    */
   private long extractValue(ByteBuf data, int index, int num) {
      int shift = index % 4;
      int modIndex = index - (index % 4);
      long val = data.getUnsignedInt(modIndex);
      val >>= shift;
      switch(num) {
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

   public BytedecoImage getBytedecoImage() {
      return image;
   }
}
