package us.ihmc.perception.netty;

import io.netty.bootstrap.Bootstrap;
import io.netty.buffer.ByteBuf;
import io.netty.channel.ChannelHandlerContext;
import io.netty.channel.EventLoopGroup;
import io.netty.channel.SimpleChannelInboundHandler;
import io.netty.channel.nio.NioEventLoopGroup;
import io.netty.channel.socket.DatagramPacket;
import io.netty.channel.socket.nio.NioDatagramChannel;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.tuple3D.Point3D32;

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
   private HashSet<Point3D32> points = new HashSet<>();
   private long time = System.nanoTime();

   public NettyOuster()
   {

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
            ByteBuf data = msg.content().readBytes(msg.content().capacity());
            data = data.order(ByteOrder.LITTLE_ENDIAN);
            int i = 0;
            System.out.print("Timestamp: ");
            i = printBytes(data, i, 8);
            System.out.print("Frame ID: ");
            i = printBytes(data, i, 2);
            System.out.print("Measurement ID: ");
            i = printBytes(data, i, 2);
            System.out.print("Encoder Count: ");
            i = printBytes(data, i, 4);
            System.out.println();

            for (int j = 0; j < 64; j++) {
               System.out.println("Block " + j + ":");
               System.out.print("Range (mm): ");
               i = printBytes(data, i, 4);
               System.out.print("Signal Photons: ");
               i = printBytes(data, i, 2);
               i++;
               System.out.print("Reflectivity: ");
               i = printBytes(data, i, 1);
               i+=2;
               System.out.print("NIR Photons: ");
               i = printBytes(data, i, 2);
               System.out.println();
            }

            System.out.println();
            i = printBytes(data, i, 4);

            System.out.println("\n\n");

            System.out.print("Timestamp: ");
            i = printBytes(data, i, 8);
            System.out.print("Frame ID: ");
            i = printBytes(data, i, 2);
            System.out.print("Measurement ID: ");
            i = printBytes(data, i, 2);
            System.out.print("Encoder Count: ");
            i = printBytes(data, i, 4);
            System.out.println();

            for (int j = 0; j < 64; j++) {
               System.out.println("Block " + j + ":");
               System.out.print("Range (mm): ");
               i = printBytes(data, i, 4);
               System.out.print("Signal Photons: ");
               i = printBytes(data, i, 2);
               i++;
               System.out.print("Reflectivity: ");
               i = printBytes(data, i, 1);
               i+=2;
               System.out.print("NIR Photons: ");
               i = printBytes(data, i, 2);
               System.out.println();
            }

            System.out.println();
            printBytes(data, i, 4);

            System.out.println("\n\n");
         }
      });
   }

   private int printBytes(ByteBuf data, int index, int num) {
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
      }

      System.out.println(val == 4294967295L ? "FFFF" : val);

      return index + num;
   }

   public void start()
   {
      bootstrap.bind(PORT);
   }

   public void stop()
   {
      group.shutdownGracefully();
   }

   public boolean hasPoints()
   {
      return false; //TODO
   }

   public HashSet<Point3D32> getPoints()
   {
      return points; //TODO points should be updated somewhere before this method for points to actually render
   }

   public static void main(String[] args)
   {
      NettyOuster nettyOuster = new NettyOuster();
      nettyOuster.initialize();
      nettyOuster.start();

      Runtime.getRuntime().addShutdownHook(new Thread(nettyOuster::stop));

      ThreadTools.sleepForever();
   }
}
