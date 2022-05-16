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
            for (int i = 0; i < data.capacity(); i++)
            {
               System.out.printf("%02x ", data.getByte(i));
            }
         }
      });
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
