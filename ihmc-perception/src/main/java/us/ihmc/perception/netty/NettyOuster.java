package us.ihmc.perception.netty;

import io.netty.bootstrap.Bootstrap;
import io.netty.buffer.ByteBuf;
import io.netty.channel.ChannelHandlerContext;
import io.netty.channel.EventLoopGroup;
import io.netty.channel.SimpleChannelInboundHandler;
import io.netty.channel.nio.NioEventLoopGroup;
import io.netty.channel.socket.DatagramPacket;
import io.netty.channel.socket.nio.NioDatagramChannel;
import us.ihmc.euclid.tuple3D.Point3D32;

import java.util.HashSet;

public class NettyOuster
{
   public static final int PORT = 7502;

   private EventLoopGroup group;
   private Bootstrap bootstrap;

   private HashSet<Point3D32> points;
   public NettyOuster() {
      points = new HashSet<>();
   }

   public void initialize() {
      group = new NioEventLoopGroup();
      bootstrap = new Bootstrap();
      bootstrap.group(group).channel(NioDatagramChannel.class).handler(new SimpleChannelInboundHandler<DatagramPacket>()
      {
         @Override
         protected void channelRead0(ChannelHandlerContext ctx, DatagramPacket msg)
         {
            ByteBuf data = msg.content().readBytes(msg.content().capacity());
            for (int i = 0; i < data.capacity(); i++) {
               System.out.printf("%02x ", data.getByte(i));
            }
         }
      });
   }

   public void start() {
      bootstrap.bind(PORT);
   }

   public void stop() {
      group.shutdownGracefully();
   }

   public boolean hasPoints() {
      return false; //TODO
   }
   public HashSet<Point3D32> getPoints() {
      return points; //TODO points should be updated somewhere before this method for points to actually render
   }
}
