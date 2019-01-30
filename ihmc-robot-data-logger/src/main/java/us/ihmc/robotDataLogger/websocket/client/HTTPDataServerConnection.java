package us.ihmc.robotDataLogger.websocket.client;

import io.netty.bootstrap.Bootstrap;
import io.netty.channel.Channel;
import io.netty.channel.ChannelFuture;
import io.netty.channel.ChannelHandlerContext;
import io.netty.channel.ChannelInitializer;
import io.netty.channel.ChannelOption;
import io.netty.channel.ChannelPipeline;
import io.netty.channel.EventLoopGroup;
import io.netty.channel.SimpleChannelInboundHandler;
import io.netty.channel.nio.NioEventLoopGroup;
import io.netty.channel.socket.SocketChannel;
import io.netty.channel.socket.nio.NioSocketChannel;
import io.netty.handler.codec.http.DefaultFullHttpRequest;
import io.netty.handler.codec.http.HttpClientCodec;
import io.netty.handler.codec.http.HttpContent;
import io.netty.handler.codec.http.HttpContentDecompressor;
import io.netty.handler.codec.http.HttpHeaderNames;
import io.netty.handler.codec.http.HttpHeaderValues;
import io.netty.handler.codec.http.HttpMethod;
import io.netty.handler.codec.http.HttpObject;
import io.netty.handler.codec.http.HttpRequest;
import io.netty.handler.codec.http.HttpResponse;
import io.netty.handler.codec.http.HttpUtil;
import io.netty.handler.codec.http.HttpVersion;
import io.netty.handler.codec.http.LastHttpContent;
import io.netty.util.CharsetUtil;
import us.ihmc.robotDataLogger.websocket.LogHTTPPaths;

public class HTTPDataServerConnection extends SimpleChannelInboundHandler<HttpObject>
{
   private final EventLoopGroup group = new NioEventLoopGroup();
   private final String host;
   
   private volatile boolean connected = false;
   private volatile boolean connectionRefused = false;

   public HTTPDataServerConnection(String host, int port)
   {
      this.host = host;
      
      Bootstrap b = new Bootstrap();
      b.group(group).channel(NioSocketChannel.class).handler(new HttpSnoopClientInitializer());
      b.option(ChannelOption.CONNECT_TIMEOUT_MILLIS, 1000);

      ChannelFuture connectFuture = b.connect(host, port);
      connectFuture.addListener((f) -> {
         if (f.isSuccess())
         {
            connected(((ChannelFuture) f.sync()).channel());
            connected = true;
         }
         else
         {
            connectionRefused = true;
            System.err.println(f.cause().getMessage());
            group.shutdownGracefully();
         }
      });
   }

   private void connected(Channel channel)
   {

      System.out.println("Connected");
      // Prepare the HTTP request.
      HttpRequest request = new DefaultFullHttpRequest(HttpVersion.HTTP_1_1, HttpMethod.GET, LogHTTPPaths.announcement);
      request.headers().set(HttpHeaderNames.HOST, host);
      request.headers().set(HttpHeaderNames.CONNECTION, HttpHeaderValues.KEEP_ALIVE);
      request.headers().set(HttpHeaderNames.ACCEPT_ENCODING, HttpHeaderValues.GZIP);
      // Send the HTTP request.
      channel.writeAndFlush(request);
   }
   
   
   

   public boolean isConnected()
   {
      return false;

   }

   public boolean hasConnectionRefused()
   {
      return connectionRefused;

   }

   private class HttpSnoopClientInitializer extends ChannelInitializer<SocketChannel>
   {

      public HttpSnoopClientInitializer()
      {
      }

      @Override
      public void initChannel(SocketChannel ch)
      {
         ChannelPipeline p = ch.pipeline();

         p.addLast(new HttpClientCodec());
         p.addLast(new HttpContentDecompressor());
         p.addLast(HTTPDataServerConnection.this);
      }
   }

   @Override
   protected void channelRead0(ChannelHandlerContext ctx, HttpObject msg) throws Exception
   {
      if (msg instanceof HttpResponse) {
         HttpResponse response = (HttpResponse) msg;

         System.err.println("STATUS: " + response.status());
         System.err.println("VERSION: " + response.protocolVersion());
         System.err.println();

         if (!response.headers().isEmpty()) {
             for (CharSequence name: response.headers().names()) {
                 for (CharSequence value: response.headers().getAll(name)) {
                     System.err.println("HEADER: " + name + " = " + value);
                 }
             }
             System.err.println();
         }

         if (HttpUtil.isTransferEncodingChunked(response)) {
             System.err.println("CHUNKED CONTENT {");
         } else {
             System.err.println("CONTENT {");
         }
     }
     if (msg instanceof HttpContent) {
         HttpContent content = (HttpContent) msg;

         System.err.print(content.content().toString(CharsetUtil.UTF_8));
         System.err.flush();

         if (content instanceof LastHttpContent) {
             System.err.println("} END OF CONTENT");
             ctx.close();
         }
}
   }

   @Override
   public void channelInactive(ChannelHandlerContext ctx) throws Exception
   {
      System.out.println("CHANNEL CLOSED!!!");
      group.shutdownGracefully();

   }

   @Override
   public void exceptionCaught(ChannelHandlerContext ctx, Throwable cause) throws Exception
   {
      cause.printStackTrace();
      ctx.close();
   }

   public static void main(String[] args)
   {
      HTTPDataServerConnection connection = new HTTPDataServerConnection("127.0.0.1", 8008);

   }

}
