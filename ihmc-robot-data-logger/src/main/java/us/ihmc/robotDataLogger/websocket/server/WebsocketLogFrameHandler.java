package us.ihmc.robotDataLogger.websocket.server;


import io.netty.channel.ChannelHandlerContext;
import io.netty.channel.SimpleChannelInboundHandler;
import io.netty.handler.codec.http.websocketx.TextWebSocketFrame;
import io.netty.handler.codec.http.websocketx.WebSocketFrame;

public class WebsocketLogFrameHandler extends SimpleChannelInboundHandler<WebSocketFrame>
{
   private final WebsocketDataBroadcaster broadcaster;

   public WebsocketLogFrameHandler(WebsocketDataBroadcaster broadcaster)
   {
      this.broadcaster = broadcaster;
   }

   @Override
   protected void channelRead0(ChannelHandlerContext ctx, WebSocketFrame frame) throws Exception
   {
      if (frame instanceof TextWebSocketFrame)
      {
         String request = ((TextWebSocketFrame) frame).text();
         System.out.println(request);
      }
      else
      {
         String message = "unsupported frame type: " + frame.getClass().getName();
         throw new UnsupportedOperationException(message);
      }
   }

   @Override
   public void channelActive(ChannelHandlerContext ctx) throws Exception
   {
      broadcaster.addClient(ctx.channel());
   }
}
