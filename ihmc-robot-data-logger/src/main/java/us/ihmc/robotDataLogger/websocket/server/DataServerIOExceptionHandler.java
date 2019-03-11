package us.ihmc.robotDataLogger.websocket.server;

import java.io.IOException;

import io.netty.channel.ChannelHandlerContext;
import io.netty.channel.ChannelInboundHandlerAdapter;

/**
 * Handler which closes the connection immediately on an IOException
 * 
 * @author Jesper Smith
 *
 */
public class DataServerIOExceptionHandler extends ChannelInboundHandlerAdapter
{
   @Override
   public void exceptionCaught(ChannelHandlerContext ctx, Throwable cause) throws Exception
   {
      if(cause instanceof IOException)
      {
         if(ctx.channel().isActive())
         {
            ctx.close(ctx.voidPromise());
         }
      }
      else
      {
         ctx.fireExceptionCaught(cause);
      }
      
   }
}
