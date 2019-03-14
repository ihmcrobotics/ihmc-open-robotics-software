package us.ihmc.robotDataLogger.websocket.server;

import static io.netty.handler.codec.http.HttpMethod.GET;
import static io.netty.handler.codec.http.HttpResponseStatus.BAD_REQUEST;
import static io.netty.handler.codec.http.HttpResponseStatus.FORBIDDEN;
import static io.netty.handler.codec.http.HttpResponseStatus.NOT_FOUND;
import static io.netty.handler.codec.http.HttpResponseStatus.OK;
import static io.netty.handler.codec.http.HttpVersion.HTTP_1_1;

import io.netty.buffer.ByteBuf;
import io.netty.buffer.Unpooled;
import io.netty.channel.ChannelFuture;
import io.netty.channel.ChannelFutureListener;
import io.netty.channel.ChannelHandlerContext;
import io.netty.channel.SimpleChannelInboundHandler;
import io.netty.handler.codec.http.DefaultFullHttpResponse;
import io.netty.handler.codec.http.FullHttpRequest;
import io.netty.handler.codec.http.FullHttpResponse;
import io.netty.handler.codec.http.HttpHeaderNames;
import io.netty.handler.codec.http.HttpUtil;
import io.netty.util.CharsetUtil;
import us.ihmc.robotDataLogger.websocket.HTTPDataServerPaths;

/**
 * 
 * Implementation of the HTTP server providing static resources
 * 
 * @author Jesper Smith
 *
 */
class HTTPDataServerDescriptionServer extends SimpleChannelInboundHandler<FullHttpRequest>
{

   private final DataServerServerContent logServerContent;

   public HTTPDataServerDescriptionServer(DataServerServerContent logServerContent)
   {
      this.logServerContent = logServerContent;
   }

   @Override
   protected void channelRead0(ChannelHandlerContext ctx, FullHttpRequest req) throws Exception
   {
      // Handle a bad request.
      if (!req.decoderResult().isSuccess())
      {
         sendHttpResponse(ctx, req, new DefaultFullHttpResponse(HTTP_1_1, BAD_REQUEST));
         return;
      }

      // Allow only GET methods.
      if (req.method() != GET)
      {
         sendHttpResponse(ctx, req, new DefaultFullHttpResponse(HTTP_1_1, FORBIDDEN));
         return;
      }

      // Send the index page
      if ("/".equals(req.uri()) || HTTPDataServerPaths.index.equals(req.uri()))
      {
         sendContent(ctx, req, logServerContent.getIndex(), logServerContent.getIndexContentType());
      }
      else if (HTTPDataServerPaths.announcement.equals(req.uri()))
      {
         sendContent(ctx, req, logServerContent.getAnnouncement(), logServerContent.getAnnouncementContentType());
      }
      else if (HTTPDataServerPaths.handshake.equals(req.uri()))
      {
         sendContent(ctx, req, logServerContent.getHandshake(), logServerContent.getHandshakeContentType());
      }
      else if (logServerContent.hasModel() && HTTPDataServerPaths.model.equals(req.uri()))
      {
         sendContent(ctx, req, logServerContent.getModel(), logServerContent.getModelContentType());
      }
      else if (logServerContent.hasResourceZip() && HTTPDataServerPaths.resources.equals(req.uri()))
      {
         sendContent(ctx, req, logServerContent.getResourceZip(), logServerContent.getResourceZipContentType());
      }         
      else
      {
         sendHttpResponse(ctx, req, new DefaultFullHttpResponse(HTTP_1_1, NOT_FOUND));
      }
   }

   private static void sendContent(ChannelHandlerContext ctx, FullHttpRequest req, ByteBuf content, String contentType)
   {
      FullHttpResponse res = new DefaultFullHttpResponse(HTTP_1_1, OK, content);

      res.headers().set(HttpHeaderNames.CONTENT_TYPE, contentType);
      HttpUtil.setContentLength(res, content.readableBytes());
      sendHttpResponse(ctx, req, res);

   }

   private static void sendHttpResponse(ChannelHandlerContext ctx, FullHttpRequest req, FullHttpResponse res)
   {
      // Generate an error page if response getStatus code is not OK (200).
      if (res.status().code() != 200)
      {
         ByteBuf buf = Unpooled.copiedBuffer(res.status().toString(), CharsetUtil.UTF_8);
         res.content().writeBytes(buf);
         buf.release();
         HttpUtil.setContentLength(res, res.content().readableBytes());
      }

      // Send the response and close the connection if necessary.
      ChannelFuture f = ctx.channel().writeAndFlush(res);
      if (!HttpUtil.isKeepAlive(req) || res.status().code() != 200)
      {
         f.addListener(ChannelFutureListener.CLOSE);
      }
   }

}