package us.ihmc.gr00t;

import io.netty.bootstrap.Bootstrap;
import io.netty.channel.Channel;
import io.netty.channel.ChannelInitializer;
import io.netty.channel.ChannelPipeline;
import io.netty.channel.EventLoopGroup;
import io.netty.channel.nio.NioEventLoopGroup;
import io.netty.channel.socket.SocketChannel;
import io.netty.channel.socket.nio.NioSocketChannel;
import io.netty.handler.codec.http.HttpClientCodec;
import io.netty.handler.codec.http.HttpObjectAggregator;
import org.msgpack.core.MessageBufferPacker;
import org.msgpack.core.MessagePack;
import org.msgpack.core.MessageUnpacker;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.log.LogTools;
import us.ihmc.openpi.OpenpiNettyWebSocketHandler;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.net.URI;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.TimeUnit;

/**
 * Client for a GR00T inference server speaking the same websocket+msgpack protocol as
 * {@link us.ihmc.openpi.OpenpiClient} (see {@code gr00t_openpi_bridge_server.py}, which forwards
 * requests to an unmodified {@code run_gr00t_server.py} over
 * {@code gr00t.policy.server_client.PolicyClient}). Unlike {@code OpenpiClient}, the port, image
 * dimensions, action chunk length, and prompt are configurable rather than fixed to openpi's
 * defaults, since GR00T's vision backbone/server may expect different values (see plan Open Item
 * #2/#4: confirm these against the actual bridge server before deploying).
 */
public class Gr00tClient
{
   private final String host;
   private final int port;
   private final int stateSize;
   private final int chunkLength;
   private final int imageWidth;
   private final int imageHeight;
   private String prompt = "";

   private EventLoopGroup group;
   private Channel channel;
   private OpenpiNettyWebSocketHandler handler;
   private final MessageBufferPacker packer = MessagePack.newDefaultBufferPacker();
   private final ByteBuffer state;
   private final SideDependentList<ByteBuffer> images;
   private final ByteBuffer actions;
   private float policyTimingMs;
   private float serverTimingMs;
   /** Number of leading steps in {@link #actions} that are genuine model predictions rather than the server's held/repeated tail; see {@link #unpack}. */
   private int horizon;

   public Gr00tClient(String host, int port, int stateSize, int chunkLength, int imageWidth, int imageHeight)
   {
      this.host = host;
      this.port = port;
      this.stateSize = stateSize;
      this.chunkLength = chunkLength;
      this.imageWidth = imageWidth;
      this.imageHeight = imageHeight;

      state = ByteBuffer.allocate(stateSize * Float.BYTES);
      state.order(ByteOrder.nativeOrder());
      images = new SideDependentList<>(ByteBuffer.allocate(3 * imageWidth * imageHeight), // uint8 rgb Channel - Height - Width
                                        ByteBuffer.allocate(3 * imageWidth * imageHeight));
      for (RobotSide side : RobotSide.values)
         images.get(side).order(ByteOrder.nativeOrder());
      actions = ByteBuffer.allocate(chunkLength * stateSize * Double.BYTES);
      actions.order(ByteOrder.nativeOrder());
      horizon = chunkLength;
   }

   public void setPrompt(String prompt)
   {
      this.prompt = prompt;
   }

   public CompletableFuture<byte[]> request()
   {
      try
      {
         if (channel == null || !channel.isActive()) // reconnect if necessary
         {
            destroy();

            try
            {
               group = new NioEventLoopGroup();
               Bootstrap bootstrap = new Bootstrap();
               handler = new OpenpiNettyWebSocketHandler(new URI("ws://" + host + ":" + port));
               bootstrap.group(group).channel(NioSocketChannel.class).handler(new ChannelInitializer<SocketChannel>()
               {
                  @Override
                  protected void initChannel(SocketChannel ch)
                  {
                     ChannelPipeline pipeline = ch.pipeline();
                     pipeline.addLast(new HttpClientCodec());
                     pipeline.addLast(new HttpObjectAggregator(65536));
                     pipeline.addLast(handler);
                  }
               });
               channel = bootstrap.connect(host, port).sync().channel();
               handler.handshakeFuture().sync();
               handler.awaitFirstMessage(10, TimeUnit.SECONDS);
            }
            catch (Exception exception)
            {
               return null;
            }
         }

         packer.clear();
         packer.packMapHeader(4);
         for (RobotSide side : RobotSide.values)
         {
            packer.packString("cam_zed_%s".formatted(side.getLowerCaseName())).packMapHeader(4);
               packer.packString("__ndarray__").packBoolean(true);
               byte[] imgData = images.get(side).array();
               packer.packString("data").packBinaryHeader(imgData.length).writePayload(imgData);
               packer.packString("dtype").packString("uint8");
               packer.packString("shape").packArrayHeader(3);
                  packer.packInt(3); // channels, rgb
                  packer.packInt(imageHeight);
                  packer.packInt(imageWidth);
         }
            packer.packString("state").packMapHeader(4);
               packer.packString("__ndarray__").packBoolean(true);
               packer.packString("data").packBinaryHeader(state.array().length).writePayload(state.array());
               packer.packString("dtype").packString("float32");
               packer.packString("shape").packArrayHeader(1).packInt(stateSize);
            packer.packString("prompt").packString(prompt);

         return handler.sendAndAwaitResponse(packer.toByteArray());
      }
      catch (Exception e)
      {
         throw new RuntimeException("Request failed", e);
      }
   }

   public void unpack(CompletableFuture<byte[]> response)
   {
      try
      {
         MessageUnpacker unpacker = MessagePack.newDefaultUnpacker(response.get());
         unpacker.unpackMapHeader(); // 4
            unpacker.unpackString(); // actions
            unpacker.unpackMapHeader(); // 4
               unpacker.unpackString();  // __ndarray__
               unpacker.unpackBoolean(); // true
               unpacker.unpackString(); // data
               unpacker.unpackBinaryHeader(); // chunkLength * stateSize * 8
               unpacker.readPayload(actions.array());
               unpacker.unpackString(); // dtype
               unpacker.unpackString(); // <f8 (double)
               unpacker.unpackString(); // shape
               unpacker.unpackArrayHeader(); // 2
                  unpacker.unpackInt(); // chunkLength
                  unpacker.unpackInt(); // stateSize
            unpacker.unpackString(); // policy_timing
            unpacker.unpackMapHeader();
               unpacker.unpackString(); // infer_ms
               policyTimingMs = unpacker.unpackFloat();
            unpacker.unpackString(); // server_timing
            unpacker.unpackMapHeader();
               unpacker.unpackString(); // infer_ms
               serverTimingMs = unpacker.unpackFloat();
            unpacker.unpackString(); // horizon
            // Genuine-prediction count within actions; the rest (up to chunkLength) is the
            // server's held/repeated last-real-step tail (see gr00t_openpi_bridge_server.py).
            horizon = Math.max(1, Math.min(chunkLength, unpacker.unpackInt()));
         unpacker.close();
      }
      catch (Exception e)
      {
         DefaultExceptionHandler.MESSAGE_AND_STACKTRACE.handleException(e);
      }
   }

   public ByteBuffer getState()
   {
      return state;
   }

   public SideDependentList<ByteBuffer> getImages()
   {
      return images;
   }

   public ByteBuffer getActionChunk()
   {
      return actions;
   }

   public int getHorizon()
   {
      return horizon;
   }

   public float getPolicyTimingMs()
   {
      return policyTimingMs;
   }

   public float getServerTimingMs()
   {
      return serverTimingMs;
   }

   public void destroy()
   {
      if (channel != null)
         channel.close();
      if (group != null)
         group.shutdownGracefully();

      channel = null;
      group = null;
   }

   public boolean hasBeenStarted()
   {
      return group != null;
   }

   public String getHost()
   {
      return host;
   }

   public int getPort()
   {
      return port;
   }

   public int getStateSize()
   {
      return stateSize;
   }

   public int getChunkLength()
   {
      return chunkLength;
   }

   public int getImageWidth()
   {
      return imageWidth;
   }

   public int getImageHeight()
   {
      return imageHeight;
   }
}
