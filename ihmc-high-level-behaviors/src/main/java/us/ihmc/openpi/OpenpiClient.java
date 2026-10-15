package us.ihmc.openpi;

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
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.net.URI;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.TimeUnit;

public class OpenpiClient
{
   private final String host;
   private final int port;
   private final int stateSize;
   private final int actionSize;
   private final int chunkLength;
   private final int imageWidth;
   private final int imageHeight;
   private final SideDependentList<String> imageKeys;
   private final String stateKey;
   private final String promptKey;
   private final String actionKey;
   private String prompt;
   private EventLoopGroup group;
   private Channel channel;
   private OpenpiNettyWebSocketHandler handler;
   private final MessageBufferPacker packer = MessagePack.newDefaultBufferPacker();
   private final ByteBuffer state;
   private final SideDependentList<ByteBuffer> images;
   private final ByteBuffer actions;
   private float policyTimingMs;
   private float serverTimingMs;
   private int horizon;
   private volatile String lastConnectionError;

   public OpenpiClient(String host, int stateSize)
   {
      this(host, 8000, stateSize, 50, 224, 224, "touch door handle");
   }

   public OpenpiClient(String host, int port, int stateSize, int chunkLength, int imageWidth, int imageHeight, String prompt)
   {
      this(host, port, stateSize, stateSize, chunkLength, imageWidth, imageHeight, prompt,
           new SideDependentList<>("cam_zed_left", "cam_zed_right"), "state", "prompt", "actions");
   }

   /**
    * Creates a client with independent state/action dimensions and configurable wire keys.
    * This keeps the historical OpenPI defaults while allowing policies whose action vector is
    * sparse, reordered, or otherwise different from the observation state vector.
    */
   public OpenpiClient(String host,
                       int port,
                       int stateSize,
                       int actionSize,
                       int chunkLength,
                       int imageWidth,
                       int imageHeight,
                       String prompt,
                       SideDependentList<String> imageKeys,
                       String stateKey,
                       String promptKey,
                       String actionKey)
   {
      if (stateSize <= 0 || actionSize <= 0 || chunkLength <= 0)
         throw new IllegalArgumentException("State size, action size, and chunk length must be positive");
      if (imageWidth <= 0 || imageHeight <= 0)
         throw new IllegalArgumentException("Image dimensions must be positive");
      if (host == null || host.isBlank())
         throw new IllegalArgumentException("Host must not be blank");
      if (port <= 0 || port > 65535)
         throw new IllegalArgumentException("Port must be in [1, 65535]");
      if (imageKeys == null)
         throw new IllegalArgumentException("Image wire keys must not be null");

      this.host = host;
      this.port = port;
      this.stateSize = stateSize;
      this.actionSize = actionSize;
      this.chunkLength = chunkLength;
      this.imageWidth = imageWidth;
      this.imageHeight = imageHeight;
      this.prompt = prompt;
      this.imageKeys = new SideDependentList<>(imageKeys.get(RobotSide.LEFT), imageKeys.get(RobotSide.RIGHT));
      this.stateKey = requireWireKey(stateKey, "state");
      this.promptKey = requireWireKey(promptKey, "prompt");
      this.actionKey = requireWireKey(actionKey, "action");
      Set<String> requestKeys = new HashSet<>();
      for (RobotSide side : RobotSide.values)
      {
         String imageKey = requireWireKey(this.imageKeys.get(side), side.getLowerCaseName() + " image");
         if (!requestKeys.add(imageKey))
            throw new IllegalArgumentException("Request wire keys must be unique; duplicate key: " + imageKey);
      }
      if (!requestKeys.add(this.stateKey))
         throw new IllegalArgumentException("Request wire keys must be unique; duplicate key: " + this.stateKey);
      if (!requestKeys.add(this.promptKey))
         throw new IllegalArgumentException("Request wire keys must be unique; duplicate key: " + this.promptKey);

      state = ByteBuffer.allocate(stateSize * Float.BYTES);
      state.order(ByteOrder.LITTLE_ENDIAN);
      images = new SideDependentList<>(ByteBuffer.allocate(3 * imageWidth * imageHeight), // uint8 rgb Channel - Height - Width
                                       ByteBuffer.allocate(3 * imageWidth * imageHeight));
      for (RobotSide side : RobotSide.values)
         images.get(side).order(ByteOrder.LITTLE_ENDIAN);
      actions = ByteBuffer.allocate(chunkLength * actionSize * Double.BYTES);
      actions.order(ByteOrder.LITTLE_ENDIAN);
      horizon = chunkLength;
   }

   private static String requireWireKey(String key, String description)
   {
      if (key == null || key.isBlank())
         throw new IllegalArgumentException(description + " wire key must not be blank");
      return key;
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
               validateServerMetadata(handler.awaitFirstMessage(10, TimeUnit.SECONDS));
               lastConnectionError = null;
            }
            catch (Exception exception)
            {
               lastConnectionError = exception.getMessage() == null ? exception.getClass().getSimpleName() : exception.getMessage();
               destroy();
               return null;
            }
         }

         packer.clear();
         packer.packMapHeader(4);
         for (RobotSide side : RobotSide.values)
         {
            packer.packString(imageKeys.get(side)).packMapHeader(4);
               packer.packString("__ndarray__").packBoolean(true);
               byte[] imgData = images.get(side).array();
               packer.packString("data").packBinaryHeader(imgData.length).writePayload(imgData);
               packer.packString("dtype").packString("uint8");
               packer.packString("shape").packArrayHeader(3);
                  packer.packInt(3); // channels, rgb
                  packer.packInt(imageHeight);
                  packer.packInt(imageWidth);
         }
            packer.packString(stateKey).packMapHeader(4);
               packer.packString("__ndarray__").packBoolean(true);
               packer.packString("data").packBinaryHeader(state.array().length).writePayload(state.array());
               packer.packString("dtype").packString("float32");
               packer.packString("shape").packArrayHeader(1).packInt(stateSize);
            packer.packString(promptKey).packString(prompt);

         return handler.sendAndAwaitResponse(packer.toByteArray());
      }
      catch (Exception e)
      {
         throw new RuntimeException("Request failed", e);
      }
   }

   /** Hook for policy-specific clients to reject an incompatible server before sending observations. */
   protected void validateServerMetadata(byte[] metadata) throws Exception
   {
   }

   /** Returns false and clears the action buffer when any response field is malformed. */
   public boolean unpack(CompletableFuture<byte[]> response)
   {
      policyTimingMs = Float.NaN;
      serverTimingMs = Float.NaN;
      horizon = chunkLength;
      try
      {
         MessageUnpacker unpacker = MessagePack.newDefaultUnpacker(response.get());
         int responseFieldCount = unpacker.unpackMapHeader();
         boolean receivedActions = false;
         for (int field = 0; field < responseFieldCount; field++)
         {
            String key = unpacker.unpackString();
            if (actionKey.equals(key))
            {
               unpackActions(unpacker);
               receivedActions = true;
            }
            else
            {
               switch (key)
               {
                  case "policy_timing" -> policyTimingMs = unpackTiming(unpacker);
                  case "server_timing" -> serverTimingMs = unpackTiming(unpacker);
                  case "horizon" -> horizon = Math.max(1, Math.min(chunkLength, unpacker.unpackInt()));
                  default -> unpacker.skipValue();
               }
            }
         }
         if (!receivedActions)
            throw new IllegalArgumentException("GR00T response did not contain " + actionKey);
         unpacker.close();
         return true;
      }
      catch (Exception e)
      {
         actions.clear();
         while (actions.hasRemaining())
            actions.put((byte) 0);
         actions.clear();
         horizon = 0;
         policyTimingMs = Float.NaN;
         serverTimingMs = Float.NaN;
         DefaultExceptionHandler.MESSAGE_AND_STACKTRACE.handleException(e);
         return false;
      }
   }

   private void unpackActions(MessageUnpacker unpacker) throws Exception
   {
      int ndarrayFieldCount = unpacker.unpackMapHeader();
      byte[] payload = null;
      String dtype = null;
      int[] shape = null;
      boolean ndarray = false;
      for (int field = 0; field < ndarrayFieldCount; field++)
      {
         String key = unpacker.unpackString();
         switch (key)
         {
            case "__ndarray__" ->
            {
               if (!unpacker.unpackBoolean())
                  throw new IllegalArgumentException("actions is not an ndarray");
               ndarray = true;
            }
            case "data" -> payload = unpacker.readPayload(unpacker.unpackBinaryHeader());
            case "dtype" -> dtype = unpacker.unpackString();
            case "shape" ->
            {
               int dimensions = unpacker.unpackArrayHeader();
               shape = new int[dimensions];
               for (int dimension = 0; dimension < dimensions; dimension++)
                  shape[dimension] = unpacker.unpackInt();
            }
            default -> unpacker.skipValue();
         }
      }

      int[] expectedShape = {chunkLength, actionSize};
      if (!ndarray)
         throw new IllegalArgumentException("actions is missing the ndarray marker");
      if (!Arrays.equals(shape, expectedShape))
         throw new IllegalArgumentException("Expected action shape " + Arrays.toString(expectedShape) + ", got " + Arrays.toString(shape));
      if (!"<f8".equals(dtype) && !"float64".equals(dtype))
         throw new IllegalArgumentException("Expected float64 actions, got " + dtype);
      if (payload == null || payload.length != actions.capacity())
         throw new IllegalArgumentException("Expected " + actions.capacity() + " action bytes, got " + (payload == null ? 0 : payload.length));
      actions.clear();
      actions.put(payload);
      actions.clear();
   }

   private static float unpackTiming(MessageUnpacker unpacker) throws Exception
   {
      int timingFieldCount = unpacker.unpackMapHeader();
      float inferenceMilliseconds = Float.NaN;
      for (int field = 0; field < timingFieldCount; field++)
      {
         String key = unpacker.unpackString();
         if ("infer_ms".equals(key))
            inferenceMilliseconds = (float) unpacker.unpackDouble();
         else
            unpacker.skipValue();
      }
      return inferenceMilliseconds;
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

   public float getPolicyTimingMs()
   {
      return policyTimingMs;
   }

   public float getServerTimingMs()
   {
      return serverTimingMs;
   }

   public int getHorizon()
   {
      return horizon;
   }

   public void destroy()
   {
      if (channel != null)
         channel.close();
      if (group != null)
         group.shutdownGracefully();

      channel = null;
      group = null;
      handler = null;
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

   public int getActionSize()
   {
      return actionSize;
   }

   public int getImageWidth()
   {
      return imageWidth;
   }

   public int getImageHeight()
   {
      return imageHeight;
   }

   public String getLastConnectionError()
   {
      return lastConnectionError;
   }

   public static void main(String[] args)
   {
      int stateSize = 3;
      OpenpiClient client = new OpenpiClient("10.6.192.65", 3);

      for (RobotSide side : RobotSide.values)
      {
         byte[] imgData = client.getImages().get(side).array();
         for (int i = 0; i < imgData.length; i++)
            imgData[i] = (byte) (i % 256);
      }

      ByteBuffer stateData = client.getState();
      for (int i = 0; i < stateSize; i++)
         stateData.putFloat((float) (i % stateSize));

      CompletableFuture<byte[]> request = client.request();
      client.unpack(request);

      LogTools.info("Action chunk: %d".formatted(client.getActionChunk().limit()));

      client.destroy();
   }
}
