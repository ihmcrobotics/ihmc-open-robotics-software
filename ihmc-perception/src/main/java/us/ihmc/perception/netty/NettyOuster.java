package us.ihmc.perception.netty;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import io.netty.bootstrap.Bootstrap;
import io.netty.buffer.ByteBuf;
import io.netty.channel.*;
import io.netty.channel.nio.NioEventLoopGroup;
import io.netty.channel.socket.DatagramPacket;
import io.netty.channel.socket.nio.NioDatagramChannel;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;

import java.io.*;
import java.net.Socket;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.time.Instant;

/**
 * Ouster Firmware User Manual: https://data.ouster.io/downloads/software-user-manual/firmware-user-manual-v2.3.0.pdf
 * Software User Manual: https://data.ouster.io/downloads/software-user-manual/software-user-manual-v2p0.pdf
 * 
 * To test, use the GNU netcat command:
 * netcat -ul 7502
 *
 * Lidar incoming data format:
 *
 * <img src="https://i.imgur.com/tB4dI3H.png" width="800" alt="description of MyImage">
 *
 */
public class NettyOuster
{
   public static final int TCP_PORT = 7501;
   public static final int UDP_PORT = 7502;

   private static final int BITS_PER_BYTE = 8;

   // Header block
   private static final int HEADER_BLOCK_BITS = 128;
   private static final int HEADER_BLOCK_BYTES = HEADER_BLOCK_BITS / BITS_PER_BYTE;
   // Header block contents
   private static final int TIMESTAMP_BITS = 64;
   private static final int TIMESTAMP_BYTES = TIMESTAMP_BITS / BITS_PER_BYTE;
   private static final int MEASUREMENT_ID_BITS = 16;
   private static final int MEASUREMENT_ID_BYTES = MEASUREMENT_ID_BITS / BITS_PER_BYTE;
   private static final int FRAME_ID_BITS = 16;
   private static final int FRAME_ID_BYTES = FRAME_ID_BITS / BITS_PER_BYTE;
   private static final int ENCODER_COUNT_BITS = 32;
   private static final int ENCODER_COUNT_BYTES = ENCODER_COUNT_BITS / BITS_PER_BYTE;

   // Channel data block
   private static final int CHANNEL_DATA_BLOCK_BITS = 96;
   private static final int CHANNEL_DATA_BLOCK_BYTES = CHANNEL_DATA_BLOCK_BITS / BITS_PER_BYTE;
   // Channel data block contents
   private static final int RANGE_MM_BITS = 20;
   private static final int RANGE_MM_BYTES = RANGE_MM_BITS / BITS_PER_BYTE;
   private static final int UNUSED_BITS = 12;
   private static final int RANGE_ROW_BYTES = (RANGE_MM_BITS + UNUSED_BITS) / BITS_PER_BYTE;
   private static final int REFLECTIVITY_BITS = 16;
   private static final int REFLECTIVITY_BYTES = REFLECTIVITY_BITS / BITS_PER_BYTE;
   private static final int SIGNAL_PHOTONS_BITS = 16;
   private static final int SIGNAL_PHOTONS_BYTES = SIGNAL_PHOTONS_BITS / BITS_PER_BYTE;
   private static final int NEAR_INFRARED_PHOTONS_BITS = 16;
   private static final int NEAR_INFRARED_PHOTONS_BYTES = NEAR_INFRARED_PHOTONS_BITS / BITS_PER_BYTE;
   private static final int MORE_UNUSED_BITS = 16;

   // Measurement block status
   private static final int MEASUREMENT_BLOCK_STATUS_BITS = 32;
   private static final int MEASUREMENT_BLOCK_STATUS_BYTES = MEASUREMENT_BLOCK_STATUS_BITS / BITS_PER_BYTE;

   // Lidar frames are composed of N UDP datagrams (data packets) containing 16 measurement blocks each
   // Example: For a OS0-128 running at 2048x10Hz, there will be N = 2048 / 16 = 128 UDP datagrams / scan
   // The frequency of UDP datagrams in this case would be 128 * 10Hz = 1280 Hz UDP datagram processing.
   //
   // UDP datagrams contain 16 measurement blocks
   // Each measurement block contains a full column of depth data: 32, 64, or 128
   // Example for OS0-128, each measurement block is (16 + (128 * 12) + 4) = 1556 bytes
   // and each UDP datagram will be 16 * measurement_block = 16 * 1556 = 24896 bytes
   //
   // A channel data block is one part of a measurement block, containing 1 pixel
   // It's 12 bytes
   private int pixelsPerColumn;
   private int columnsPerFrame;
   private static final int MEASUREMENT_BLOCKS_PER_UDP_DATAGRAM = 16;
   private int[] pixelShift;
   private int measurementBlockSize;
   private int udpDatagramsPerFrame;
   private int lidarFrameSizeBytes;

   private volatile boolean tryingTCP = false;
   private volatile boolean tcpInitialized = false;

   private final EventLoopGroup group;
   private final Bootstrap bootstrap;
   private Runnable onFrameReceived = null;
   private Instant aquisitionInstant;
   private ByteBuffer lidarFrameByteBuffer;
   private long nextExpectedMeasurementID = -1;

   public NettyOuster()
   {
      group = new NioEventLoopGroup();
      bootstrap = new Bootstrap();
      bootstrap.group(group).channel(NioDatagramChannel.class).handler(new SimpleChannelInboundHandler<DatagramPacket>()
      {
         @Override
         protected void channelRead0(ChannelHandlerContext context, DatagramPacket packet)
         {
            aquisitionInstant = Instant.now();

            if (!tryingTCP && !tcpInitialized)
            {
               tryingTCP = true;
               // Address looks like "/192.168.x.x" so we just discard the '/'
               String modifiedAddress = packet.sender().getAddress().toString().substring(1);
               // Do this on a thread so we don't hang up the UDP thread.
               ThreadTools.startAsDaemon(() ->
               {
                  configureTCP(modifiedAddress);
                  tryingTCP = false;
               }, "TCPConfiguration");
            }
            else
            {
               ByteBuf content = packet.content();

               // Ouster data is little endian
               int measurementID = content.getUnsignedShortLE(TIMESTAMP_BYTES);
               if (nextExpectedMeasurementID > 0 && measurementID != nextExpectedMeasurementID)
               {
                  LogTools.warn("UDP datagram skipped! Expected measurement ID {} but was {}", nextExpectedMeasurementID, measurementID);
               }
               nextExpectedMeasurementID = (measurementID + MEASUREMENT_BLOCKS_PER_UDP_DATAGRAM) % columnsPerFrame;

               lidarFrameByteBuffer.position(measurementID * measurementBlockSize);
               content.readBytes(lidarFrameByteBuffer);

               boolean isLastBlockInFrame = measurementID == (columnsPerFrame - MEASUREMENT_BLOCKS_PER_UDP_DATAGRAM);
               if (isLastBlockInFrame && onFrameReceived != null)
               {
                  onFrameReceived.run();
               }
            }
         }
      });

      // Ouster OS0-128 uses about 1.5 MB for the buffer, so let's set the allocator to 2 MB to leave a little wiggle room
      bootstrap.option(ChannelOption.RCVBUF_ALLOCATOR, new FixedRecvByteBufAllocator(Conversions.megabytesToBytes(2)));
   }

   private void configureTCP(String host)
   {
      LogTools.info("Attempting to query host " + host);

      String jsonResponse = "";
      LogTools.info("Binding to TCP port " + TCP_PORT);
      try (Socket socket = new Socket(host, TCP_PORT))
      {
         OutputStream output = socket.getOutputStream();
         PrintWriter writer = new PrintWriter(output, true);
         writer.println("get_lidar_data_format");

         InputStream input = socket.getInputStream();
         BufferedReader reader = new BufferedReader(new InputStreamReader(input));

         jsonResponse = reader.readLine();
      }
      catch (UnknownHostException unknownHostException)
      {
         LogTools.error("Ouster host could not be found.");
      }
      catch (IOException ioException)
      {
         LogTools.error(ioException.getMessage());
         LogTools.error(ioException.getStackTrace());
      }
      finally
      {
         LogTools.info("Unbound from TCP port " + TCP_PORT);
      }

      try
      {
         ObjectMapper mapper = new ObjectMapper();
         JsonNode root = mapper.readTree(jsonResponse);

         pixelsPerColumn = root.get("pixels_per_column").asInt();
         columnsPerFrame = root.get("columns_per_frame").asInt();
         measurementBlockSize = HEADER_BLOCK_BYTES + (pixelsPerColumn * CHANNEL_DATA_BLOCK_BYTES) + MEASUREMENT_BLOCK_STATUS_BYTES;
         udpDatagramsPerFrame = columnsPerFrame / MEASUREMENT_BLOCKS_PER_UDP_DATAGRAM;
         lidarFrameSizeBytes = MEASUREMENT_BLOCKS_PER_UDP_DATAGRAM * measurementBlockSize * udpDatagramsPerFrame;

         LogTools.info("Pixels Per Column: {}, Columns Per Frame: {}, UDP datagrams per frame: {}",
                       pixelsPerColumn,
                       columnsPerFrame,
                       udpDatagramsPerFrame);

         lidarFrameByteBuffer = ByteBuffer.allocateDirect(lidarFrameSizeBytes);

         JsonNode pixelShiftNode = root.get("pixel_shift_by_row");
         pixelShift = new int[pixelsPerColumn];
         for (int i = 0; i < pixelsPerColumn; i++)
         {
            pixelShift[i] = pixelShiftNode.get(i).asInt();
         }

         // TODO: Store and print frequency
      }
      catch (JsonProcessingException jsonProcessingException)
      {
         LogTools.error(jsonProcessingException.getMessage());
      }

      LogTools.info("Ouster is initialized.");
      tcpInitialized = true;
   }

   /**
    * Bind to UDP and begin receiving data.
    */
   public void bind()
   {
      LogTools.info("Binding to UDP port " + UDP_PORT);
      bootstrap.bind(UDP_PORT);
   }

   public void destroy()
   {
      LogTools.info("Unbinding from UDP port " + UDP_PORT);
      group.shutdownGracefully();
   }

   public boolean isInitialized()
   {
      return tcpInitialized;
   }

   public int getImageWidth()
   {
      return tcpInitialized ? columnsPerFrame : -1;
   }

   public int getImageHeight()
   {
      return tcpInitialized ? pixelsPerColumn : -1;
   }

   public void setOnFrameReceived(Runnable onFrameReceived)
   {
      this.onFrameReceived = onFrameReceived;
   }

   public ByteBuffer getLidarFrameByteBuffer()
   {
      return lidarFrameByteBuffer;
   }

   public Instant getAquisitionInstant()
   {
      return aquisitionInstant;
   }

   public int getPixelsPerColumn()
   {
      return pixelsPerColumn;
   }

   public int getColumnsPerFrame()
   {
      return columnsPerFrame;
   }

   public int getMeasurementBlockSize()
   {
      return measurementBlockSize;
   }
}
