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

   // A frame is the whole full width depth image before it's starts repeating again; also a lidar packet
   // A packet is a group of data that comes in one UDP message; also a measurement block
   // A column is a single column of depth data
   // A pixel is one depth measurement; also a channel data block
   private int pixelsPerColumn;
   private int columnsPerFrame;
   private int columnsPerMeasurementBlock;
   private int[] pixelShift;
   private int measurementBlocksPerFrame;
   private int channelDataBlocksPerBlocksPerMeasurementBlock;
   private int measurementBlockSize;
   private int lidarFrameSizeBytes;

   private volatile boolean tryingTCP = false;
   private volatile boolean tcpInitialized = false;

   private final EventLoopGroup group;
   private final Bootstrap bootstrap;
   private Runnable onFrameReceived = null;
   private Instant aquisitionInstant;
   private ByteBuffer lidarFrameByteBuffer;

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
               if (lidarFrameByteBuffer == null)
               {
                  LogTools.info("Ouster is initialized.");
                  lidarFrameByteBuffer = ByteBuffer.allocateDirect(lidarFrameSizeBytes);
               }

               ByteBuf bufferedData = packet.content().readBytes(measurementBlockSize);

               long measurementID = bufferedData.getUnsignedInt(TIMESTAMP_BYTES);

               lidarFrameByteBuffer.position((int) measurementID * measurementBlockSize);
               bufferedData.readBytes(lidarFrameByteBuffer);

               // TODO: Make sure to throw errors on missed UDP packets
               boolean isLastBlockInFrame = measurementID == (measurementBlocksPerFrame - 1);
               if (isLastBlockInFrame && onFrameReceived != null)
                  onFrameReceived.run();

               bufferedData.release();
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
         columnsPerMeasurementBlock = root.get("columns_per_packet").asInt();
         measurementBlocksPerFrame = columnsPerFrame / columnsPerMeasurementBlock;
         channelDataBlocksPerBlocksPerMeasurementBlock = columnsPerMeasurementBlock * pixelsPerColumn;
         measurementBlockSize = HEADER_BLOCK_BYTES
                                + (channelDataBlocksPerBlocksPerMeasurementBlock * CHANNEL_DATA_BLOCK_BYTES)
                                + MEASUREMENT_BLOCK_STATUS_BYTES;
         lidarFrameSizeBytes = measurementBlockSize * measurementBlocksPerFrame;

         LogTools.info("Pixels Per Column: {}, Columns Per Frame: {}, Columns Per Measurement Block: {}",
                       pixelsPerColumn,
                       columnsPerFrame,
                       columnsPerMeasurementBlock);

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

   public int getColumnsPerMeasurementBlock()
   {
      return columnsPerMeasurementBlock;
   }

   public int getPixelsPerColumn()
   {
      return pixelsPerColumn;
   }

   public int getMeasurementBlockSize()
   {
      return measurementBlockSize;
   }
}
