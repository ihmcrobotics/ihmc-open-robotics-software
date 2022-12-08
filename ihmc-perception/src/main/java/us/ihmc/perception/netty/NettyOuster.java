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
import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.tools.thread.Activator;

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
 * Lidar packet format:
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

   private final Activator nativesLoadedActivator;

   // A frame is the whole full width depth image before it's starts repeating again; also a lidar packet
   // A packet is a group of data that comes in one UDP message; also a measurement block
   // A column is a single column of depth data
   // A pixel is one depth measurement; also a channel data block
   private int pixelsPerColumn;
   private int columnsPerFrame;
   private int columnsPerPacket;
   private int[] pixelShift;
   private int measurementBlocksPerFrame;
   private int channelDataBlocksPerBlocksPerPacket;
   private int measurementBlockSize;
   private int lidarPacketSize;

   private boolean tcpInitialized = false;

   private final EventLoopGroup group;
   private final Bootstrap bootstrap;
   private BytedecoImage depthImageMeters;
   private Runnable onFrameReceived = null;
   private Instant aquisitionInstant;
   private OpenCLManager openCLManager;
   private ByteBuffer lidarPacketBuffer;
   private _cl_program openCLProgram;
   private _cl_kernel extractDepthImageKernel;

   public NettyOuster()
   {
      nativesLoadedActivator = BytedecoTools.loadNativesOnAThread();

      group = new NioEventLoopGroup();
      bootstrap = new Bootstrap();
      bootstrap.group(group).channel(NioDatagramChannel.class).handler(new SimpleChannelInboundHandler<DatagramPacket>()
      {
         @Override
         protected void channelRead0(ChannelHandlerContext context, DatagramPacket packet)
         {
            aquisitionInstant = Instant.now();

            if (!tcpInitialized)
            {
               boolean success = configureTCP(packet.sender().getAddress().toString().substring(1));
               if (!success)
               { //Address looks like "/192.168.x.x" so we just discard the '/'
                  LogTools.error("Failed to initialize Ouster using TCP API.");
               }
            }
            else
            {
               if (nativesLoadedActivator.poll())
               {
                  if (nativesLoadedActivator.isNewlyActivated())
                  {
                     depthImageMeters = new BytedecoImage(columnsPerFrame, pixelsPerColumn, opencv_core.CV_32FC1);
                     depthImageMeters.getBytedecoOpenCVMat().setTo(new Mat(0.0f)); // Initialize to 0

                     openCLManager = new OpenCLManager();
                     openCLProgram = openCLManager.loadProgram("OusterDepthImageUpdater");
                     extractDepthImageKernel = openCLManager.createKernel(openCLProgram, "extractDepthImage");

                     lidarPacketBuffer = ByteBuffer.allocateDirect(lidarPacketSize);
                  }

                  ByteBuf bufferedData = packet.content().readBytes(measurementBlockSize);

                  long measurementID = bufferedData.getUnsignedInt(TIMESTAMP_BYTES);

                  lidarPacketBuffer.position((int) measurementID * measurementBlockSize);
                  bufferedData.readBytes(lidarPacketBuffer);

                  boolean isLastBlockInFrame = measurementID == (measurementBlocksPerFrame - 1);
                  if (isLastBlockInFrame && onFrameReceived != null)
                     onFrameReceived.run();

                  bufferedData.release();
               }
            }
         }
      });
      bootstrap.option(ChannelOption.RCVBUF_ALLOCATOR, new FixedRecvByteBufAllocator(lidarPacketSize));
   }

   private boolean configureTCP(String host)
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
         return false;
      }
      catch (IOException ioException)
      {
         LogTools.error(ioException.getMessage());
         LogTools.error(ioException.getStackTrace());
         return false;
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
         columnsPerPacket = root.get("columns_per_packet").asInt();
         measurementBlocksPerFrame = columnsPerFrame / columnsPerPacket;
         channelDataBlocksPerBlocksPerPacket = columnsPerPacket * pixelsPerColumn;
         measurementBlockSize = HEADER_BLOCK_BYTES + (channelDataBlocksPerBlocksPerPacket * CHANNEL_DATA_BLOCK_BYTES) + MEASUREMENT_BLOCK_STATUS_BYTES;
         lidarPacketSize = measurementBlockSize * measurementBlocksPerFrame;

         LogTools.info("Pixels Per Column: {}, Columns Per Frame: {}, Columns Per Packet: {}", pixelsPerColumn, columnsPerFrame, columnsPerPacket);

         JsonNode pixelShiftNode = root.get("pixel_shift_by_row");
         pixelShift = new int[pixelsPerColumn];
         for (int i = 0; i < pixelsPerColumn; i++)
         {
            pixelShift[i] = pixelShiftNode.get(i).asInt();
         }
      }
      catch (JsonProcessingException jsonProcessingException)
      {
         LogTools.error(jsonProcessingException.getMessage());
         return false;
      }

      tcpInitialized = true;
      return true;
   }

   /**
    * Bind to UDP, and begin receiving data.
    * Note that data will not be processed until Ouster's TCP API is queried for image data, which will not happen until after this call.
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

   public BytedecoImage getDepthImageMeters()
   {
      return depthImageMeters;
   }

   public void setOnFrameReceived(Runnable onFrameReceived)
   {
      this.onFrameReceived = onFrameReceived;
   }

   public Instant getAquisitionInstant()
   {
      return aquisitionInstant;
   }
}
