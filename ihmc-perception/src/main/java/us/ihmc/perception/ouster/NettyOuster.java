package us.ihmc.perception.ouster;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import io.netty.bootstrap.Bootstrap;
import io.netty.buffer.ByteBuf;
import io.netty.channel.*;
import io.netty.channel.epoll.Epoll;
import io.netty.channel.epoll.EpollDatagramChannel;
import io.netty.channel.epoll.EpollEventLoopGroup;
import io.netty.channel.socket.DatagramPacket;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.perception.tools.NativeMemoryTools;

import java.io.*;
import java.net.Socket;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.time.Instant;
import java.util.function.Consumer;

/**
 * This class is written to do no operations on incoming Ouster data for maximum performance.
 * It just lines the incoming data up into a single buffer representing one full frame. It also
 * does so on the event that the UDP datagrams are received by Netty for the fastest possible
 * response. All operations on this data should be done externally by OpenCL kernels.
 *
 * Ouster Firmware User Manual: https://data.ouster.io/downloads/software-user-manual/firmware-user-manual-v2.3.0.pdf
 * Software User Manual: https://data.ouster.io/downloads/software-user-manual/software-user-manual-v2p0.pdf
 *
 * To connect to the Ouster, type in os-122221003063.local into the browser, finding the serial number
 * from the Confluence page: https://confluence.ihmc.us/display/PER/In-House+Perception+Sensors+Tracker
 *
 * Apparently it is better to use this magical command from the user manual:
 * avahi-browse -lrt _roger._tcp
 *
 * To test, use the GNU netcat command:
 * netcat -ul 7502
 *
 * <p>
 * Lidar incoming data format:
 * </p>
 *
 * <img src="https://www.ihmc.us/wp-content/uploads/2023/02/OusterLidarDataFormat.png" width="800">
 *
 */
public class NettyOuster
{
   public static final int TCP_PORT = 7501;
   public static final int UDP_PORT = 7502;

   /** Ouster produces ranges as unsigned integers in millimeters, so this is just a conversion to meters. */
   public static final float DISCRETE_RESOLUTION = 0.001f;
   public static final int MAX_POINTS_PER_COLUMN = 128;

   public static final int BITS_PER_BYTE = 8;

   // Header block
   public static final int HEADER_BLOCK_BITS = 128;
   public static final int HEADER_BLOCK_BYTES = HEADER_BLOCK_BITS / BITS_PER_BYTE;
   // Header block contents
   public static final int TIMESTAMP_BITS = 64;
   public static final int TIMESTAMP_BYTES = TIMESTAMP_BITS / BITS_PER_BYTE;
   public static final int MEASUREMENT_ID_BITS = 16;
   public static final int MEASUREMENT_ID_BYTES = MEASUREMENT_ID_BITS / BITS_PER_BYTE;
   public static final int FRAME_ID_BITS = 16;
   public static final int FRAME_ID_BYTES = FRAME_ID_BITS / BITS_PER_BYTE;
   public static final int ENCODER_COUNT_BITS = 32;
   public static final int ENCODER_COUNT_BYTES = ENCODER_COUNT_BITS / BITS_PER_BYTE;

   // Channel data block
   public static final int CHANNEL_DATA_BLOCK_BITS = 96;
   public static final int CHANNEL_DATA_BLOCK_BYTES = CHANNEL_DATA_BLOCK_BITS / BITS_PER_BYTE;
   // Channel data block contents
   public static final int RANGE_MM_BITS = 20;
   public static final int RANGE_MM_BYTES = RANGE_MM_BITS / BITS_PER_BYTE;
   public static final int UNUSED_BITS = 12;
   public static final int RANGE_ROW_BYTES = (RANGE_MM_BITS + UNUSED_BITS) / BITS_PER_BYTE;
   public static final int REFLECTIVITY_BITS = 16;
   public static final int REFLECTIVITY_BYTES = REFLECTIVITY_BITS / BITS_PER_BYTE;
   public static final int SIGNAL_PHOTONS_BITS = 16;
   public static final int SIGNAL_PHOTONS_BYTES = SIGNAL_PHOTONS_BITS / BITS_PER_BYTE;
   public static final int NEAR_INFRARED_PHOTONS_BITS = 16;
   public static final int NEAR_INFRARED_PHOTONS_BYTES = NEAR_INFRARED_PHOTONS_BITS / BITS_PER_BYTE;
   public static final int MORE_UNUSED_BITS = 16;

   // Measurement block status
   public static final int MEASUREMENT_BLOCK_STATUS_BITS = 32;
   public static final int MEASUREMENT_BLOCK_STATUS_BYTES = MEASUREMENT_BLOCK_STATUS_BITS / BITS_PER_BYTE;

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
   public static final int MEASUREMENT_BLOCKS_PER_UDP_DATAGRAM = 16;
   private ByteBuffer pixelShiftBuffer;
   private float lidarOriginToBeamOrigin;
   private ByteBuffer beamAltitudeAnglesBuffer;
   private ByteBuffer beamAzimuthAnglesBuffer;
   private int measurementBlockSize;
   private int udpDatagramsPerFrame;
   private int lidarFrameSizeBytes;
   private final RigidBodyTransform spindleCenterToBaseTransform = new RigidBodyTransform();

   private volatile boolean tryingTCP = false;
   private volatile boolean tcpInitialized = false;
   private String sanitizededHostAddress;

   private final EventLoopGroup group;
   private final Bootstrap bootstrap;
   private Runnable onFrameReceived = null;
   private Instant aquisitionInstant;
   private ByteBuffer lidarFrameByteBuffer;
   private long nextExpectedMeasurementID = -1;
   private String lidarMode;

   private boolean printedWarning = false;

   public NettyOuster()
   {
      /*
       * We require Epoll to be available to avoid high CPU in ROS2Node instances.
       * This service will only work on Linux.
       * https://netty.io/wiki/native-transports.html
       */
      if (!Epoll.isAvailable())
      {
         throw new RuntimeException("Netty native transport (Epoll) is not available. Epoll is required to avoid high CPU in certain situations.");
      }

      group = new EpollEventLoopGroup();
      bootstrap = new Bootstrap();
      bootstrap.group(group).channel(EpollDatagramChannel.class).handler(new SimpleChannelInboundHandler<DatagramPacket>()
      {
         @Override
         protected void channelRead0(ChannelHandlerContext context, DatagramPacket packet)
         {
            aquisitionInstant = Instant.now();

            if (!tryingTCP && !tcpInitialized)
            {
               tryingTCP = true;
               // Address looks like "/192.168.x.x" so we just discard the '/'
               sanitizededHostAddress = packet.sender().getAddress().toString().substring(1);
               // Do this on a thread so we don't hang up the UDP thread.
               ThreadTools.startAsDaemon(() ->
               {
                  configureTCP();
                  tryingTCP = false;
               }, "TCPConfiguration");
            }
            if (tcpInitialized)
            {
               ByteBuf content = packet.content();

               // Ouster data is little endian
               int measurementID = content.getUnsignedShortLE(TIMESTAMP_BYTES);
               if (nextExpectedMeasurementID > 0 && measurementID != nextExpectedMeasurementID && !printedWarning)
               {
                  printedWarning = true;
                  LogTools.warn("UDP datagram skipped! Expected measurement ID {} but was {}. Skipping this warning in the future", nextExpectedMeasurementID, measurementID);
               }
               nextExpectedMeasurementID = (measurementID + MEASUREMENT_BLOCKS_PER_UDP_DATAGRAM) % columnsPerFrame;

               // Copying UDP datagram content into correct position in whole frame buffer.
               lidarFrameByteBuffer.position(measurementID * measurementBlockSize);
               content.getBytes(0, lidarFrameByteBuffer);

               boolean isLastBlockInFrame = measurementID == (columnsPerFrame - MEASUREMENT_BLOCKS_PER_UDP_DATAGRAM);
               if (isLastBlockInFrame && onFrameReceived != null)
               {
                  onFrameReceived.run();
               }
            }
         }
      });

      // Ouster OS0-128 at 2048x10 sends 3,186,688 bytes per revolution, at 10 Hz, is ~31.9 MB/s (~255 Mb/s)
      // We tried lots of options, including lower and higher sizes, 4 MB seemed to be a sweet spot.
      // Lower values resulted in the beginning part of the image being cut off.
      // Higher values resulted in intermitent dropouts of parts of the scan lines throughout the entire image.
      // We tried the AdaptiveRecvByteBufAllocator, but depending on the initial values, we see the same behavior.
      // The AdaptiveRecvByteBufAllocator never seemed to adapt at all. 
      bootstrap.option(ChannelOption.RCVBUF_ALLOCATOR, new FixedRecvByteBufAllocator(Conversions.megabytesToBytes(4)));
   }

   private void configureTCP()
   {
      performQuery("get_lidar_data_format", rootNode ->
      {
         pixelsPerColumn = rootNode.get("pixels_per_column").asInt();
         columnsPerFrame = rootNode.get("columns_per_frame").asInt();
         measurementBlockSize = HEADER_BLOCK_BYTES + (pixelsPerColumn * CHANNEL_DATA_BLOCK_BYTES) + MEASUREMENT_BLOCK_STATUS_BYTES;
         udpDatagramsPerFrame = columnsPerFrame / MEASUREMENT_BLOCKS_PER_UDP_DATAGRAM;
         lidarFrameSizeBytes = MEASUREMENT_BLOCKS_PER_UDP_DATAGRAM * measurementBlockSize * udpDatagramsPerFrame;

         LogTools.info("Pixels Per Column: {}, Columns Per Frame: {}, UDP datagrams per frame: {}",
                       pixelsPerColumn,
                       columnsPerFrame,
                       udpDatagramsPerFrame);
         LogTools.info("Measurement block size (B): {}, Lidar frame size (B): {}", measurementBlockSize, lidarFrameSizeBytes);

         lidarFrameByteBuffer = ByteBuffer.allocateDirect(lidarFrameSizeBytes);

         JsonNode pixelShiftNode = rootNode.get("pixel_shift_by_row");
         pixelShiftBuffer = NativeMemoryTools.allocate(pixelsPerColumn * Integer.BYTES);
         for (int i = 0; i < pixelsPerColumn; i++)
         {
            pixelShiftBuffer.putInt(pixelShiftNode.get(i).asInt());
         }
         pixelShiftBuffer.rewind();
         int[] pixelShiftsArray = new int[pixelsPerColumn];
         pixelShiftBuffer.asIntBuffer().get(pixelShiftsArray);
         LogTools.debug("Pixel shifts: {}", pixelShiftsArray);
         pixelShiftBuffer.rewind();
      });

      performQuery("get_config_param active", rootNode ->
      {
         // Lidar mode contains the frequency like 1024x20 or 2048x10
         lidarMode = rootNode.get("lidar_mode").asText();
         LogTools.info("Lidar mode: {}", lidarMode);
      });

      performQuery("get_beam_intrinsics", rootNode ->
      {
         // Convert to meters
         lidarOriginToBeamOrigin = (float) rootNode.get("lidar_origin_to_beam_origin_mm").asDouble() / 1000.0f;
         LogTools.info("Lidar origin to beam origin: {}", lidarOriginToBeamOrigin);
         JsonNode beamAltitudeAnglesNode = rootNode.get("beam_altitude_angles");
         JsonNode beamAzimuthAnglesNode = rootNode.get("beam_azimuth_angles");
         beamAltitudeAnglesBuffer = NativeMemoryTools.allocate(pixelsPerColumn * Integer.BYTES);
         beamAzimuthAnglesBuffer = NativeMemoryTools.allocate(pixelsPerColumn * Integer.BYTES);
         for (int i = 0; i < pixelsPerColumn; i++)
         {
            beamAltitudeAnglesBuffer.putFloat((float) Math.toRadians(beamAltitudeAnglesNode.get(i).asDouble()));
            beamAzimuthAnglesBuffer.putFloat((float) Math.toRadians(beamAzimuthAnglesNode.get(i).asDouble()));
         }
         beamAltitudeAnglesBuffer.rewind();
         beamAzimuthAnglesBuffer.rewind();
         float[] altitudeAnglesArray = new float[pixelsPerColumn];
         float[] azimuthAnglesArray = new float[pixelsPerColumn];
         beamAltitudeAnglesBuffer.asFloatBuffer().get(altitudeAnglesArray);
         beamAzimuthAnglesBuffer.asFloatBuffer().get(azimuthAnglesArray);
         LogTools.debug("Altitude angles: {}", altitudeAnglesArray);
         LogTools.debug("Azimuth angles: {}", azimuthAnglesArray);
         beamAltitudeAnglesBuffer.rewind();
         beamAzimuthAnglesBuffer.rewind();
      });

      performQuery("get_lidar_intrinsics", rootNode ->
      {
         JsonNode lidarToSensorTransformNode = rootNode.get("lidar_to_sensor_transform");
         spindleCenterToBaseTransform.getRotation().setAndNormalize(lidarToSensorTransformNode.get(0).asDouble(),
                                                                    lidarToSensorTransformNode.get(1).asDouble(),
                                                                    lidarToSensorTransformNode.get(2).asDouble(),
                                                                    lidarToSensorTransformNode.get(4).asDouble(),
                                                                    lidarToSensorTransformNode.get(5).asDouble(),
                                                                    lidarToSensorTransformNode.get(6).asDouble(),
                                                                    lidarToSensorTransformNode.get(8).asDouble(),
                                                                    lidarToSensorTransformNode.get(9).asDouble(),
                                                                    lidarToSensorTransformNode.get(10).asDouble());
         spindleCenterToBaseTransform.getTranslation().set(lidarToSensorTransformNode.get(3).asDouble(),
                                                           lidarToSensorTransformNode.get(7).asDouble(),
                                                           lidarToSensorTransformNode.get(11).asDouble());
      });

      LogTools.info("Ouster is initialized.");
      tcpInitialized = true;
   }

   private void performQuery(String command, Consumer<JsonNode> rootNodeConsumer)
   {
      LogTools.info("Attempting to query host " + sanitizededHostAddress + " with command " + command);

      String jsonResponse = "";
      LogTools.info("Binding to TCP port " + TCP_PORT);
      try (Socket socket = new Socket(sanitizededHostAddress, TCP_PORT))
      {
         OutputStream output = socket.getOutputStream();
         PrintWriter writer = new PrintWriter(output, true);
         writer.println(command);

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
         JsonNode rootNode = mapper.readTree(jsonResponse);
         rootNodeConsumer.accept(rootNode);
      }
      catch (JsonProcessingException jsonProcessingException)
      {
         LogTools.error(jsonProcessingException.getMessage());
      }
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

   /**
    * This getter allows access to the whole frame buffer.
    * This contains all the data the Ouster gives us in an unmodified format.
    */
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

   /**
    * This getter allows access to the pixel shift buffer which is necessary to line the rows up correctly.
    */
   public ByteBuffer getPixelShiftBuffer()
   {
      return pixelShiftBuffer;
   }

   public ByteBuffer getBeamAltitudeAnglesBuffer()
   {
      return beamAltitudeAnglesBuffer;
   }

   public ByteBuffer getBeamAzimuthAnglesBuffer()
   {
      return beamAzimuthAnglesBuffer;
   }

   public float getLidarOriginToBeamOrigin()
   {
      return lidarOriginToBeamOrigin;
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
