package us.ihmc.perception.ouster;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import io.netty.buffer.ByteBuf;
import io.netty.buffer.Unpooled;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.perception.tools.NativeMemoryTools;
import us.ihmc.tools.thread.RestartableThread;

import java.io.*;
import java.net.*;
import java.nio.ByteBuffer;
import java.time.Instant;
import java.util.function.Consumer;

/**
 * This class is written to do no operations on incoming Ouster data for maximum performance.
 * It just lines the incoming data up into a single buffer representing one full frame. It also
 * does so on the event that the UDP datagrams are received by Netty for the fastest possible
 * response. All operations on this data should be done externally by OpenCL kernels.
 * <p>
 * Ouster Firmware User Manual: https://data.ouster.io/downloads/software-user-manual/firmware-user-manual-v2.3.0.pdf
 * Software User Manual: https://data.ouster.io/downloads/software-user-manual/software-user-manual-v2p0.pdf
 * <p>
 * To connect to the Ouster, type in os-122221003063.local into the browser, finding the serial number
 * from the Confluence page: https://confluence.ihmc.us/display/PER/In-House+Perception+Sensors+Tracker
 * <p>
 * Apparently it is better to use this magical command from the user manual:
 * avahi-browse -lrt _roger._tcp
 * <p>
 * To test, use the GNU netcat command:
 * netcat -ul 7502
 *
 * <p>
 * Lidar incoming data format:
 * </p>
 *
 * <img src="https://www.ihmc.us/wp-content/uploads/2023/02/OusterLidarDataFormat.png" width="800">
 */
public class OusterNetServer
{
   public static final int TCP_PORT = 7501;
   public static final int UDP_PORT = 7502;

   /**
    * Ouster produces ranges as unsigned integers in millimeters, so this is just a conversion to meters.
    */
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

   private static final int RECEIVE_PACKET_TIMEOUT = 3000;

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
   private int columnsPerPacket;
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

   private volatile boolean tcpInitialized = false;
   private String sanitizededHostAddress;

   private Runnable onFrameReceived = null;
   private Instant aquisitionInstant;
   private ByteBuffer lidarFrameByteBuffer;
   private long nextExpectedMeasurementID = -1;
   private String lidarMode;

   private boolean printedWarning = false;

   private DatagramSocket udpSocket;
   private byte[] udpBuffer = new byte[0];
   private final RestartableThread udpServerThread;
   private volatile boolean initialized = false;

   public OusterNetServer()
   {
      udpServerThread = new RestartableThread(getClass().getSimpleName(), this::run);
   }

   public void run()
   {
      if (!initialized)
      {
         try
         {
            LogTools.info("Binding to UDP port: " + UDP_PORT);
            udpSocket = new DatagramSocket(UDP_PORT);
            udpSocket.setSoTimeout(RECEIVE_PACKET_TIMEOUT);
            initialized = true;
         }
         catch (SocketException e)
         {
            LogTools.error(e);
         }
      }

      DatagramPacket packet = new DatagramPacket(udpBuffer, udpBuffer.length);
      try
      {
         udpSocket.receive(packet);
      }
      catch (SocketTimeoutException timeoutException)
      {
         LogTools.warn("Ouster socket did not receive a packet.");
         return;
      }
      catch (IOException e)
      {
         LogTools.error(e);
      }

      if (!tcpInitialized)
      {
         // Address looks like "/192.168.x.x" so we just discard the '/'
         sanitizededHostAddress = packet.getAddress().toString().substring(1);

         // Blocking
         configureTCP();

         int bufferSize = measurementBlockSize * columnsPerFrame;
         udpBuffer = new byte[bufferSize];

         lidarFrameByteBuffer = ByteBuffer.allocateDirect(bufferSize);

         // Ignore the current packet and keep moving since we recreated the buffer
         return;
      }

      aquisitionInstant = Instant.now();

      ByteBuf content = Unpooled.wrappedBuffer(packet.getData());

      // Ouster data is little endian
      int measurementID = content.getUnsignedShortLE(TIMESTAMP_BYTES);
      if (nextExpectedMeasurementID > 0 && measurementID != nextExpectedMeasurementID && !printedWarning)
      {
         printedWarning = true;
         LogTools.warn("UDP datagram skipped! Expected measurement ID {} but was {}. Skipping this warning in the future",
                       nextExpectedMeasurementID,
                       measurementID);
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

      content.release();
   }

   private void configureTCP()
   {
      LogTools.info("Querying Ouster parameters...");

      performQuery("get_lidar_data_format", rootNode ->
      {
         pixelsPerColumn = rootNode.get("pixels_per_column").asInt();
         columnsPerPacket = rootNode.get("columns_per_packet").asInt();
         columnsPerFrame = rootNode.get("columns_per_frame").asInt();
         measurementBlockSize = HEADER_BLOCK_BYTES + (pixelsPerColumn * CHANNEL_DATA_BLOCK_BYTES) + MEASUREMENT_BLOCK_STATUS_BYTES;
         udpDatagramsPerFrame = columnsPerFrame / MEASUREMENT_BLOCKS_PER_UDP_DATAGRAM;
         lidarFrameSizeBytes = MEASUREMENT_BLOCKS_PER_UDP_DATAGRAM * measurementBlockSize * udpDatagramsPerFrame;

         LogTools.info("Pixels Per Column: {}, Columns Per Frame: {}, UDP datagrams per frame: {}", pixelsPerColumn, columnsPerFrame, udpDatagramsPerFrame);
         LogTools.info("Measurement block size (B): {}, Lidar frame size (B): {}", measurementBlockSize, lidarFrameSizeBytes);

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
      LogTools.info("Binding to TCP port: " + TCP_PORT);
      try (Socket socket = new Socket(sanitizededHostAddress, TCP_PORT))
      {
         socket.setSoTimeout(RECEIVE_PACKET_TIMEOUT);
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
         LogTools.info("Unbound from TCP port: " + TCP_PORT);
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

   public void start()
   {
      udpServerThread.start();
   }

   public void stop()
   {
      udpServerThread.stop();
   }

   public void destroy()
   {
      udpServerThread.blockingStop();

      if (udpSocket != null)
         udpSocket.close();
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
