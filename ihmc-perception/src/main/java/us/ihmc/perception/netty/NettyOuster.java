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
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;

import java.io.*;
import java.net.Socket;
import java.net.UnknownHostException;
import java.time.Instant;

/**
 * Ouster Firmware User Manual: https://data.ouster.io/downloads/software-user-manual/firmware-user-manual-v2.3.0.pdf
 * 
 * To test, use the GNU netcat command:
 * netcat -ul 7502
 */
public class NettyOuster
{
   public static final int TCP_PORT = 7501;
   public static final int UDP_PORT = 7502;
   public static final int MAX_PACKET_SIZE = 24896; //Defined by software user manual
   private static int actualLidarPacketSize;

   // -- LIDAR Information --
   private int pixelsPerColumn;
   private int columnsPerFrame;
   private int columnsPerPacket;
   private int[] pixelShift;
   // -- End LIDAR Information --

   private boolean tcpInitialized = false;

   private final EventLoopGroup group;
   private final Bootstrap bootstrap;
   private BytedecoImage depthImageMeters;
   private Runnable onFrameReceived = null;
   private Instant aquisitionInstant;

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

            if (!tcpInitialized)
            {
               if (!configureTCP(packet.sender().getAddress().toString().substring(1)))
               { //Address looks like "/192.168.x.x" so we just discard the '/'
                  LogTools.error("Failed to initialize Ouster using TCP API.");
                  return;
               }
               depthImageMeters = new BytedecoImage(columnsPerFrame, pixelsPerColumn, opencv_core.CV_32FC1);
               depthImageMeters.getBytedecoOpenCVMat().setTo(new Mat(0.0f)); //Initialize matrix to 0

               actualLidarPacketSize = columnsPerPacket * (16 + (pixelsPerColumn * 12) + 4);
            }

            ByteBuf bufferedData = packet.content().readBytes(actualLidarPacketSize);

            int i = 0;
            for (int columnNumber = 0; columnNumber < columnsPerPacket; columnNumber++)
            {
               i += 8; //Timestamp
               int measurementID = (int) extractValue(bufferedData, i, 2);
               i += 8; //Measurement ID (above), Frame ID, Encoder Count

               long[] range = new long[pixelsPerColumn];
               for (int blockID = 0; blockID < pixelsPerColumn; blockID++)
               { //Note that blockID is useful data here
                  range[blockID] = extractValue(bufferedData, i, 4);
                  i += 12; //Range, and other values we don't care about
               }

               boolean dataOkay = extractValue(bufferedData, i, 4) == 0xFFFFFFFFL;
               i += 4;

               if (dataOkay)
               {
                  for (int k = 0; k < 64; k++)
                  {
                     float rangeScaled = range[k] / 1000.0F;
                     if (rangeScaled > 30.0)
                     {
                        rangeScaled = 0.0f;
                     }

                     //Calculate column by adding the reported row pixel shift to the measurement ID, and then adjusting for over/underflow
                     int column = (measurementID + pixelShift[k]) % columnsPerFrame;
                     depthImageMeters.getBytedecoOpenCVMat().ptr(k, column).putFloat(rangeScaled);
                  }
               }
            }

            if (onFrameReceived != null)
               onFrameReceived.run();

            bufferedData.release();
         }
      });

      bootstrap.option(ChannelOption.RCVBUF_ALLOCATOR, new FixedRecvByteBufAllocator(MAX_PACKET_SIZE));
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
         pixelShift = new int[pixelsPerColumn];

         JsonNode pixelShiftNode = root.get("pixel_shift_by_row");
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
    * Java doesn't have support for unsigned 32-bit integers* so we have to use a long instead
    */
   private long extractValue(ByteBuf nettyByteBuf, int index, int numberOfBytes)
   {
      int shift = index % 4;
      int modIndex = index - (index % 4);

      //Use little endian accessors (getUnsignedIntLE, not getUnsignedInt)
      long val = nettyByteBuf.getUnsignedIntLE(modIndex);
      val >>= shift;
      switch (numberOfBytes)
      {
         case 1:
            val = val & 0xFF;
            break;
         case 2:
            val = val & 0xFFFF;
            break;
         case 4:
            break;
         case 8:
            val += nettyByteBuf.getUnsignedIntLE(modIndex + 4) << 32;
            break;
         default:
            return -1;
      }

      return val;
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
