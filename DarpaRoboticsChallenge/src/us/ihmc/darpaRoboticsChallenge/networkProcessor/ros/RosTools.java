package us.ihmc.darpaRoboticsChallenge.networkProcessor.ros;

import java.awt.image.BufferedImage;
import java.awt.image.ColorModel;
import java.awt.image.DataBuffer;
import java.awt.image.DataBufferByte;
import java.awt.image.Raster;
import java.awt.image.SampleModel;
import java.awt.image.WritableRaster;
import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.net.InetAddress;
import java.net.Socket;
import java.net.URI;
import java.net.URISyntaxException;
import java.net.UnknownHostException;

import javax.imageio.ImageIO;

import org.ros.address.InetAddressFactory;
import org.ros.node.NodeConfiguration;

public class RosTools
{

   public static BufferedImage bufferedImageFromByteArrayJpeg(ColorModel colorModel, byte[] payload, int width, int height)
   {
      BufferedImage ret = new BufferedImage(width, height, BufferedImage.TYPE_3BYTE_BGR);
      DataBuffer dataBuffer = new DataBufferByte(payload, payload.length, 0);
      SampleModel sampleModel = colorModel.createCompatibleSampleModel(width, height);

      WritableRaster raster = Raster.createWritableRaster(sampleModel, dataBuffer, null);
      ret.setData(raster);

      return ret;
   }

   public static BufferedImage bufferedImageFromRosMessageRaw(ColorModel colorModel, sensor_msgs.Image imageMessage)
   {
      int width = imageMessage.getWidth();
      int height = imageMessage.getHeight();

      byte[] payload = imageMessage.getData().array();
      BufferedImage ret = new BufferedImage(width, height, BufferedImage.TYPE_3BYTE_BGR);
      DataBuffer dataBuffer = new DataBufferByte(payload, payload.length, imageMessage.getData().arrayOffset());
      SampleModel sampleModel = colorModel.createCompatibleSampleModel(width, height);

      WritableRaster raster = Raster.createWritableRaster(sampleModel, dataBuffer, null);
      ret.setData(raster);

      return ret;
   }

   public static BufferedImage bufferedImageFromRosMessageJpeg(ColorModel colorModel, sensor_msgs.CompressedImage imageMessage)
   {

      BufferedImage ret = null;
      byte[] payload = imageMessage.getData().array();
      try
      {
         int offset = imageMessage.getData().arrayOffset();
         ret = ImageIO.read(new ByteArrayInputStream(payload, offset, payload.length - offset));
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

      return ret;
   }

   public static InetAddress getMyIP(URI master)
   {
      try
      {
         InetAddress inetAddress = InetAddress.getByName(master.getHost());
         InetAddress listenAddress;
         if (inetAddress.isLoopbackAddress())
         {
            listenAddress = InetAddressFactory.newLoopback();
         }
         else
         {
            Socket testSocket = new Socket(master.getHost(), master.getPort());
            listenAddress = testSocket.getLocalAddress();
            testSocket.close();
            /*
             * At first the following code looks like a better solution, however
             * InetAddress.isReachable does not work without being a super user.
             */

            //            // Find first local address the master host is reachable from
            //            final Enumeration<NetworkInterface> networkInterfaces = NetworkInterface.getNetworkInterfaces();
            //            
            //            while(networkInterfaces.hasMoreElements()) {
            //               NetworkInterface iface = networkInterfaces.nextElement();
            //               
            //               if(iface.isLoopback())
            //               {
            //                  continue;
            //               }
            //               
            //               if(inetAddress.isReachable(iface, 0, 100))
            //               {
            //                  for(InterfaceAddress ifaceAddr : iface.getInterfaceAddresses())
            //                  {
            //                     if(ifaceAddr.getAddress().getAddress().length == 4)
            //                     {
            //                        listenAddress = ifaceAddr.getAddress();
            //                     }
            //                  }
            //               }
            //             }

         }
         return listenAddress;
      }
      catch (UnknownHostException e)
      {
         throw new RuntimeException("Invalid hostname: " + master.getHost());
      }
      catch (IOException e)
      {
         throw new RuntimeException("Cannot connect to ROS\n" + e.getMessage());
      }
      
   }
   
   public static NodeConfiguration createNodeConfiguration(URI master)
   {
      InetAddress listenAddress = getMyIP(master);
      

      NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(listenAddress.getHostAddress(), master);
      nodeConfiguration.setMasterUri(master);

      return nodeConfiguration;
   }

   public static InetAddress getMyIP(String rosMasterURI)
   {
      try
      {
         return getMyIP(new URI(rosMasterURI));
      }
      catch (URISyntaxException e)
      {
         throw new RuntimeException(e);
      }
   }

}
