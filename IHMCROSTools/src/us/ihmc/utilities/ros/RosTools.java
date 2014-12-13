package us.ihmc.utilities.ros;

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
import java.net.URI;
import java.net.URISyntaxException;

import javax.imageio.ImageIO;

import org.ros.node.NodeConfiguration;

import us.ihmc.communication.net.NetworkTools;

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

   public static NodeConfiguration createNodeConfiguration(URI master)
   {
      InetAddress listenAddress = NetworkTools.getMyIP(master);
      

      NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(listenAddress.getHostAddress(), master);
      nodeConfiguration.setMasterUri(master);

      return nodeConfiguration;
   }

   public static InetAddress getMyIP(String rosMasterURI)
   {
      try
      {
         return NetworkTools.getMyIP(new URI(rosMasterURI));
      }
      catch (URISyntaxException e)
      {
         throw new RuntimeException(e);
      }
   }

}
