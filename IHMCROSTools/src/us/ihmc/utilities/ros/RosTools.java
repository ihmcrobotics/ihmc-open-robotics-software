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
import java.net.Socket;
import java.net.URI;
import java.net.URISyntaxException;
import java.net.UnknownHostException;

import javax.imageio.ImageIO;

import org.ros.node.NodeConfiguration;

import geometry_msgs.Point;
import geometry_msgs.Pose;
import geometry_msgs.Quaternion;
import geometry_msgs.Vector3;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.Vector3D32;

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
      InetAddress listenAddress = getMyIP(master);
      

      NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(listenAddress.getHostAddress(), master);
      nodeConfiguration.setMasterUri(master);

      return nodeConfiguration;
   }
   
   public static InetAddress getMyIP(URI master)
   {

      try
      {
         return getMyIP(master.getHost(), master.getPort());
      }
      catch (UnknownHostException e)
      {
         throw new RuntimeException("Unknown hostname for ROS master: " + master.getHost());
      }
      catch (IOException e)
      {
         throw new RuntimeException("Cannot connect to ROS host " + master.getHost() + "\n" + e.getMessage());
      }
   }

   public static InetAddress getMyIP(String host, int port) throws IOException, UnknownHostException
   {
      InetAddress inetAddress = InetAddress.getByName(host);
      InetAddress listenAddress;
      if (inetAddress.isLoopbackAddress())
      {
         listenAddress = inetAddress;
      }
      else
      {
         Socket testSocket = new Socket(host, port);
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
   
   

   public static InetAddress getMyIP(String rosMasterURI)
   {
      try
      {
         return getMyIP(new URI(rosMasterURI));
      }
      catch (URISyntaxException e)
      {
         throw new RuntimeException("Invalid ROS Master URI", e);
      }
   }

   
   public static void packRosQuaternionToQuat4d(Quaternion rosQuat, us.ihmc.euclid.tuple4D.Quaternion quat)
   {
      quat.set(rosQuat.getX(), rosQuat.getY(), rosQuat.getZ(), rosQuat.getW());
      
   }

   public static void packRosVector3ToVector3d(Vector3 rosVector, Vector3D vectorToPack)
   {
      vectorToPack.setX(rosVector.getX());
      vectorToPack.setY(rosVector.getY());
      vectorToPack.setZ(rosVector.getZ());
   }
   
   public static void packVector3dToGeometry_msgsVector3(Vector3D vector, Vector3 rosVectorToPack)
   {
      rosVectorToPack.setX(vector.getX());
      rosVectorToPack.setY(vector.getY());
      rosVectorToPack.setZ(vector.getZ());
   }
   
   public static void packVector3fToGeometry_msgsVector3(Vector3D32 vector, Vector3 rosVectorToPack)
   {
      rosVectorToPack.setX(vector.getX());
      rosVectorToPack.setY(vector.getY());
      rosVectorToPack.setZ(vector.getZ());
   }
   
   public static void packVector3dToGeometry_MsgPoint(Vector3D position, Point point)
   {
      point.setX(position.getX());
      point.setY(position.getY());
      point.setZ(position.getZ());
   }

   public static void packRigidBodyTransformToGeometry_msgsPose(RigidBodyTransform pelvisTransform, Pose pose)
   {
      Vector3D point = new Vector3D();
      pelvisTransform.getTranslation(point);

      us.ihmc.euclid.tuple4D.Quaternion rotation = new us.ihmc.euclid.tuple4D.Quaternion();
      pelvisTransform.getRotation(rotation);
      
      packVector3dAndQuat4dToGeometry_msgsPose(point, rotation, pose);
   }
   
   public static void packVector3dAndQuat4dToGeometry_msgsPose(Vector3D point, us.ihmc.euclid.tuple4D.Quaternion rotation, Pose pose)
   {
      RosTools.packVector3dToGeometry_MsgPoint(point, pose.getPosition());
      RosTools.packQuat4dToGeometry_msgsQuaternion(rotation, pose.getOrientation());
   }

   private static void packQuat4dToGeometry_msgsQuaternion(us.ihmc.euclid.tuple4D.Quaternion quat, Quaternion orientation)
   {
      orientation.setW(quat.getS());
      orientation.setX(quat.getX());
      orientation.setY(quat.getY());
      orientation.setZ(quat.getZ());
   }
 }
