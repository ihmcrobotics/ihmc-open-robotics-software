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
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import javax.imageio.ImageIO;

import boofcv.struct.calib.CameraPinholeBrown;
import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.node.NodeConfiguration;

import geometry_msgs.Point;
import geometry_msgs.Pose;
import geometry_msgs.Quaternion;
import geometry_msgs.Vector3;
import sensor_msgs.CameraInfo;
import sensor_msgs.Image;
import sensor_msgs.PointCloud2;
import sensor_msgs.PointField;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.commons.robotics.robotSide.SideDependentList;
import us.ihmc.commons.StringTools;

public class RosTools
{
   public static final String MULTISENSE_VIDEO = "/multisense/left/image_rect_color/compressed";
   public static final String MULTISENSE_CAMERA_INFO = "/multisense/left/image_rect_color/camera_info";
   public static final String MULTISENSE_PPS = "/multisense/stamped_pps";
   public static final String D435_VIDEO = "/camera/color/image_raw";
   public static final String D435_VIDEO_COMPRESSED = "/camera/color/image_raw/compressed";
   public static final String D435_CAMERA_INFO = "/camera/color/camera_info";
   public static final String D435_POINT_CLOUD = "/camera/depth/color/points";
   public static final String D435_DEPTH = "/camera/depth/image_rect_raw";
   public static final String D435_DEPTH_CAMERA_INFO = "/camera/depth/camera_info";
   public static final String L515_VIDEO = "/chest_l515/color/image_raw";
   public static final String L515_COMPRESSED_VIDEO = "/chest_l515/color/image_raw/compressed";
   public static final String L515_DEPTH = "/chest_l515/depth/image_rect_raw";
   public static final String L515_POINT_CLOUD = "/chest_l515/depth/color/points";
   public static final String OUSTER_POINT_CLOUD = "/os_cloud_node/points";
   public static final String MAPSENSE_POINT_CLOUD = "/mapsense/color/points";
   public static final String BLACKFLY_LEFT_VIDEO = "/blackfly/left/image_color";
   public static final String BLACKFLY_RIGHT_VIDEO = "/blackfly/right/image_color";
   public static final SideDependentList<String> BLACKFLY_VIDEO_TOPICS = new SideDependentList<>(BLACKFLY_LEFT_VIDEO, BLACKFLY_RIGHT_VIDEO);
   public static final String SLAM_POSE = "/mapsense/slam/pose";
   public static final String SEMANTIC_TARGET_POSE = "/semantic/target/pose";
   public static final String SEMANTIC_TARGET_CLOUD = "/semantic/object/points";
   public static final String SEMANTIC_MASK = "/semantic/deeplab/mask";
   public static final String L515_COLOR_CAMERA_INFO = "/chest_l515/color/camera_info";
   public static final String L515_DEPTH_CAMERA_INFO = "/chest_l515/depth/camera_info";
   public static final String MAPSENSE_DEPTH_IMAGE = L515_DEPTH;
   public static final String MAPSENSE_DEPTH_CAMERA_INFO = L515_DEPTH_CAMERA_INFO;
   public static final String MAPSENSE_REGIONS = "/mapsense/planar_regions";
   public static final String MAPSENSE_CONFIGURATION = "/map/config";
   // See https://www.stereolabs.com/docs/ros/zed-node/
   public static final String ZED2_LEFT_EYE_VIDEO_COMPRESSED = "/zed/zed_node/left/image_rect_color/compressed";
   public static final String ZED2_LEFT_EYE_VIDEO = "/zed/color/left/image_raw";
   public static final String ZED2_RIGHT_EYE_VIDEO = "/zed/color/right/image_raw";
   public static final String ZED2_POINT_CLOUD = "/zed/zed_node/point_cloud/cloud_registered";
   public static final String LOGITECH_BRIO_LEFT_COMPRESSED = "/logitech/left/cam/color/image_raw/compressed";
   public static final String LOGITECH_BRIO_RIGHT_COMPRESSED = "/logitech/right/cam/color/image_raw/compressed";

   public static RosMainNode createRosNode(String uri, String name)
   {
      return createRosNode(ExceptionTools.handle(() -> new URI(uri), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE), name);
   }

   public static RosMainNode createRosNode(URI uri, String name)
   {
      LogTools.info("Creating ROS 1 node. name: {} URI: {}", name, uri);
      return new RosMainNode(uri, name);
   }

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
      return bufferedImageFromRosMessageJpeg(imageMessage);
   }

   public static BufferedImage bufferedImageFromRosMessageJpeg(sensor_msgs.CompressedImage imageMessage)
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

   /**
    * Returns a ByteBuffer that starts with the data part.
    */
   public static ByteBuffer sliceNettyBuffer(ChannelBuffer channelBuffer)
   {
      ByteBuffer originalByteBuffer = channelBuffer.toByteBuffer();
      int arrayOffset = channelBuffer.arrayOffset();
      originalByteBuffer.position(arrayOffset);
      return originalByteBuffer.slice();
   }

   public static CameraPinholeBrown cameraIntrisicsFromCameraInfo(CameraInfo cameraInfo)
   {
      CameraPinholeBrown cameraPinholeBrown = new CameraPinholeBrown();
      double[] P = cameraInfo.getP();
      cameraPinholeBrown.fx = P[0];
      cameraPinholeBrown.skew = P[1];
      cameraPinholeBrown.cx = P[2];
      cameraPinholeBrown.fy = P[5];
      cameraPinholeBrown.cy = P[6];
      cameraPinholeBrown.width = cameraInfo.getWidth();
      cameraPinholeBrown.height = cameraInfo.getHeight();
      return cameraPinholeBrown;
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

   public static ByteBuffer wrapPointCloud2Array(PointCloud2 pointCloud2)
   {
      int numberOfPoints = pointCloud2.getWidth() * pointCloud2.getHeight();
      int offset = pointCloud2.getData().arrayOffset();
      int pointStep = pointCloud2.getPointStep();

      ByteBuffer byteBuffer = ByteBuffer.wrap(pointCloud2.getData().array(), offset, numberOfPoints * pointStep);

      if (pointCloud2.getIsBigendian())
         byteBuffer.order(ByteOrder.BIG_ENDIAN);
      else
         byteBuffer.order(ByteOrder.LITTLE_ENDIAN);

      return byteBuffer;
   }

   public static void printPointCloud2Info(String name, PointCloud2 pointCloud2)
   {
      LogTools.info("PointCloud2 Name: {}", name);
      LogTools.info(StringTools.format("Height: {} Width: {} (Total: {}) Bigendian: {} Point step: {} Row step: {}",
                                       pointCloud2.getHeight(),
                                       pointCloud2.getWidth(),
                                       pointCloud2.getHeight() * pointCloud2.getWidth(),
                                       pointCloud2.getIsBigendian(),
                                       pointCloud2.getPointStep(),
                                       pointCloud2.getRowStep()));
      int i = 0;
      for (PointField field : pointCloud2.getFields())
      {
         String datatype = "";
         if (field.getDatatype() == PointField.INT8)
            datatype = "INT8";
         if (field.getDatatype() == PointField.UINT8)
            datatype = "UINT8";
         if (field.getDatatype() == PointField.INT16)
            datatype = "INT16";
         if (field.getDatatype() == PointField.UINT16)
            datatype = "UINT16";
         if (field.getDatatype() == PointField.INT32)
            datatype = "INT32";
         if (field.getDatatype() == PointField.UINT32)
            datatype = "UINT32";
         if (field.getDatatype() == PointField.FLOAT32)
            datatype = "FLOAT32";
         if (field.getDatatype() == PointField.FLOAT64)
            datatype = "FLOAT64";

         LogTools.info(StringTools.format("Field {} Name: {} Count: {} Offset: {} Type: {}",
                                          i,
                                          field.getName(),
                                          field.getCount(),
                                          field.getOffset(),
                                          datatype));
         i++;
      }
   }

   public static void printImageInfo(String name, Image image)
   {
      LogTools.info("Image Name: {}", name);
      LogTools.info(StringTools.format("Height: {} Width: {} (Total: {}) Encoding: {} Bigendian: {} Step: {}",
                                       image.getHeight(),
                                       image.getWidth(),
                                       image.getHeight() * image.getWidth(),
                                       image.getEncoding(),
                                       image.getIsBigendian(),
                                       image.getStep()));
   }

   public static void toEuclid(Pose rosPose, Pose3DBasics euclidPose)
   {
      euclidPose.getPosition().set(rosPose.getPosition().getX(),
                                   rosPose.getPosition().getY(),
                                   rosPose.getPosition().getZ());
      euclidPose.getOrientation().set(rosPose.getOrientation().getX(),
                                      rosPose.getOrientation().getY(),
                                      rosPose.getOrientation().getZ(),
                                      rosPose.getOrientation().getW());
   }

   public static void toRos(Pose3DReadOnly euclidPose, Pose rosPose)
   {
      rosPose.getPosition().setX(euclidPose.getPosition().getX());
      rosPose.getPosition().setY(euclidPose.getPosition().getY());
      rosPose.getPosition().setZ(euclidPose.getPosition().getZ());
      rosPose.getOrientation().setX(euclidPose.getOrientation().getX());
      rosPose.getOrientation().setY(euclidPose.getOrientation().getY());
      rosPose.getOrientation().setZ(euclidPose.getOrientation().getZ());
      rosPose.getOrientation().setW(euclidPose.getOrientation().getS());
   }

   public static void packRosQuaternionToEuclidQuaternion(Quaternion rosQuat, QuaternionBasics quat)
   {
      quat.set(rosQuat.getX(), rosQuat.getY(), rosQuat.getZ(), rosQuat.getW());
   }

   public static void packRosVector3ToEuclidTuple3D(Vector3 rosVector, Tuple3DBasics vectorToPack)
   {
      vectorToPack.setX(rosVector.getX());
      vectorToPack.setY(rosVector.getY());
      vectorToPack.setZ(rosVector.getZ());
   }

   public static void packEuclidTuple3DToGeometry_msgsVector3(Tuple3DReadOnly vector, Vector3 rosVectorToPack)
   {
      rosVectorToPack.setX(vector.getX());
      rosVectorToPack.setY(vector.getY());
      rosVectorToPack.setZ(vector.getZ());
   }

   public static void packEuclidTuple3DToGeometry_MsgPoint(Tuple3DReadOnly position, Point point)
   {
      point.setX(position.getX());
      point.setY(position.getY());
      point.setZ(position.getZ());
   }

   public static void packEuclidRigidBodyTransformToGeometry_msgsPose(RigidBodyTransform pelvisTransform, Pose pose)
   {
      Vector3D point = new Vector3D();
      point.set(pelvisTransform.getTranslation());

      us.ihmc.euclid.tuple4D.Quaternion rotation = new us.ihmc.euclid.tuple4D.Quaternion();
      rotation.set(pelvisTransform.getRotation());

      packEuclidTuple3DAndQuaternionToGeometry_msgsPose(point, rotation, pose);
   }

   public static void packEuclidTuple3DAndQuaternionToGeometry_msgsPose(Tuple3DReadOnly point, QuaternionReadOnly rotation, Pose pose)
   {
      RosTools.packEuclidTuple3DToGeometry_MsgPoint(point, pose.getPosition());
      RosTools.packQuat4dToGeometry_msgsQuaternion(rotation, pose.getOrientation());
   }

   private static void packQuat4dToGeometry_msgsQuaternion(QuaternionReadOnly quat, Quaternion orientation)
   {
      orientation.setW(quat.getS());
      orientation.setX(quat.getX());
      orientation.setY(quat.getY());
      orientation.setZ(quat.getZ());
   }
}
