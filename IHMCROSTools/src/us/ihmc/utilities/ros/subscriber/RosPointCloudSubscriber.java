package us.ihmc.utilities.ros.subscriber;

import java.awt.Color;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import us.ihmc.euclid.tuple3D.Point3D;

import sensor_msgs.PointCloud2;
import us.ihmc.utilities.ros.types.PointType;

public abstract class RosPointCloudSubscriber extends AbstractRosTopicSubscriber<PointCloud2>
{
   private boolean DEBUG = false;

   public RosPointCloudSubscriber()
   {
      super(sensor_msgs.PointCloud2._TYPE);
   }

   /*
    * rostopic echo /ibeo/points /ibeo_link header: seq: 38993 stamp: secs:
    * 1401924668 nsecs: 468290060 frame_id: /ibeo_link height: 1 width: 2025
    * fields: - name: x offset: 0 datatype: 7 count: 1 - name: y offset: 4
    * datatype: 7 count: 1 - name: z offset: 8 datatype: 7 count: 1
    * is_bigendian: False point_step: 16 row_step: 32400 data: [172, 111, 184,
    * 64, 115, 205, 219, 64, 127, 44, 124, 190, 0, 0, 128, 63, 122, 230, 184,
    * 64, 9, 91, 220, 64, 108, 116, 248, 189, 0, 0, 128, 63, 251, 94, 131, 64,
    * 5, 32, 151, 64, 182, 14, 50, 190, 0, 0, 128, 63, 104, 76, 133, 64, 164,
    * 87, 153, 64, 190, 177, 179, 189, 0, 0, 128, 63, 77, 37, 182, 64, 22, 75,
    * 202, 64, 78, 220, 116, 190, 0, 0, 128, 63, 26, 69, 183, 64, 185, 138, 203,
    * 64, 181, 224, 247, 189, 0, 0, 128, 63, 46, 240, 181, 64, 216, 26, 195, 64,
    * 106, 181, 114, 190, 0, 0, 128, 63, 183, 165, 182, 64, 132, 221, 195, 64,
    * 80, 224
    */

   /*
    * 
    * tingfan@unknownid-All-Series:~/gworkspace/IHMCPerception/catkin_ws/src/
    * lidar_to_point_cloud_transformer$ rostopic echo
    * /multisense/image_points2_color |more header: seq: 1417800 stamp: secs:
    * 1423988842 nsecs: 953257000 frame_id:
    * /multisense/left_camera_optical_frame height: 1 width: 442515 fields: -
    * name: x offset: 0 datatype: 7 count: 1 - name: y offset: 4 datatype: 7
    * count: 1 - name: z offset: 8 datatype: 7 count: 1 - name: rgb offset: 12
    * datatype: 7 count: 1 is_bigendian: False point_step: 16 row_step: 7080240
    * data: [33, 215, 239, 63, 230, 81, 175, 191, 130, 209, 59, 64, 60, 80, 69,
    * 0, 22, 132, 240, 63, 230, 81, 175, 191, 130, 209, 59, 64, 66, 91, 73, 0,
    * 11, 49, 241, 63, 230, 81, 175, 191, 130, 209
    */
   
   public static class UnpackedPointCloud
   {
      Point3D[] points = null;
      float[] intensities = null;
      Color[] pointColors = null;
      PointType pointType = null;
      int width;
      int height;
      
      public UnpackedPointCloud()
      {
         
      }
      
      public UnpackedPointCloud(int width, int height, PointType pointType, Point3D[] points, Color[] pointColors)
      {
         this.pointColors=pointColors;
         this.points = points;
         this.width=width;
         this.height=height;
         this.pointType=pointType;
      }
      


      public int getWidth()
      {
         return width;
      }

      public int getHeight()
      {
         return height;
      }

      public Point3D[] getPoints()
      {
         return points;
      }

      public float[] getIntensities()
      {
         return intensities;
      }

      public Color[] getPointColors()
      {
         return pointColors;
      }

      public PointType getPointType()
      {
         return pointType;
      }

      public float[] getXYZRGB()
      {
         float[] xyzrgb = new float[points.length * 4];

         int counter = 0;
         for (int i = 0; i < points.length; i++)
         {
            xyzrgb[counter++] = (float) points[i].getX();
            xyzrgb[counter++] = (float) points[i].getY();
            xyzrgb[counter++] = (float) points[i].getZ();
            xyzrgb[counter++] = Float.intBitsToFloat(pointColors[i].getRGB());
         }
         return xyzrgb;
      }
   }

   public static UnpackedPointCloud unpackPointsAndIntensities(PointCloud2 pointCloud)
   {

      UnpackedPointCloud packet = new UnpackedPointCloud();
      int numberOfPoints = pointCloud.getWidth() * pointCloud.getHeight();
      packet.points = new Point3D[numberOfPoints];
      packet.pointType = PointType.fromFromFieldNames(pointCloud.getFields());
      packet.width = pointCloud.getWidth();
      packet.height = pointCloud.getHeight();

      switch (packet.pointType)
      {
      case XYZI:
         packet.intensities = new float[numberOfPoints];

         break;

      case XYZRGB:
    	  packet.pointColors = new Color[numberOfPoints];
    	  break;
    	  
      case XYZ:
    	  break;
      }

      int offset = pointCloud.getData().arrayOffset();
      int pointStep = pointCloud.getPointStep();

      byte[] array = pointCloud.getData().array();
	ByteBuffer byteBuffer = ByteBuffer.wrap(array, offset, numberOfPoints * pointStep);

      if (pointCloud.getIsBigendian())
         byteBuffer.order(ByteOrder.BIG_ENDIAN);
      else
         byteBuffer.order(ByteOrder.LITTLE_ENDIAN);

      for (int i = 0; i < numberOfPoints; i++)
      {
         byteBuffer.position(i * pointStep + offset);
         float x = byteBuffer.getFloat();
         float y = byteBuffer.getFloat();
         float z = byteBuffer.getFloat();

         packet.points[i] = new Point3D(x, y, z);

         switch (packet.pointType)
         {
         case XYZI:
            packet.intensities[i] = byteBuffer.getFloat();
            break;

         case XYZRGB:
            int r = 0,
            g = 0,
            b = 0,
            a = 0;
            try
            {
               b = byteToUnsignedInt(byteBuffer.get());
               g = byteToUnsignedInt(byteBuffer.get());
               r = byteToUnsignedInt(byteBuffer.get());
               a = byteToUnsignedInt(byteBuffer.get());
               packet.pointColors[i] = new Color(r, g, b);
               //               System.out.println(r + " " + g + " " + b + " " + a);
            }
            catch (RuntimeException e)
            {
               System.out.println(g);
            }
		case XYZ:
			break;
         }
      }

      return packet;
   }

   private static int byteToUnsignedInt(byte b)
   {
      return ((int) b) & 0xff;
   }

   @Override
   public abstract void onNewMessage(PointCloud2 pointCloud);

}
