package us.ihmc.utilities.ros.publisher;

import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.LittleEndianHeapChannelBuffer;
import org.ros.message.Time;

import sensor_msgs.PointCloud2;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.dataStructures.MutableColor;
import us.ihmc.utilities.ros.types.PointType;

public class RosPointCloudPublisher extends RosTopicPublisher<PointCloud2>
{
 

   PointType pointType;

   public RosPointCloudPublisher(PointType pointType, boolean latch)
   {
      super(PointCloud2._TYPE, latch);
      this.pointType = pointType;
   }
   


   public void publish(Point3D[] points, float[] intensities, String frameId)
   {
      PointCloud2 message = getMessage();
      message.getHeader().setFrameId(frameId);
      message.getHeader().setStamp(Time.fromMillis(System.currentTimeMillis()));
      message.setHeight(1);
      message.setWidth(points.length);
      message.setPointStep(pointType.getPointStep());
      int dataLength = pointType.getPointStep() * points.length;
      message.setRowStep(dataLength);
      message.setIsBigendian(false);
      message.setIsDense(true);
      message.setFields(pointType.getPointField());
      
      ChannelBuffer buffer = new LittleEndianHeapChannelBuffer(dataLength);
      for(int i=0;i<points.length;i++)
      {
         buffer.writeFloat((float)points[i].getX());
         buffer.writeFloat((float)points[i].getY());
         buffer.writeFloat((float)points[i].getZ());
         buffer.writeFloat(intensities[i]);
      }
      message.setData(buffer);
      
      publish(message);
   }



   public void publish(Point3D[] points, MutableColor[] colors, String frameId)
   {
      PointCloud2 message = getMessage();
      message.getHeader().setFrameId(frameId);
      message.getHeader().setStamp(Time.fromMillis(System.currentTimeMillis()));
      message.setHeight(1);
      message.setWidth(points.length);
      message.setPointStep(pointType.getPointStep());
      int dataLength = pointType.getPointStep() * points.length;
      message.setRowStep(dataLength);
      message.setIsBigendian(false);
      message.setIsDense(true);
      message.setFields(pointType.getPointField());
      
      ChannelBuffer buffer = new LittleEndianHeapChannelBuffer(dataLength);
      for(int i=0;i<points.length;i++)
      {
         buffer.writeFloat((float)points[i].getX());
         buffer.writeFloat((float)points[i].getY());
         buffer.writeFloat((float)points[i].getZ());
         buffer.writeByte((int)colors[i].getZ());
         buffer.writeByte((int)colors[i].getY());
         buffer.writeByte((int)colors[i].getX());
         buffer.writeByte(0); //dummy;
      }
      message.setData(buffer);
      
      publish(message);
      
   }

   public void publish(Point3D[] points, MutableColor color, String frameId)
   {
      PointCloud2 message = getMessage();
      message.getHeader().setFrameId(frameId);
      message.getHeader().setStamp(Time.fromMillis(System.currentTimeMillis()));
      message.setHeight(1);
      message.setWidth(points.length);
      message.setPointStep(pointType.getPointStep());
      int dataLength = pointType.getPointStep() * points.length;
      message.setRowStep(dataLength);
      message.setIsBigendian(false);
      message.setIsDense(true);
      message.setFields(pointType.getPointField());

      ChannelBuffer buffer = new LittleEndianHeapChannelBuffer(dataLength);
      for(int i=0;i<points.length;i++)
      {
         buffer.writeFloat((float)points[i].getX());
         buffer.writeFloat((float)points[i].getY());
         buffer.writeFloat((float)points[i].getZ());
         buffer.writeByte((int)color.getZ());
         buffer.writeByte((int)color.getY());
         buffer.writeByte((int)color.getX());
         buffer.writeByte(0); //dummy;
      }
      message.setData(buffer);

      publish(message);
   }

}
