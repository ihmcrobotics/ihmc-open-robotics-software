package us.ihmc.utilities.ros.publisher;

import org.ros.message.Time;

import controller_msgs.msg.dds.IMUPacket;
import geometry_msgs.Quaternion;
import geometry_msgs.Vector3;
import std_msgs.Header;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public class RosImuPublisher extends RosTopicPublisher<sensor_msgs.Imu>
{
   int counter=0;
   public RosImuPublisher(boolean latched)
   {
      super(sensor_msgs.Imu._TYPE,latched);
   }
   
   public void publish(long timestamp, Vector3DReadOnly linearAcceleration, QuaternionReadOnly orientation, Vector3DReadOnly angularVelocity, String frameId)
   {
      sensor_msgs.Imu message = getMessage();

      Vector3  localLinearAcceleration = newMessageFromType(Vector3._TYPE);
      localLinearAcceleration.setX(linearAcceleration.getX());
      localLinearAcceleration.setY(linearAcceleration.getY());
      localLinearAcceleration.setZ(linearAcceleration.getZ());
      
      Quaternion localOrientation = newMessageFromType(Quaternion._TYPE);
      localOrientation.setW(orientation.getS());
      localOrientation.setX(orientation.getX());
      localOrientation.setY(orientation.getY());
      localOrientation.setZ(orientation.getZ());
      
      Vector3 localAngularVelocity = newMessageFromType(Vector3._TYPE);
      localAngularVelocity.setX(angularVelocity.getX());
      localAngularVelocity.setY(angularVelocity.getY());
      localAngularVelocity.setZ(angularVelocity.getZ());
      
      Header header = newMessageFromType(Header._TYPE);
      header.setFrameId(frameId);
      header.setStamp(Time.fromNano(timestamp));
      header.setSeq(counter);
      counter++;
            
      message.setHeader(header);
      message.setLinearAcceleration(localLinearAcceleration);
      message.setOrientation(localOrientation);
      message.setAngularVelocity(localAngularVelocity);
      
      publish(message);
   }

   public void publish(long timeStamp, IMUPacket imuPacketForSensor, String frameId)
   {
      publish(timeStamp, imuPacketForSensor.getLinearAcceleration(), imuPacketForSensor.getOrientation(),
            imuPacketForSensor.getAngularVelocity(), frameId);
   }
}