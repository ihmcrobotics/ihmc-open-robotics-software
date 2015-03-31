package us.ihmc.utilities.ros.publisher;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Vector3d;

import org.ros.message.Time;

import std_msgs.Header;
import trooper_mlc_msgs.RawIMUData;

public class RosCachedRawIMUDataPublisher extends RosTopicPublisher<trooper_mlc_msgs.CachedRawIMUData>
{
   int counter=0;
   private ArrayList<trooper_mlc_msgs.RawIMUData> availableImuData = new ArrayList<RawIMUData>();
   
   
   public RosCachedRawIMUDataPublisher(boolean latched)
   {
      super(trooper_mlc_msgs.CachedRawIMUData._TYPE,latched);
   }
   
   public void publish(long timestamp, List<trooper_mlc_msgs.RawIMUData> imuData)
   {
      trooper_mlc_msgs.CachedRawIMUData message = getMessage();

      Header header = newMessageFromType(Header._TYPE);
      header.setFrameId("pelvis_imu");
      header.setStamp(Time.fromNano(timestamp));
      header.setSeq(counter);
      counter++;
            
      message.setHeader(header);
      message.setData(imuData);
      
      publish(message);
   }
   
   /**
    * Not Thread Safe
    * @param timestamp
    * @param rawImuAngularVelocity
    * @param rawImuLinearAcceleration
    */
   public void appendRawImuData(long timestamp, Vector3d rawImuAngularVelocity, Vector3d rawImuLinearAcceleration)
   {
      RawIMUData rawImuData = newMessageFromType(trooper_mlc_msgs.RawIMUData._TYPE);
      
      rawImuData.setImuTimestamp(timestamp);
      //delta angle (radians) in the frame of the IMU
      rawImuData.setDax(rawImuAngularVelocity.getX());
      rawImuData.setDay(rawImuAngularVelocity.getY());
      rawImuData.setDaz(rawImuAngularVelocity.getZ());
      //linear acceleration (m/s^2) in the frame of the IM
      rawImuData.setDdx(rawImuLinearAcceleration.getX());
      rawImuData.setDdy(rawImuLinearAcceleration.getY());
      rawImuData.setDdz(rawImuLinearAcceleration.getZ());
      
      availableImuData.add(rawImuData);
      if(availableImuData.size() == 15)
      {
         ArrayList<trooper_mlc_msgs.RawIMUData> imuBatch = new ArrayList<RawIMUData>();
         
         for(int i = 0; i < 15; i++)
         {
            imuBatch.add(availableImuData.remove(0));
         }
         publish(timestamp, imuBatch);
      }
   }
}