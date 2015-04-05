package us.ihmc.utilities.ros.publisher;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.ros.message.Time;

import std_msgs.Header;
import trooper_mlc_msgs.RawIMUData;
import us.ihmc.utilities.math.geometry.RotationFunctions;

public class RosCachedRawIMUDataPublisher extends RosTopicPublisher<trooper_mlc_msgs.CachedRawIMUData>
{
   int rosPacketSendCounter=0;
   int rawImuCounter = 0;
   double[] currentImuOrientation = new double[3];
   double[] previousImuOrientation = new double[3];
   
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
      header.setSeq(rosPacketSendCounter);
      rosPacketSendCounter++;
            
      message.setHeader(header);
      message.setData(imuData);
      
      publish(message);
   }
   
   /**
    * @param timestamp
    * @param rawImuAngularVelocity
    * @param rawImuLinearAcceleration
    */
   public synchronized void appendRawImuData(long timestamp, Quat4d rawImuOrientation, Vector3d rawImuLinearAcceleration)
   {
      RawIMUData rawImuData = newMessageFromType(trooper_mlc_msgs.RawIMUData._TYPE);
      RotationFunctions.setYawPitchRollBasedOnQuaternion(currentImuOrientation, rawImuOrientation);
      
      rawImuData.setImuTimestamp(timestamp);
      //delta angle (radians) in the frame of the IMU
      rawImuData.setDax(currentImuOrientation[1] - previousImuOrientation[1]);
      rawImuData.setDay(currentImuOrientation[0] - previousImuOrientation[0]);
      rawImuData.setDaz(currentImuOrientation[2] - previousImuOrientation[2]);
      //linear acceleration (m/s^2) in the frame of the IM
      rawImuData.setDdx(rawImuLinearAcceleration.getX());
      rawImuData.setDdy(rawImuLinearAcceleration.getY());
      rawImuData.setDdz(rawImuLinearAcceleration.getZ());
      rawImuData.setPacketCount(rawImuCounter);
      rawImuCounter++;
      
      availableImuData.add(rawImuData);
      if(availableImuData.size() == 15)
      {
         ArrayList<trooper_mlc_msgs.RawIMUData> imuBatch = new ArrayList<RawIMUData>();
         for(int i = 0; i < 15; i++)
         {
            imuBatch.add(availableImuData.get(14 - i));
         }
         publish(timestamp, imuBatch);
         availableImuData.remove(0);
      }
   }
}