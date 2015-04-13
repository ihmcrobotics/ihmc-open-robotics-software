package us.ihmc.utilities.ros.publisher;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.ros.message.Time;

import std_msgs.Header;
import trooper_mlc_msgs.RawIMUData;
import us.ihmc.communication.packets.dataobjects.IMUPacket;
import us.ihmc.utilities.math.TimeTools;
import us.ihmc.utilities.math.geometry.RotationFunctions;

public class RosCachedRawIMUDataPublisher extends RosTopicPublisher<trooper_mlc_msgs.CachedRawIMUData>
{
   int rosPacketSendCounter=0;
   long rawImuCounter = 0;
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
    * @param timestampInNanoSeconds
    * @param rawImuAngularVelocity
    * @param linearAccele
    */
   public synchronized void appendRawImuData(long timestampInNanoSeconds, Quat4d orientation, Vector3d linearAcceleration)
   {
      RawIMUData rawImuData = newMessageFromType(trooper_mlc_msgs.RawIMUData._TYPE);
      RotationFunctions.setYawPitchRollBasedOnQuaternion(currentImuOrientation, orientation);
      rawImuData.setImuTimestamp(TimeTools.nanoSecondsToMicroseconds(timestampInNanoSeconds));
      //delta angle (radians) in the frame of the IMU
      rawImuData.setDax(currentImuOrientation[2] - previousImuOrientation[2]);
      rawImuData.setDay(currentImuOrientation[1] - previousImuOrientation[1]);
      rawImuData.setDaz(currentImuOrientation[0] - previousImuOrientation[0]);
      //linear acceleration (m/s^2) in the frame of the IM
      rawImuData.setDdx(linearAcceleration.getX());
      rawImuData.setDdy(linearAcceleration.getY());
      rawImuData.setDdz(linearAcceleration.getZ());
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
         publish(timestampInNanoSeconds, imuBatch);
         availableImuData.remove(0);
      }
      
      for (int i = 0; i < currentImuOrientation.length; i++)
      {
         previousImuOrientation[i] = currentImuOrientation[i];
      }
   }
   
   public void appendRawImuData(long timeStamp, IMUPacket imuPacket)
   {
      appendRawImuData(timeStamp, imuPacket.getOrientation(), imuPacket.getLinearAcceleration());
   } 
   
   public ArrayList<trooper_mlc_msgs.RawIMUData> createRandomRawImuData(Random random, int size)
   {
      long timestampInNanoSeconds = random.nextLong();
      ArrayList<trooper_mlc_msgs.RawIMUData> imuBatch = new ArrayList<RawIMUData>();
      int randomImuCounter = random.nextInt();
      for(int i = 0; i < size; i++)
      {
         RawIMUData rawImuData = newMessageFromType(trooper_mlc_msgs.RawIMUData._TYPE);
         rawImuData.setImuTimestamp(TimeTools.nanoSecondsToMicroseconds(timestampInNanoSeconds));
         rawImuData.setPacketCount(randomImuCounter);
         rawImuData.setDax(random.nextInt());
         rawImuData.setDay(random.nextInt());
         rawImuData.setDaz(random.nextInt());
         //linear acceleration (m/s^2) in the frame of the IM
         rawImuData.setDdx(random.nextInt());
         rawImuData.setDdy(random.nextInt());
         rawImuData.setDdz(random.nextInt());
         randomImuCounter++;
         imuBatch.add(rawImuData);
      }
      return imuBatch;
   }
}