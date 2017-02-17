package us.ihmc.humanoidRobotics.communication.packets.sensing;

import java.util.Random;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.communication.packets.HighBandwidthPacket;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.producers.VideoSource;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.TransformableDataObject;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.geometry.TransformTools;
import us.ihmc.robotics.random.RandomTools;

@HighBandwidthPacket
public class VideoPacket extends Packet<VideoPacket> implements TransformableDataObject<VideoPacket>
{
   public VideoSource videoSource;
   public long timeStamp;
   public byte[] data;
   public Point3D position;
   public Quaternion orientation;
   public IntrinsicParameters intrinsicParameters;
   
   public VideoPacket()
   {
      
   }

   public VideoPacket(VideoSource videoSource, long timeStamp, byte[] data, Point3D position, Quaternion orientation, IntrinsicParameters intrinsicParameters)
   {
      this(videoSource, timeStamp, data, position, orientation, intrinsicParameters, null);
   }

   public VideoPacket(VideoSource videoSource, long timeStamp, byte[] data, Point3D position, Quaternion orientation, IntrinsicParameters intrinsicParameters,
         PacketDestination packetDestination)
   {
      if(packetDestination != null)
         setDestination(packetDestination);
      this.videoSource = videoSource;
      this.timeStamp = timeStamp;
      this.data = data;
      this.position = position;
      this.orientation = orientation;
      this.intrinsicParameters = intrinsicParameters;
   }

   public VideoSource getVideoSource()
   {
      return videoSource;
   }

   public long getTimeStamp()
   {
      return timeStamp;
   }

   public byte[] getData()
   {
      return data;
   }

   public IntrinsicParameters getIntrinsicParameters()
   {
      return intrinsicParameters;
   }

   public Point3D getPosition()
   {
      return position;
   }

   public Quaternion getOrientation()
   {
      return orientation;
   }

   @Override
   public boolean epsilonEquals(VideoPacket other, double epsilon)
   {
      if (!getPosition().epsilonEquals(other.getPosition(), epsilon))
      {
         System.out.println(getPosition());
         System.out.println(other.getPosition());

         return false;
      }

      if (!RotationTools.quaternionEpsilonEquals(getOrientation(), other.getOrientation(), epsilon))
      {
         System.out.println(getOrientation());
         System.out.println(other.getOrientation());

         return false;
      }


      if (this.getData().length != other.getData().length)
      {
         System.out.println("Data length");
         System.out.println(getData().length);
         System.out.println(other.getData().length);

         return false;
      }

      for (int i = 0; i < getData().length; i++)
      {
         if (getData()[i] != other.getData()[i])
         {
            return false;
         }
      }

      return true;
   }

   @Override
   public VideoPacket transform(RigidBodyTransform transform)
   {
      Point3D newPoint = TransformTools.getTransformedPoint(getPosition(), transform);
      Quaternion newOrientation = TransformTools.getTransformedQuat(getOrientation(), transform);

      return new VideoPacket(getVideoSource(), getTimeStamp(), getData(), newPoint, newOrientation, getIntrinsicParameters());
   }

   @Override
   public String toString()
   {
      return "VideoPacket [source=" + videoSource + ", timeStamp=" + timeStamp + ", data=" + data.length + " byte, position=" + position + ", orientation=" + orientation;
   }

   public VideoPacket(Random random)
   {
      int[] POSITION_BITS = { 16, 16, 16 };
      int QUATERNION_BITS = 14;
      int SOURCE_ID_BITS = 8;

      int positionOffset = 32;
      int rotationOffset = positionOffset + POSITION_BITS[0] + POSITION_BITS[1] + POSITION_BITS[2];
      int metaDataLength = rotationOffset + 4 * QUATERNION_BITS + SOURCE_ID_BITS;

      int length = random.nextInt((int) (Math.pow(2, 20) - 1 - metaDataLength / 8));

      //    int length = (int) (Math.pow(2, 20) - 1 - metaDataLength/8);
      byte[] data = new byte[length];
      random.nextBytes(data);

      Point3D position = RandomTools.generateRandomPoint(random, 2.0, 2.0, 1.0);
      Quaternion orientation = RandomTools.generateRandomQuaternion(random);

      this.timeStamp = 0;
      this.videoSource = VideoSource.MULTISENSE_LEFT_EYE;
      this.data = data;
      this.position = position;
      this.orientation = orientation;
   }
}
