package us.ihmc.humanoidRobotics.communication.packets.sensing;

import java.util.Arrays;
import java.util.Random;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.communication.packets.HighBandwidthPacket;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.producers.VideoSource;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.random.RandomGeometry;

@HighBandwidthPacket
public class VideoPacket extends Packet<VideoPacket>
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

   public VideoPacket(VideoPacket other)
   {
      videoSource = other.videoSource;
      timeStamp = other.timeStamp;
      data = Arrays.copyOf(other.data, other.data.length);
      position = new Point3D(other.position);
      orientation = new Quaternion(other.orientation);
      intrinsicParameters = new IntrinsicParameters(other.intrinsicParameters);
   }

   public VideoPacket(VideoSource videoSource, long timeStamp, byte[] data, Point3DReadOnly position, QuaternionReadOnly orientation, IntrinsicParameters intrinsicParameters)
   {
      this(videoSource, timeStamp, data, position, orientation, intrinsicParameters, null);
   }

   public VideoPacket(VideoSource videoSource, long timeStamp, byte[] data, Point3DReadOnly position, QuaternionReadOnly orientation, IntrinsicParameters intrinsicParameters,
         PacketDestination packetDestination)
   {
      if(packetDestination != null)
         setDestination(packetDestination);
      this.videoSource = videoSource;
      this.timeStamp = timeStamp;
      this.data = data;
      this.position = new Point3D(position);
      this.orientation = new Quaternion(orientation);
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

   public Point3DReadOnly getPosition()
   {
      return position;
   }

   public QuaternionReadOnly getOrientation()
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
   public String toString()
   {
      return "VideoPacket [source=" + videoSource + ", timeStamp=" + timeStamp + ", data=" + data.length + " byte, position=" + position + ", orientation=" + orientation;
   }

   public VideoPacket(Random random)
   {
      byte[] data = new byte[random.nextInt((int) (Math.pow(2, 20) - 19))];
      random.nextBytes(data);

      this.timeStamp = 0;
      this.videoSource = VideoSource.MULTISENSE_LEFT_EYE;
      this.data = data;
      this.position = RandomGeometry.nextPoint3D(random, 2.0, 2.0, 1.0);
      this.orientation = RandomGeometry.nextQuaternion(random);
      this.intrinsicParameters = new IntrinsicParameters();
   }
}
