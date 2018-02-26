package us.ihmc.humanoidRobotics.communication.packets.sensing;

import java.util.Arrays;

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

   @Override
   public void set(VideoPacket other)
   {
      videoSource = other.videoSource;
      timeStamp = other.timeStamp;
      data = Arrays.copyOf(other.data, other.data.length);
      position = new Point3D(other.position);
      orientation = new Quaternion(other.orientation);
      intrinsicParameters = other.intrinsicParameters;
      setPacketInformation(other);
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
      return "VideoPacket [source=" + videoSource + ", timeStamp=" + timeStamp + ", data=" + data.length + " byte, position=" + position + ", orientation="
            + orientation;
   }
}
