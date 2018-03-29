package us.ihmc.humanoidRobotics.communication.packets.sensing;

import gnu.trove.list.array.TByteArrayList;
import us.ihmc.communication.packets.HighBandwidthPacket;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.IntrinsicParametersMessage;
import us.ihmc.robotics.geometry.RotationTools;

@HighBandwidthPacket
public class VideoPacket extends Packet<VideoPacket>
{
   public static final byte VIDEO_SOURCE_MULTISENSE_LEFT_EYE = 0;
   public static final byte VIDEO_SOURCE_MULTISENSE_RIGHT_EYE = 1;
   public static final byte VIDEO_SOURCE_FISHEYE_LEFT = 2;
   public static final byte VIDEO_SOURCE_FISHEYE_RIGHT = 3;
   public static final byte VIDEO_SOURCE_CV_THRESHOLD = 4;
   public static final byte VIDEO_SOURCE_IMAGE_PROCESSING_BEHAVIOR = 5;
   public static final byte VIDEO_SOURCE_AWARE_FACE_TRACKER = 6;

   public byte videoSource;
   public long timeStamp;
   public TByteArrayList data = new TByteArrayList();
   public Point3D position = new Point3D();
   public Quaternion orientation = new Quaternion();
   public IntrinsicParametersMessage intrinsicParameters = new IntrinsicParametersMessage();

   public VideoPacket()
   {

   }

   public VideoPacket(VideoPacket other)
   {
      set(other);
   }

   @Override
   public void set(VideoPacket other)
   {
      videoSource = other.videoSource;
      timeStamp = other.timeStamp;
      MessageTools.copyData(other.data, data);
      position.set(other.position);
      orientation.set(other.orientation);
      intrinsicParameters.set(other.intrinsicParameters);
      setPacketInformation(other);
   }

   public byte getVideoSource()
   {
      return videoSource;
   }

   public long getTimeStamp()
   {
      return timeStamp;
   }

   public TByteArrayList getData()
   {
      return data;
   }

   public IntrinsicParametersMessage getIntrinsicParameters()
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

      if (!data.equals(other.data))
         return false;

      return true;
   }

   @Override
   public String toString()
   {
      return "VideoPacket [source=" + videoSource + ", timeStamp=" + timeStamp + ", data=" + data.size() + " byte, position=" + position + ", orientation="
            + orientation;
   }
}
