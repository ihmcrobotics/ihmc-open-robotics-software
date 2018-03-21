package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

public class VideoPacket extends Packet<VideoPacket> implements Settable<VideoPacket>, EpsilonComparable<VideoPacket>
{
   public static final byte VIDEO_SOURCE_MULTISENSE_LEFT_EYE = (byte) 0;
   public static final byte VIDEO_SOURCE_MULTISENSE_RIGHT_EYE = (byte) 1;
   public static final byte VIDEO_SOURCE_FISHEYE_LEFT = (byte) 2;
   public static final byte VIDEO_SOURCE_FISHEYE_RIGHT = (byte) 3;
   public static final byte VIDEO_SOURCE_CV_THRESHOLD = (byte) 4;
   public static final byte VIDEO_SOURCE_IMAGE_PROCESSING_BEHAVIOR = (byte) 5;
   public static final byte VIDEO_SOURCE_AWARE_FACE_TRACKER = (byte) 6;
   public byte video_source_ = (byte) 255;
   public long timestamp_;
   public us.ihmc.idl.IDLSequence.Byte data_;
   public us.ihmc.euclid.tuple3D.Point3D position_;
   public us.ihmc.euclid.tuple4D.Quaternion orientation_;
   public controller_msgs.msg.dds.IntrinsicParametersMessage intrinsic_parameters_;

   public VideoPacket()
   {

      data_ = new us.ihmc.idl.IDLSequence.Byte(100, "type_9");

      position_ = new us.ihmc.euclid.tuple3D.Point3D();
      orientation_ = new us.ihmc.euclid.tuple4D.Quaternion();
      intrinsic_parameters_ = new controller_msgs.msg.dds.IntrinsicParametersMessage();
   }

   public VideoPacket(VideoPacket other)
   {
      set(other);
   }

   public void set(VideoPacket other)
   {
      video_source_ = other.video_source_;

      timestamp_ = other.timestamp_;

      data_.set(other.data_);
      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.position_, position_);
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.orientation_, orientation_);
      controller_msgs.msg.dds.IntrinsicParametersMessagePubSubType.staticCopy(other.intrinsic_parameters_, intrinsic_parameters_);
   }

   public byte getVideoSource()
   {
      return video_source_;
   }

   public void setVideoSource(byte video_source)
   {
      video_source_ = video_source;
   }

   public long getTimestamp()
   {
      return timestamp_;
   }

   public void setTimestamp(long timestamp)
   {
      timestamp_ = timestamp;
   }

   public us.ihmc.idl.IDLSequence.Byte getData()
   {
      return data_;
   }

   public us.ihmc.euclid.tuple3D.Point3D getPosition()
   {
      return position_;
   }

   public us.ihmc.euclid.tuple4D.Quaternion getOrientation()
   {
      return orientation_;
   }

   public controller_msgs.msg.dds.IntrinsicParametersMessage getIntrinsicParameters()
   {
      return intrinsic_parameters_;
   }

   @Override
   public boolean epsilonEquals(VideoPacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.video_source_, other.video_source_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.timestamp_, other.timestamp_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.data_, other.data_, epsilon))
         return false;

      if (!this.position_.epsilonEquals(other.position_, epsilon))
         return false;

      if (!this.orientation_.epsilonEquals(other.orientation_, epsilon))
         return false;

      if (!this.intrinsic_parameters_.epsilonEquals(other.intrinsic_parameters_, epsilon))
         return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;
      if (!(other instanceof VideoPacket))
         return false;

      VideoPacket otherMyClass = (VideoPacket) other;

      if (this.video_source_ != otherMyClass.video_source_)
         return false;

      if (this.timestamp_ != otherMyClass.timestamp_)
         return false;

      if (!this.data_.equals(otherMyClass.data_))
         return false;

      if (!this.position_.equals(otherMyClass.position_))
         return false;

      if (!this.orientation_.equals(otherMyClass.orientation_))
         return false;

      if (!this.intrinsic_parameters_.equals(otherMyClass.intrinsic_parameters_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("VideoPacket {");
      builder.append("video_source=");
      builder.append(this.video_source_);

      builder.append(", ");
      builder.append("timestamp=");
      builder.append(this.timestamp_);

      builder.append(", ");
      builder.append("data=");
      builder.append(this.data_);

      builder.append(", ");
      builder.append("position=");
      builder.append(this.position_);

      builder.append(", ");
      builder.append("orientation=");
      builder.append(this.orientation_);

      builder.append(", ");
      builder.append("intrinsic_parameters=");
      builder.append(this.intrinsic_parameters_);

      builder.append("}");
      return builder.toString();
   }
}