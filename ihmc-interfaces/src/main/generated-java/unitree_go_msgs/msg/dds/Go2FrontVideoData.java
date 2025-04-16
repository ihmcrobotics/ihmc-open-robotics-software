package unitree_go_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class Go2FrontVideoData extends Packet<Go2FrontVideoData> implements Settable<Go2FrontVideoData>, EpsilonComparable<Go2FrontVideoData>
{
   public long time_frame_;
   public us.ihmc.idl.IDLSequence.Byte  video720p_;
   public us.ihmc.idl.IDLSequence.Byte  video360p_;
   public us.ihmc.idl.IDLSequence.Byte  video180p_;

   public Go2FrontVideoData()
   {
      video720p_ = new us.ihmc.idl.IDLSequence.Byte (100, "type_9");

      video360p_ = new us.ihmc.idl.IDLSequence.Byte (100, "type_9");

      video180p_ = new us.ihmc.idl.IDLSequence.Byte (100, "type_9");

   }

   public Go2FrontVideoData(Go2FrontVideoData other)
   {
      this();
      set(other);
   }

   public void set(Go2FrontVideoData other)
   {
      time_frame_ = other.time_frame_;

      video720p_.set(other.video720p_);
      video360p_.set(other.video360p_);
      video180p_.set(other.video180p_);
   }

   public void setTimeFrame(long time_frame)
   {
      time_frame_ = time_frame;
   }
   public long getTimeFrame()
   {
      return time_frame_;
   }


   public us.ihmc.idl.IDLSequence.Byte  getVideo720p()
   {
      return video720p_;
   }


   public us.ihmc.idl.IDLSequence.Byte  getVideo360p()
   {
      return video360p_;
   }


   public us.ihmc.idl.IDLSequence.Byte  getVideo180p()
   {
      return video180p_;
   }


   public static Supplier<Go2FrontVideoDataPubSubType> getPubSubType()
   {
      return Go2FrontVideoDataPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return Go2FrontVideoDataPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(Go2FrontVideoData other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.time_frame_, other.time_frame_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.video720p_, other.video720p_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.video360p_, other.video360p_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.video180p_, other.video180p_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof Go2FrontVideoData)) return false;

      Go2FrontVideoData otherMyClass = (Go2FrontVideoData) other;

      if(this.time_frame_ != otherMyClass.time_frame_) return false;

      if (!this.video720p_.equals(otherMyClass.video720p_)) return false;
      if (!this.video360p_.equals(otherMyClass.video360p_)) return false;
      if (!this.video180p_.equals(otherMyClass.video180p_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("Go2FrontVideoData {");
      builder.append("time_frame=");
      builder.append(this.time_frame_);      builder.append(", ");
      builder.append("video720p=");
      builder.append(this.video720p_);      builder.append(", ");
      builder.append("video360p=");
      builder.append(this.video360p_);      builder.append(", ");
      builder.append("video180p=");
      builder.append(this.video180p_);
      builder.append("}");
      return builder.toString();
   }
}
