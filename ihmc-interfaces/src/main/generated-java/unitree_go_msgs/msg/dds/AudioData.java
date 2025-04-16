package unitree_go_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class AudioData extends Packet<AudioData> implements Settable<AudioData>, EpsilonComparable<AudioData>
{
   public long time_frame_;
   public us.ihmc.idl.IDLSequence.Byte  data_;

   public AudioData()
   {
      data_ = new us.ihmc.idl.IDLSequence.Byte (100, "type_9");

   }

   public AudioData(AudioData other)
   {
      this();
      set(other);
   }

   public void set(AudioData other)
   {
      time_frame_ = other.time_frame_;

      data_.set(other.data_);
   }

   public void setTimeFrame(long time_frame)
   {
      time_frame_ = time_frame;
   }
   public long getTimeFrame()
   {
      return time_frame_;
   }


   public us.ihmc.idl.IDLSequence.Byte  getData()
   {
      return data_;
   }


   public static Supplier<AudioDataPubSubType> getPubSubType()
   {
      return AudioDataPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return AudioDataPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(AudioData other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.time_frame_, other.time_frame_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.data_, other.data_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof AudioData)) return false;

      AudioData otherMyClass = (AudioData) other;

      if(this.time_frame_ != otherMyClass.time_frame_) return false;

      if (!this.data_.equals(otherMyClass.data_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AudioData {");
      builder.append("time_frame=");
      builder.append(this.time_frame_);      builder.append(", ");
      builder.append("data=");
      builder.append(this.data_);
      builder.append("}");
      return builder.toString();
   }
}
