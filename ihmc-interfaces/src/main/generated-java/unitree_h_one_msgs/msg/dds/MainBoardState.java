package unitree_h_one_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class MainBoardState extends Packet<MainBoardState> implements Settable<MainBoardState>, EpsilonComparable<MainBoardState>
{
   public short[] temperature_;
   public float[] value_;

   public MainBoardState()
   {
      temperature_ = new short[6];

      value_ = new float[6];

   }

   public MainBoardState(MainBoardState other)
   {
      this();
      set(other);
   }

   public void set(MainBoardState other)
   {
      for(int i1 = 0; i1 < temperature_.length; ++i1)
      {
            temperature_[i1] = other.temperature_[i1];

      }

      for(int i3 = 0; i3 < value_.length; ++i3)
      {
            value_[i3] = other.value_[i3];

      }

   }


   public short[] getTemperature()
   {
      return temperature_;
   }


   public float[] getValue()
   {
      return value_;
   }


   public static Supplier<MainBoardStatePubSubType> getPubSubType()
   {
      return MainBoardStatePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return MainBoardStatePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(MainBoardState other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      for(int i5 = 0; i5 < temperature_.length; ++i5)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.temperature_[i5], other.temperature_[i5], epsilon)) return false;
      }

      for(int i7 = 0; i7 < value_.length; ++i7)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.value_[i7], other.value_[i7], epsilon)) return false;
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof MainBoardState)) return false;

      MainBoardState otherMyClass = (MainBoardState) other;

      for(int i9 = 0; i9 < temperature_.length; ++i9)
      {
                if(this.temperature_[i9] != otherMyClass.temperature_[i9]) return false;

      }
      for(int i11 = 0; i11 < value_.length; ++i11)
      {
                if(this.value_[i11] != otherMyClass.value_[i11]) return false;

      }

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("MainBoardState {");
      builder.append("temperature=");
      builder.append(java.util.Arrays.toString(this.temperature_));      builder.append(", ");
      builder.append("value=");
      builder.append(java.util.Arrays.toString(this.value_));
      builder.append("}");
      return builder.toString();
   }
}
