package unitree_h_one_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class PressSensorState extends Packet<PressSensorState> implements Settable<PressSensorState>, EpsilonComparable<PressSensorState>
{
   public float[] pressure_;
   public float[] temperature_;

   public PressSensorState()
   {
      pressure_ = new float[12];

      temperature_ = new float[12];

   }

   public PressSensorState(PressSensorState other)
   {
      this();
      set(other);
   }

   public void set(PressSensorState other)
   {
      for(int i1 = 0; i1 < pressure_.length; ++i1)
      {
            pressure_[i1] = other.pressure_[i1];

      }

      for(int i3 = 0; i3 < temperature_.length; ++i3)
      {
            temperature_[i3] = other.temperature_[i3];

      }

   }


   public float[] getPressure()
   {
      return pressure_;
   }


   public float[] getTemperature()
   {
      return temperature_;
   }


   public static Supplier<PressSensorStatePubSubType> getPubSubType()
   {
      return PressSensorStatePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return PressSensorStatePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(PressSensorState other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      for(int i5 = 0; i5 < pressure_.length; ++i5)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.pressure_[i5], other.pressure_[i5], epsilon)) return false;
      }

      for(int i7 = 0; i7 < temperature_.length; ++i7)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.temperature_[i7], other.temperature_[i7], epsilon)) return false;
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof PressSensorState)) return false;

      PressSensorState otherMyClass = (PressSensorState) other;

      for(int i9 = 0; i9 < pressure_.length; ++i9)
      {
                if(this.pressure_[i9] != otherMyClass.pressure_[i9]) return false;

      }
      for(int i11 = 0; i11 < temperature_.length; ++i11)
      {
                if(this.temperature_[i11] != otherMyClass.temperature_[i11]) return false;

      }

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("PressSensorState {");
      builder.append("pressure=");
      builder.append(java.util.Arrays.toString(this.pressure_));      builder.append(", ");
      builder.append("temperature=");
      builder.append(java.util.Arrays.toString(this.temperature_));
      builder.append("}");
      return builder.toString();
   }
}
