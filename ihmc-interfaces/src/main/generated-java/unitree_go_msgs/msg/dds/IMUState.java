package unitree_go_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class IMUState extends Packet<IMUState> implements Settable<IMUState>, EpsilonComparable<IMUState>
{
   public float[] quaternion_;
   public float[] gyroscope_;
   public float[] accelerometer_;
   public float[] rpy_;
   public byte temperature_;

   public IMUState()
   {
      quaternion_ = new float[4];

      gyroscope_ = new float[3];

      accelerometer_ = new float[3];

      rpy_ = new float[3];

   }

   public IMUState(IMUState other)
   {
      this();
      set(other);
   }

   public void set(IMUState other)
   {
      for(int i1 = 0; i1 < quaternion_.length; ++i1)
      {
            quaternion_[i1] = other.quaternion_[i1];

      }

      for(int i3 = 0; i3 < gyroscope_.length; ++i3)
      {
            gyroscope_[i3] = other.gyroscope_[i3];

      }

      for(int i5 = 0; i5 < accelerometer_.length; ++i5)
      {
            accelerometer_[i5] = other.accelerometer_[i5];

      }

      for(int i7 = 0; i7 < rpy_.length; ++i7)
      {
            rpy_[i7] = other.rpy_[i7];

      }

      temperature_ = other.temperature_;

   }


   public float[] getQuaternion()
   {
      return quaternion_;
   }


   public float[] getGyroscope()
   {
      return gyroscope_;
   }


   public float[] getAccelerometer()
   {
      return accelerometer_;
   }


   public float[] getRpy()
   {
      return rpy_;
   }

   public void setTemperature(byte temperature)
   {
      temperature_ = temperature;
   }
   public byte getTemperature()
   {
      return temperature_;
   }


   public static Supplier<IMUStatePubSubType> getPubSubType()
   {
      return IMUStatePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return IMUStatePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(IMUState other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      for(int i9 = 0; i9 < quaternion_.length; ++i9)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.quaternion_[i9], other.quaternion_[i9], epsilon)) return false;
      }

      for(int i11 = 0; i11 < gyroscope_.length; ++i11)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.gyroscope_[i11], other.gyroscope_[i11], epsilon)) return false;
      }

      for(int i13 = 0; i13 < accelerometer_.length; ++i13)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.accelerometer_[i13], other.accelerometer_[i13], epsilon)) return false;
      }

      for(int i15 = 0; i15 < rpy_.length; ++i15)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.rpy_[i15], other.rpy_[i15], epsilon)) return false;
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.temperature_, other.temperature_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof IMUState)) return false;

      IMUState otherMyClass = (IMUState) other;

      for(int i17 = 0; i17 < quaternion_.length; ++i17)
      {
                if(this.quaternion_[i17] != otherMyClass.quaternion_[i17]) return false;

      }
      for(int i19 = 0; i19 < gyroscope_.length; ++i19)
      {
                if(this.gyroscope_[i19] != otherMyClass.gyroscope_[i19]) return false;

      }
      for(int i21 = 0; i21 < accelerometer_.length; ++i21)
      {
                if(this.accelerometer_[i21] != otherMyClass.accelerometer_[i21]) return false;

      }
      for(int i23 = 0; i23 < rpy_.length; ++i23)
      {
                if(this.rpy_[i23] != otherMyClass.rpy_[i23]) return false;

      }
      if(this.temperature_ != otherMyClass.temperature_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("IMUState {");
      builder.append("quaternion=");
      builder.append(java.util.Arrays.toString(this.quaternion_));      builder.append(", ");
      builder.append("gyroscope=");
      builder.append(java.util.Arrays.toString(this.gyroscope_));      builder.append(", ");
      builder.append("accelerometer=");
      builder.append(java.util.Arrays.toString(this.accelerometer_));      builder.append(", ");
      builder.append("rpy=");
      builder.append(java.util.Arrays.toString(this.rpy_));      builder.append(", ");
      builder.append("temperature=");
      builder.append(this.temperature_);
      builder.append("}");
      return builder.toString();
   }
}
