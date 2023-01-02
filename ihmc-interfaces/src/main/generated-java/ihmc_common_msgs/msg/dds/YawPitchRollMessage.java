package ihmc_common_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * A yaw-pitch-roll is used to represent a 3D orientation by three successive rotations: rotation around the z-axis
       * (yaw), then around the y-axis (pitch), and then around the x-axis (roll). The three components yaw, pitch, and
       * roll represents the angle for rotation expressed in radians.
       */
public class YawPitchRollMessage extends Packet<YawPitchRollMessage> implements Settable<YawPitchRollMessage>, EpsilonComparable<YawPitchRollMessage>
{
   /**
            * The yaw angle representing the first rotation around the z-axis.
            */
   public double yaw_;
   /**
            * The pitch angle representing the second rotation around the y-axis.
            */
   public double pitch_;
   /**
            * The roll angle representing the third rotation around the x-axis.
            */
   public double roll_;

   public YawPitchRollMessage()
   {
   }

   public YawPitchRollMessage(YawPitchRollMessage other)
   {
      this();
      set(other);
   }

   public void set(YawPitchRollMessage other)
   {
      yaw_ = other.yaw_;

      pitch_ = other.pitch_;

      roll_ = other.roll_;

   }

   /**
            * The yaw angle representing the first rotation around the z-axis.
            */
   public void setYaw(double yaw)
   {
      yaw_ = yaw;
   }
   /**
            * The yaw angle representing the first rotation around the z-axis.
            */
   public double getYaw()
   {
      return yaw_;
   }

   /**
            * The pitch angle representing the second rotation around the y-axis.
            */
   public void setPitch(double pitch)
   {
      pitch_ = pitch;
   }
   /**
            * The pitch angle representing the second rotation around the y-axis.
            */
   public double getPitch()
   {
      return pitch_;
   }

   /**
            * The roll angle representing the third rotation around the x-axis.
            */
   public void setRoll(double roll)
   {
      roll_ = roll;
   }
   /**
            * The roll angle representing the third rotation around the x-axis.
            */
   public double getRoll()
   {
      return roll_;
   }


   public static Supplier<YawPitchRollMessagePubSubType> getPubSubType()
   {
      return YawPitchRollMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return YawPitchRollMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(YawPitchRollMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.yaw_, other.yaw_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.pitch_, other.pitch_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.roll_, other.roll_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof YawPitchRollMessage)) return false;

      YawPitchRollMessage otherMyClass = (YawPitchRollMessage) other;

      if(this.yaw_ != otherMyClass.yaw_) return false;

      if(this.pitch_ != otherMyClass.pitch_) return false;

      if(this.roll_ != otherMyClass.roll_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("YawPitchRollMessage {");
      builder.append("yaw=");
      builder.append(this.yaw_);      builder.append(", ");
      builder.append("pitch=");
      builder.append(this.pitch_);      builder.append(", ");
      builder.append("roll=");
      builder.append(this.roll_);
      builder.append("}");
      return builder.toString();
   }
}
