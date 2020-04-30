package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Atlas specific message
       */
public class BlackFlyParameterPacket extends Packet<BlackFlyParameterPacket> implements Settable<BlackFlyParameterPacket>, EpsilonComparable<BlackFlyParameterPacket>
{

   public static final byte ROBOT_SIDE_LEFT = (byte) 0;

   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public boolean auto_exposure_;

   public boolean auto_gain_;

   public boolean auto_shutter_;

   public double exposure_;

   public double frame_rate_;

   public boolean from_ui_;

   public double gain_;

   public double shutter_;

   public byte robot_side_ = (byte) 255;

   public BlackFlyParameterPacket()
   {











   }

   public BlackFlyParameterPacket(BlackFlyParameterPacket other)
   {
      this();
      set(other);
   }

   public void set(BlackFlyParameterPacket other)
   {

      sequence_id_ = other.sequence_id_;


      auto_exposure_ = other.auto_exposure_;


      auto_gain_ = other.auto_gain_;


      auto_shutter_ = other.auto_shutter_;


      exposure_ = other.exposure_;


      frame_rate_ = other.frame_rate_;


      from_ui_ = other.from_ui_;


      gain_ = other.gain_;


      shutter_ = other.shutter_;


      robot_side_ = other.robot_side_;

   }


   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public void setSequenceId(long sequence_id)
   {
      sequence_id_ = sequence_id;
   }
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long getSequenceId()
   {
      return sequence_id_;
   }


   public void setAutoExposure(boolean auto_exposure)
   {
      auto_exposure_ = auto_exposure;
   }
   public boolean getAutoExposure()
   {
      return auto_exposure_;
   }


   public void setAutoGain(boolean auto_gain)
   {
      auto_gain_ = auto_gain;
   }
   public boolean getAutoGain()
   {
      return auto_gain_;
   }


   public void setAutoShutter(boolean auto_shutter)
   {
      auto_shutter_ = auto_shutter;
   }
   public boolean getAutoShutter()
   {
      return auto_shutter_;
   }


   public void setExposure(double exposure)
   {
      exposure_ = exposure;
   }
   public double getExposure()
   {
      return exposure_;
   }


   public void setFrameRate(double frame_rate)
   {
      frame_rate_ = frame_rate;
   }
   public double getFrameRate()
   {
      return frame_rate_;
   }


   public void setFromUi(boolean from_ui)
   {
      from_ui_ = from_ui;
   }
   public boolean getFromUi()
   {
      return from_ui_;
   }


   public void setGain(double gain)
   {
      gain_ = gain;
   }
   public double getGain()
   {
      return gain_;
   }


   public void setShutter(double shutter)
   {
      shutter_ = shutter;
   }
   public double getShutter()
   {
      return shutter_;
   }


   public void setRobotSide(byte robot_side)
   {
      robot_side_ = robot_side;
   }
   public byte getRobotSide()
   {
      return robot_side_;
   }


   public static Supplier<BlackFlyParameterPacketPubSubType> getPubSubType()
   {
      return BlackFlyParameterPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return BlackFlyParameterPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(BlackFlyParameterPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.auto_exposure_, other.auto_exposure_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.auto_gain_, other.auto_gain_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.auto_shutter_, other.auto_shutter_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.exposure_, other.exposure_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.frame_rate_, other.frame_rate_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.from_ui_, other.from_ui_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.gain_, other.gain_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.shutter_, other.shutter_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof BlackFlyParameterPacket)) return false;

      BlackFlyParameterPacket otherMyClass = (BlackFlyParameterPacket) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.auto_exposure_ != otherMyClass.auto_exposure_) return false;


      if(this.auto_gain_ != otherMyClass.auto_gain_) return false;


      if(this.auto_shutter_ != otherMyClass.auto_shutter_) return false;


      if(this.exposure_ != otherMyClass.exposure_) return false;


      if(this.frame_rate_ != otherMyClass.frame_rate_) return false;


      if(this.from_ui_ != otherMyClass.from_ui_) return false;


      if(this.gain_ != otherMyClass.gain_) return false;


      if(this.shutter_ != otherMyClass.shutter_) return false;


      if(this.robot_side_ != otherMyClass.robot_side_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BlackFlyParameterPacket {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("auto_exposure=");
      builder.append(this.auto_exposure_);      builder.append(", ");

      builder.append("auto_gain=");
      builder.append(this.auto_gain_);      builder.append(", ");

      builder.append("auto_shutter=");
      builder.append(this.auto_shutter_);      builder.append(", ");

      builder.append("exposure=");
      builder.append(this.exposure_);      builder.append(", ");

      builder.append("frame_rate=");
      builder.append(this.frame_rate_);      builder.append(", ");

      builder.append("from_ui=");
      builder.append(this.from_ui_);      builder.append(", ");

      builder.append("gain=");
      builder.append(this.gain_);      builder.append(", ");

      builder.append("shutter=");
      builder.append(this.shutter_);      builder.append(", ");

      builder.append("robot_side=");
      builder.append(this.robot_side_);
      builder.append("}");
      return builder.toString();
   }
}
