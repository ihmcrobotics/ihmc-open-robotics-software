package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Atlas specific message.
       */
public class BDIBehaviorCommandPacket extends Packet<BDIBehaviorCommandPacket> implements Settable<BDIBehaviorCommandPacket>, EpsilonComparable<BDIBehaviorCommandPacket>
{

   public static final byte NONE = (byte) 0;

   public static final byte FREEZE = (byte) 1;

   public static final byte STAND_PREP = (byte) 2;

   public static final byte STAND = (byte) 3;

   public static final byte WALK = (byte) 4;

   public static final byte STEP = (byte) 5;

   public static final byte MANIPULATE = (byte) 6;

   public static final byte USER = (byte) 7;

   public static final byte CALIBRATE = (byte) 8;

   public static final byte SOFT_STOP = (byte) 9;

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public byte atlas_bdi_robot_behavior_ = (byte) 255;

   public boolean stop_;

   public BDIBehaviorCommandPacket()
   {




   }

   public BDIBehaviorCommandPacket(BDIBehaviorCommandPacket other)
   {
      this();
      set(other);
   }

   public void set(BDIBehaviorCommandPacket other)
   {

      sequence_id_ = other.sequence_id_;


      atlas_bdi_robot_behavior_ = other.atlas_bdi_robot_behavior_;


      stop_ = other.stop_;

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


   public void setAtlasBdiRobotBehavior(byte atlas_bdi_robot_behavior)
   {
      atlas_bdi_robot_behavior_ = atlas_bdi_robot_behavior;
   }
   public byte getAtlasBdiRobotBehavior()
   {
      return atlas_bdi_robot_behavior_;
   }


   public void setStop(boolean stop)
   {
      stop_ = stop;
   }
   public boolean getStop()
   {
      return stop_;
   }


   public static Supplier<BDIBehaviorCommandPacketPubSubType> getPubSubType()
   {
      return BDIBehaviorCommandPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return BDIBehaviorCommandPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(BDIBehaviorCommandPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.atlas_bdi_robot_behavior_, other.atlas_bdi_robot_behavior_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.stop_, other.stop_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof BDIBehaviorCommandPacket)) return false;

      BDIBehaviorCommandPacket otherMyClass = (BDIBehaviorCommandPacket) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.atlas_bdi_robot_behavior_ != otherMyClass.atlas_bdi_robot_behavior_) return false;


      if(this.stop_ != otherMyClass.stop_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BDIBehaviorCommandPacket {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("atlas_bdi_robot_behavior=");
      builder.append(this.atlas_bdi_robot_behavior_);      builder.append(", ");

      builder.append("stop=");
      builder.append(this.stop_);
      builder.append("}");
      return builder.toString();
   }
}
