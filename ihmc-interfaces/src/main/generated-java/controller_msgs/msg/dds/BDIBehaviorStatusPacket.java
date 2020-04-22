package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Atlas specific message.
       */
public class BDIBehaviorStatusPacket extends Packet<BDIBehaviorStatusPacket> implements Settable<BDIBehaviorStatusPacket>, EpsilonComparable<BDIBehaviorStatusPacket>
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

   public byte current_bdi_robot_behavior_ = (byte) 255;

   public BDIBehaviorStatusPacket()
   {



   }

   public BDIBehaviorStatusPacket(BDIBehaviorStatusPacket other)
   {
      this();
      set(other);
   }

   public void set(BDIBehaviorStatusPacket other)
   {

      sequence_id_ = other.sequence_id_;


      current_bdi_robot_behavior_ = other.current_bdi_robot_behavior_;

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


   public void setCurrentBdiRobotBehavior(byte current_bdi_robot_behavior)
   {
      current_bdi_robot_behavior_ = current_bdi_robot_behavior;
   }
   public byte getCurrentBdiRobotBehavior()
   {
      return current_bdi_robot_behavior_;
   }


   public static Supplier<BDIBehaviorStatusPacketPubSubType> getPubSubType()
   {
      return BDIBehaviorStatusPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return BDIBehaviorStatusPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(BDIBehaviorStatusPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_bdi_robot_behavior_, other.current_bdi_robot_behavior_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof BDIBehaviorStatusPacket)) return false;

      BDIBehaviorStatusPacket otherMyClass = (BDIBehaviorStatusPacket) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.current_bdi_robot_behavior_ != otherMyClass.current_bdi_robot_behavior_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BDIBehaviorStatusPacket {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("current_bdi_robot_behavior=");
      builder.append(this.current_bdi_robot_behavior_);
      builder.append("}");
      return builder.toString();
   }
}
