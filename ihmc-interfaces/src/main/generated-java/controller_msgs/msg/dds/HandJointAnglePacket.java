package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Message used to report the current joint angles for the fingers of a hand.
       */
public class HandJointAnglePacket extends Packet<HandJointAnglePacket> implements Settable<HandJointAnglePacket>, EpsilonComparable<HandJointAnglePacket>
{

   public static final byte ROBOT_SIDE_LEFT = (byte) 0;

   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public byte robot_side_ = (byte) 255;

   public us.ihmc.idl.IDLSequence.Double  joint_angles_;

   public boolean connected_;

   public boolean calibrated_;

   public HandJointAnglePacket()
   {



      joint_angles_ = new us.ihmc.idl.IDLSequence.Double (100, "type_6");




   }

   public HandJointAnglePacket(HandJointAnglePacket other)
   {
      this();
      set(other);
   }

   public void set(HandJointAnglePacket other)
   {

      sequence_id_ = other.sequence_id_;


      robot_side_ = other.robot_side_;


      joint_angles_.set(other.joint_angles_);

      connected_ = other.connected_;


      calibrated_ = other.calibrated_;

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


   public void setRobotSide(byte robot_side)
   {
      robot_side_ = robot_side;
   }
   public byte getRobotSide()
   {
      return robot_side_;
   }



   public us.ihmc.idl.IDLSequence.Double  getJointAngles()
   {
      return joint_angles_;
   }


   public void setConnected(boolean connected)
   {
      connected_ = connected;
   }
   public boolean getConnected()
   {
      return connected_;
   }


   public void setCalibrated(boolean calibrated)
   {
      calibrated_ = calibrated;
   }
   public boolean getCalibrated()
   {
      return calibrated_;
   }


   public static Supplier<HandJointAnglePacketPubSubType> getPubSubType()
   {
      return HandJointAnglePacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return HandJointAnglePacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(HandJointAnglePacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsDoubleSequence(this.joint_angles_, other.joint_angles_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.connected_, other.connected_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.calibrated_, other.calibrated_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof HandJointAnglePacket)) return false;

      HandJointAnglePacket otherMyClass = (HandJointAnglePacket) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.robot_side_ != otherMyClass.robot_side_) return false;


      if (!this.joint_angles_.equals(otherMyClass.joint_angles_)) return false;

      if(this.connected_ != otherMyClass.connected_) return false;


      if(this.calibrated_ != otherMyClass.calibrated_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("HandJointAnglePacket {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");

      builder.append("joint_angles=");
      builder.append(this.joint_angles_);      builder.append(", ");

      builder.append("connected=");
      builder.append(this.connected_);      builder.append(", ");

      builder.append("calibrated=");
      builder.append(this.calibrated_);
      builder.append("}");
      return builder.toString();
   }
}
