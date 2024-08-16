package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * This is a lightweight version of JointspaceTrajectoryMessage designed for streaming.
       */
public class JointspaceStreamingMessage extends Packet<JointspaceStreamingMessage> implements Settable<JointspaceStreamingMessage>, EpsilonComparable<JointspaceStreamingMessage>
{
   public us.ihmc.idl.IDLSequence.Float  positions_;
   public us.ihmc.idl.IDLSequence.Float  velocities_;
   public us.ihmc.idl.IDLSequence.Float  accelerations_;

   public JointspaceStreamingMessage()
   {
      positions_ = new us.ihmc.idl.IDLSequence.Float (12, "type_5");

      velocities_ = new us.ihmc.idl.IDLSequence.Float (12, "type_5");

      accelerations_ = new us.ihmc.idl.IDLSequence.Float (12, "type_5");

   }

   public JointspaceStreamingMessage(JointspaceStreamingMessage other)
   {
      this();
      set(other);
   }

   public void set(JointspaceStreamingMessage other)
   {
      positions_.set(other.positions_);
      velocities_.set(other.velocities_);
      accelerations_.set(other.accelerations_);
   }


   public us.ihmc.idl.IDLSequence.Float  getPositions()
   {
      return positions_;
   }


   public us.ihmc.idl.IDLSequence.Float  getVelocities()
   {
      return velocities_;
   }


   public us.ihmc.idl.IDLSequence.Float  getAccelerations()
   {
      return accelerations_;
   }


   public static Supplier<JointspaceStreamingMessagePubSubType> getPubSubType()
   {
      return JointspaceStreamingMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return JointspaceStreamingMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(JointspaceStreamingMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.positions_, other.positions_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.velocities_, other.velocities_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.accelerations_, other.accelerations_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof JointspaceStreamingMessage)) return false;

      JointspaceStreamingMessage otherMyClass = (JointspaceStreamingMessage) other;

      if (!this.positions_.equals(otherMyClass.positions_)) return false;
      if (!this.velocities_.equals(otherMyClass.velocities_)) return false;
      if (!this.accelerations_.equals(otherMyClass.accelerations_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("JointspaceStreamingMessage {");
      builder.append("positions=");
      builder.append(this.positions_);      builder.append(", ");
      builder.append("velocities=");
      builder.append(this.velocities_);      builder.append(", ");
      builder.append("accelerations=");
      builder.append(this.accelerations_);
      builder.append("}");
      return builder.toString();
   }
}
