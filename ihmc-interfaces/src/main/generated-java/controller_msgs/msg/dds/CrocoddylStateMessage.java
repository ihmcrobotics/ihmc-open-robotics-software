package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This is a message that holds data to describe the state.
       * 
       * The state is defined by:
       * * the state at the beginning of the interval, and
       * * the state's rate of change during the interval,
       * in which the state contains the robot's posture and velocity.
       * 
       * The initial state and its rate of change allow us to compute the
       * state in a desired time.
       * 
       * The order of the state is designed to be consistent with the URDF.
       * Quaternions can be accommodated due to the different dimensions of
       * the state and its rate of change.
       */
public class CrocoddylStateMessage extends Packet<CrocoddylStateMessage> implements Settable<CrocoddylStateMessage>, EpsilonComparable<CrocoddylStateMessage>
{
   /**
            * This represents the state at the beginning of the interval.
            * This is broken into (position, rate).
            * The first three of position are base position.
            * The next four are the quaternion rotation of the base.
            * The rate is (linear, angular)
            */
   public us.ihmc.idl.IDLSequence.Double  x_;
   /**
            * This represents the state's rate of change during the interval
            */
   public us.ihmc.idl.IDLSequence.Double  dx_;

   public CrocoddylStateMessage()
   {
      x_ = new us.ihmc.idl.IDLSequence.Double (100, "type_6");

      dx_ = new us.ihmc.idl.IDLSequence.Double (100, "type_6");

   }

   public CrocoddylStateMessage(CrocoddylStateMessage other)
   {
      this();
      set(other);
   }

   public void set(CrocoddylStateMessage other)
   {
      x_.set(other.x_);
      dx_.set(other.dx_);
   }


   /**
            * This represents the state at the beginning of the interval.
            * This is broken into (position, rate).
            * The first three of position are base position.
            * The next four are the quaternion rotation of the base.
            * The rate is (linear, angular)
            */
   public us.ihmc.idl.IDLSequence.Double  getX()
   {
      return x_;
   }


   /**
            * This represents the state's rate of change during the interval
            */
   public us.ihmc.idl.IDLSequence.Double  getDx()
   {
      return dx_;
   }


   public static Supplier<CrocoddylStateMessagePubSubType> getPubSubType()
   {
      return CrocoddylStateMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return CrocoddylStateMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(CrocoddylStateMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsDoubleSequence(this.x_, other.x_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsDoubleSequence(this.dx_, other.dx_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof CrocoddylStateMessage)) return false;

      CrocoddylStateMessage otherMyClass = (CrocoddylStateMessage) other;

      if (!this.x_.equals(otherMyClass.x_)) return false;
      if (!this.dx_.equals(otherMyClass.dx_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("CrocoddylStateMessage {");
      builder.append("x=");
      builder.append(this.x_);      builder.append(", ");
      builder.append("dx=");
      builder.append(this.dx_);
      builder.append("}");
      return builder.toString();
   }
}
