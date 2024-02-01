package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This is a message that holds data to describe the control.
       * 
       * The control is defined by:
       * * the type of control input,
       * * the type of control parametrization,
       * * the control vector, and
       * * the feedback gain
       * 
       * This information allow us to compute the control in a desired time.
       * The order of the control is designed to be consistent with the URDF.
       */
public class CrocoddylControlMessage extends Packet<CrocoddylControlMessage> implements Settable<CrocoddylControlMessage>, EpsilonComparable<CrocoddylControlMessage>
{
   /**
          * Type of control inputs
          */
   public static final byte EFFORT = (byte) 0;
   public static final byte ACCELERATION_CONTACTFORCE = (byte) 1;
   /**
          * Type of control parametrization
          */
   public static final byte POLYZERO = (byte) 0;
   public static final byte POLYONE = (byte) 1;
   public static final byte POLYTWO = (byte) 2;
   public byte input_;
   public byte parametrization_;
   /**
            * This represents the control input vector
            */
   public us.ihmc.idl.IDLSequence.Double  u_;
   /**
            * This represents the feedback gain
            */
   public controller_msgs.msg.dds.CrocoddylFeedbackGainMessage gain_;

   public CrocoddylControlMessage()
   {
      u_ = new us.ihmc.idl.IDLSequence.Double (100, "type_6");

      gain_ = new controller_msgs.msg.dds.CrocoddylFeedbackGainMessage();
   }

   public CrocoddylControlMessage(CrocoddylControlMessage other)
   {
      this();
      set(other);
   }

   public void set(CrocoddylControlMessage other)
   {
      input_ = other.input_;

      parametrization_ = other.parametrization_;

      u_.set(other.u_);
      controller_msgs.msg.dds.CrocoddylFeedbackGainMessagePubSubType.staticCopy(other.gain_, gain_);
   }

   public void setInput(byte input)
   {
      input_ = input;
   }
   public byte getInput()
   {
      return input_;
   }

   public void setParametrization(byte parametrization)
   {
      parametrization_ = parametrization;
   }
   public byte getParametrization()
   {
      return parametrization_;
   }


   /**
            * This represents the control input vector
            */
   public us.ihmc.idl.IDLSequence.Double  getU()
   {
      return u_;
   }


   /**
            * This represents the feedback gain
            */
   public controller_msgs.msg.dds.CrocoddylFeedbackGainMessage getGain()
   {
      return gain_;
   }


   public static Supplier<CrocoddylControlMessagePubSubType> getPubSubType()
   {
      return CrocoddylControlMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return CrocoddylControlMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(CrocoddylControlMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.input_, other.input_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.parametrization_, other.parametrization_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsDoubleSequence(this.u_, other.u_, epsilon)) return false;

      if (!this.gain_.epsilonEquals(other.gain_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof CrocoddylControlMessage)) return false;

      CrocoddylControlMessage otherMyClass = (CrocoddylControlMessage) other;

      if(this.input_ != otherMyClass.input_) return false;

      if(this.parametrization_ != otherMyClass.parametrization_) return false;

      if (!this.u_.equals(otherMyClass.u_)) return false;
      if (!this.gain_.equals(otherMyClass.gain_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("CrocoddylControlMessage {");
      builder.append("input=");
      builder.append(this.input_);      builder.append(", ");
      builder.append("parametrization=");
      builder.append(this.parametrization_);      builder.append(", ");
      builder.append("u=");
      builder.append(this.u_);      builder.append(", ");
      builder.append("gain=");
      builder.append(this.gain_);
      builder.append("}");
      return builder.toString();
   }
}
