package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message describes the feedback gain computed by the solver.
       * 
       * A state-feedback gain is used to apply state feedback control.
       * Generally speaking, this is a matrix defined by the dimension of state and control spaces.
       * Both dimensions need to described in nx and nu, respectively.
       * 
       * The values of this gain are stored in data array.
       * The entries of the matrix are stored in row-major order.
       * The entire first row is stored first, followed by the entire second row, and so on.
       */
public class CrocoddylFeedbackGainMessage extends Packet<CrocoddylFeedbackGainMessage> implements Settable<CrocoddylFeedbackGainMessage>, EpsilonComparable<CrocoddylFeedbackGainMessage>
{
   /**
            * This represents the dimension of the state and control
            * number of columns
            */
   public long nx_;
   /**
            * number of rows
            */
   public long nu_;
   /**
            * This represents the entries of the matrix allocated by row order.
            */
   public us.ihmc.idl.IDLSequence.Double  data_;

   public CrocoddylFeedbackGainMessage()
   {
      data_ = new us.ihmc.idl.IDLSequence.Double (100, "type_6");

   }

   public CrocoddylFeedbackGainMessage(CrocoddylFeedbackGainMessage other)
   {
      this();
      set(other);
   }

   public void set(CrocoddylFeedbackGainMessage other)
   {
      nx_ = other.nx_;

      nu_ = other.nu_;

      data_.set(other.data_);
   }

   /**
            * This represents the dimension of the state and control
            * number of columns
            */
   public void setNx(long nx)
   {
      nx_ = nx;
   }
   /**
            * This represents the dimension of the state and control
            * number of columns
            */
   public long getNx()
   {
      return nx_;
   }

   /**
            * number of rows
            */
   public void setNu(long nu)
   {
      nu_ = nu;
   }
   /**
            * number of rows
            */
   public long getNu()
   {
      return nu_;
   }


   /**
            * This represents the entries of the matrix allocated by row order.
            */
   public us.ihmc.idl.IDLSequence.Double  getData()
   {
      return data_;
   }


   public static Supplier<CrocoddylFeedbackGainMessagePubSubType> getPubSubType()
   {
      return CrocoddylFeedbackGainMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return CrocoddylFeedbackGainMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(CrocoddylFeedbackGainMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.nx_, other.nx_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.nu_, other.nu_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsDoubleSequence(this.data_, other.data_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof CrocoddylFeedbackGainMessage)) return false;

      CrocoddylFeedbackGainMessage otherMyClass = (CrocoddylFeedbackGainMessage) other;

      if(this.nx_ != otherMyClass.nx_) return false;

      if(this.nu_ != otherMyClass.nu_) return false;

      if (!this.data_.equals(otherMyClass.data_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("CrocoddylFeedbackGainMessage {");
      builder.append("nx=");
      builder.append(this.nx_);      builder.append(", ");
      builder.append("nu=");
      builder.append(this.nu_);      builder.append(", ");
      builder.append("data=");
      builder.append(this.data_);
      builder.append("}");
      return builder.toString();
   }
}
