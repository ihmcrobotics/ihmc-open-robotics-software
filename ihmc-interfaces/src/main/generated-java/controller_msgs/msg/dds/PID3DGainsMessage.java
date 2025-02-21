package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * this represents the 3d gains for a PID controller
       */
public class PID3DGainsMessage extends Packet<PID3DGainsMessage> implements Settable<PID3DGainsMessage>, EpsilonComparable<PID3DGainsMessage>
{
   public controller_msgs.msg.dds.PIDGainsMessage gains_x_;
   public controller_msgs.msg.dds.PIDGainsMessage gains_y_;
   public controller_msgs.msg.dds.PIDGainsMessage gains_z_;
   public double maximum_feedback_;
   public double maximum_feedback_rate_;
   public double maximum_integral_error_;
   public double maximum_derivative_error_;
   public double maximum_proportional_error_;

   public PID3DGainsMessage()
   {
      gains_x_ = new controller_msgs.msg.dds.PIDGainsMessage();
      gains_y_ = new controller_msgs.msg.dds.PIDGainsMessage();
      gains_z_ = new controller_msgs.msg.dds.PIDGainsMessage();
   }

   public PID3DGainsMessage(PID3DGainsMessage other)
   {
      this();
      set(other);
   }

   public void set(PID3DGainsMessage other)
   {
      controller_msgs.msg.dds.PIDGainsMessagePubSubType.staticCopy(other.gains_x_, gains_x_);
      controller_msgs.msg.dds.PIDGainsMessagePubSubType.staticCopy(other.gains_y_, gains_y_);
      controller_msgs.msg.dds.PIDGainsMessagePubSubType.staticCopy(other.gains_z_, gains_z_);
      maximum_feedback_ = other.maximum_feedback_;

      maximum_feedback_rate_ = other.maximum_feedback_rate_;

      maximum_integral_error_ = other.maximum_integral_error_;

      maximum_derivative_error_ = other.maximum_derivative_error_;

      maximum_proportional_error_ = other.maximum_proportional_error_;

   }


   public controller_msgs.msg.dds.PIDGainsMessage getGainsX()
   {
      return gains_x_;
   }


   public controller_msgs.msg.dds.PIDGainsMessage getGainsY()
   {
      return gains_y_;
   }


   public controller_msgs.msg.dds.PIDGainsMessage getGainsZ()
   {
      return gains_z_;
   }

   public void setMaximumFeedback(double maximum_feedback)
   {
      maximum_feedback_ = maximum_feedback;
   }
   public double getMaximumFeedback()
   {
      return maximum_feedback_;
   }

   public void setMaximumFeedbackRate(double maximum_feedback_rate)
   {
      maximum_feedback_rate_ = maximum_feedback_rate;
   }
   public double getMaximumFeedbackRate()
   {
      return maximum_feedback_rate_;
   }

   public void setMaximumIntegralError(double maximum_integral_error)
   {
      maximum_integral_error_ = maximum_integral_error;
   }
   public double getMaximumIntegralError()
   {
      return maximum_integral_error_;
   }

   public void setMaximumDerivativeError(double maximum_derivative_error)
   {
      maximum_derivative_error_ = maximum_derivative_error;
   }
   public double getMaximumDerivativeError()
   {
      return maximum_derivative_error_;
   }

   public void setMaximumProportionalError(double maximum_proportional_error)
   {
      maximum_proportional_error_ = maximum_proportional_error;
   }
   public double getMaximumProportionalError()
   {
      return maximum_proportional_error_;
   }


   public static Supplier<PID3DGainsMessagePubSubType> getPubSubType()
   {
      return PID3DGainsMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return PID3DGainsMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(PID3DGainsMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.gains_x_.epsilonEquals(other.gains_x_, epsilon)) return false;
      if (!this.gains_y_.epsilonEquals(other.gains_y_, epsilon)) return false;
      if (!this.gains_z_.epsilonEquals(other.gains_z_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_feedback_, other.maximum_feedback_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_feedback_rate_, other.maximum_feedback_rate_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_integral_error_, other.maximum_integral_error_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_derivative_error_, other.maximum_derivative_error_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_proportional_error_, other.maximum_proportional_error_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof PID3DGainsMessage)) return false;

      PID3DGainsMessage otherMyClass = (PID3DGainsMessage) other;

      if (!this.gains_x_.equals(otherMyClass.gains_x_)) return false;
      if (!this.gains_y_.equals(otherMyClass.gains_y_)) return false;
      if (!this.gains_z_.equals(otherMyClass.gains_z_)) return false;
      if(this.maximum_feedback_ != otherMyClass.maximum_feedback_) return false;

      if(this.maximum_feedback_rate_ != otherMyClass.maximum_feedback_rate_) return false;

      if(this.maximum_integral_error_ != otherMyClass.maximum_integral_error_) return false;

      if(this.maximum_derivative_error_ != otherMyClass.maximum_derivative_error_) return false;

      if(this.maximum_proportional_error_ != otherMyClass.maximum_proportional_error_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("PID3DGainsMessage {");
      builder.append("gains_x=");
      builder.append(this.gains_x_);      builder.append(", ");
      builder.append("gains_y=");
      builder.append(this.gains_y_);      builder.append(", ");
      builder.append("gains_z=");
      builder.append(this.gains_z_);      builder.append(", ");
      builder.append("maximum_feedback=");
      builder.append(this.maximum_feedback_);      builder.append(", ");
      builder.append("maximum_feedback_rate=");
      builder.append(this.maximum_feedback_rate_);      builder.append(", ");
      builder.append("maximum_integral_error=");
      builder.append(this.maximum_integral_error_);      builder.append(", ");
      builder.append("maximum_derivative_error=");
      builder.append(this.maximum_derivative_error_);      builder.append(", ");
      builder.append("maximum_proportional_error=");
      builder.append(this.maximum_proportional_error_);
      builder.append("}");
      return builder.toString();
   }
}
