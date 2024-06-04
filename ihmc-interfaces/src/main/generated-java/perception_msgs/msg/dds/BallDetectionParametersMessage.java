package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class BallDetectionParametersMessage extends Packet<BallDetectionParametersMessage> implements Settable<BallDetectionParametersMessage>, EpsilonComparable<BallDetectionParametersMessage>
{
   /**
            * Diameter of ball being detected, in meters
            */
   public double ball_diameter_;
   /**
            * Position alpha filter value
            */
   public double alpha_;
   /**
            * HSV lower bounds
            */
   public double h_low_;
   public double s_low_;
   public double v_low_;
   /**
            * HSV upper bounds
            */
   public double h_high_;
   public double s_high_;
   public double v_high_;

   public BallDetectionParametersMessage()
   {
   }

   public BallDetectionParametersMessage(BallDetectionParametersMessage other)
   {
      this();
      set(other);
   }

   public void set(BallDetectionParametersMessage other)
   {
      ball_diameter_ = other.ball_diameter_;

      alpha_ = other.alpha_;

      h_low_ = other.h_low_;

      s_low_ = other.s_low_;

      v_low_ = other.v_low_;

      h_high_ = other.h_high_;

      s_high_ = other.s_high_;

      v_high_ = other.v_high_;

   }

   /**
            * Diameter of ball being detected, in meters
            */
   public void setBallDiameter(double ball_diameter)
   {
      ball_diameter_ = ball_diameter;
   }
   /**
            * Diameter of ball being detected, in meters
            */
   public double getBallDiameter()
   {
      return ball_diameter_;
   }

   /**
            * Position alpha filter value
            */
   public void setAlpha(double alpha)
   {
      alpha_ = alpha;
   }
   /**
            * Position alpha filter value
            */
   public double getAlpha()
   {
      return alpha_;
   }

   /**
            * HSV lower bounds
            */
   public void setHLow(double h_low)
   {
      h_low_ = h_low;
   }
   /**
            * HSV lower bounds
            */
   public double getHLow()
   {
      return h_low_;
   }

   public void setSLow(double s_low)
   {
      s_low_ = s_low;
   }
   public double getSLow()
   {
      return s_low_;
   }

   public void setVLow(double v_low)
   {
      v_low_ = v_low;
   }
   public double getVLow()
   {
      return v_low_;
   }

   /**
            * HSV upper bounds
            */
   public void setHHigh(double h_high)
   {
      h_high_ = h_high;
   }
   /**
            * HSV upper bounds
            */
   public double getHHigh()
   {
      return h_high_;
   }

   public void setSHigh(double s_high)
   {
      s_high_ = s_high;
   }
   public double getSHigh()
   {
      return s_high_;
   }

   public void setVHigh(double v_high)
   {
      v_high_ = v_high;
   }
   public double getVHigh()
   {
      return v_high_;
   }


   public static Supplier<BallDetectionParametersMessagePubSubType> getPubSubType()
   {
      return BallDetectionParametersMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return BallDetectionParametersMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(BallDetectionParametersMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.ball_diameter_, other.ball_diameter_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.alpha_, other.alpha_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.h_low_, other.h_low_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.s_low_, other.s_low_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.v_low_, other.v_low_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.h_high_, other.h_high_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.s_high_, other.s_high_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.v_high_, other.v_high_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof BallDetectionParametersMessage)) return false;

      BallDetectionParametersMessage otherMyClass = (BallDetectionParametersMessage) other;

      if(this.ball_diameter_ != otherMyClass.ball_diameter_) return false;

      if(this.alpha_ != otherMyClass.alpha_) return false;

      if(this.h_low_ != otherMyClass.h_low_) return false;

      if(this.s_low_ != otherMyClass.s_low_) return false;

      if(this.v_low_ != otherMyClass.v_low_) return false;

      if(this.h_high_ != otherMyClass.h_high_) return false;

      if(this.s_high_ != otherMyClass.s_high_) return false;

      if(this.v_high_ != otherMyClass.v_high_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BallDetectionParametersMessage {");
      builder.append("ball_diameter=");
      builder.append(this.ball_diameter_);      builder.append(", ");
      builder.append("alpha=");
      builder.append(this.alpha_);      builder.append(", ");
      builder.append("h_low=");
      builder.append(this.h_low_);      builder.append(", ");
      builder.append("s_low=");
      builder.append(this.s_low_);      builder.append(", ");
      builder.append("v_low=");
      builder.append(this.v_low_);      builder.append(", ");
      builder.append("h_high=");
      builder.append(this.h_high_);      builder.append(", ");
      builder.append("s_high=");
      builder.append(this.s_high_);      builder.append(", ");
      builder.append("v_high=");
      builder.append(this.v_high_);
      builder.append("}");
      return builder.toString();
   }
}
