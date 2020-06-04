package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is used to provides additional properties for cameras.
       */
public class IntrinsicParametersMessage extends Packet<IntrinsicParametersMessage> implements Settable<IntrinsicParametersMessage>, EpsilonComparable<IntrinsicParametersMessage>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public int width_;

   public int height_;

   public double fx_;

   public double fy_;

   public double skew_;

   public double cx_;

   public double cy_;

   public us.ihmc.idl.IDLSequence.Double  radial_;

   public double t1_;

   public double t2_;

   public IntrinsicParametersMessage()
   {









      radial_ = new us.ihmc.idl.IDLSequence.Double (100, "type_6");




   }

   public IntrinsicParametersMessage(IntrinsicParametersMessage other)
   {
      this();
      set(other);
   }

   public void set(IntrinsicParametersMessage other)
   {

      sequence_id_ = other.sequence_id_;


      width_ = other.width_;


      height_ = other.height_;


      fx_ = other.fx_;


      fy_ = other.fy_;


      skew_ = other.skew_;


      cx_ = other.cx_;


      cy_ = other.cy_;


      radial_.set(other.radial_);

      t1_ = other.t1_;


      t2_ = other.t2_;

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


   public void setWidth(int width)
   {
      width_ = width;
   }
   public int getWidth()
   {
      return width_;
   }


   public void setHeight(int height)
   {
      height_ = height;
   }
   public int getHeight()
   {
      return height_;
   }


   public void setFx(double fx)
   {
      fx_ = fx;
   }
   public double getFx()
   {
      return fx_;
   }


   public void setFy(double fy)
   {
      fy_ = fy;
   }
   public double getFy()
   {
      return fy_;
   }


   public void setSkew(double skew)
   {
      skew_ = skew;
   }
   public double getSkew()
   {
      return skew_;
   }


   public void setCx(double cx)
   {
      cx_ = cx;
   }
   public double getCx()
   {
      return cx_;
   }


   public void setCy(double cy)
   {
      cy_ = cy;
   }
   public double getCy()
   {
      return cy_;
   }



   public us.ihmc.idl.IDLSequence.Double  getRadial()
   {
      return radial_;
   }


   public void setT1(double t1)
   {
      t1_ = t1;
   }
   public double getT1()
   {
      return t1_;
   }


   public void setT2(double t2)
   {
      t2_ = t2;
   }
   public double getT2()
   {
      return t2_;
   }


   public static Supplier<IntrinsicParametersMessagePubSubType> getPubSubType()
   {
      return IntrinsicParametersMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return IntrinsicParametersMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(IntrinsicParametersMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.width_, other.width_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.height_, other.height_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.fx_, other.fx_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.fy_, other.fy_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.skew_, other.skew_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.cx_, other.cx_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.cy_, other.cy_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsDoubleSequence(this.radial_, other.radial_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.t1_, other.t1_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.t2_, other.t2_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof IntrinsicParametersMessage)) return false;

      IntrinsicParametersMessage otherMyClass = (IntrinsicParametersMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.width_ != otherMyClass.width_) return false;


      if(this.height_ != otherMyClass.height_) return false;


      if(this.fx_ != otherMyClass.fx_) return false;


      if(this.fy_ != otherMyClass.fy_) return false;


      if(this.skew_ != otherMyClass.skew_) return false;


      if(this.cx_ != otherMyClass.cx_) return false;


      if(this.cy_ != otherMyClass.cy_) return false;


      if (!this.radial_.equals(otherMyClass.radial_)) return false;

      if(this.t1_ != otherMyClass.t1_) return false;


      if(this.t2_ != otherMyClass.t2_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("IntrinsicParametersMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("width=");
      builder.append(this.width_);      builder.append(", ");

      builder.append("height=");
      builder.append(this.height_);      builder.append(", ");

      builder.append("fx=");
      builder.append(this.fx_);      builder.append(", ");

      builder.append("fy=");
      builder.append(this.fy_);      builder.append(", ");

      builder.append("skew=");
      builder.append(this.skew_);      builder.append(", ");

      builder.append("cx=");
      builder.append(this.cx_);      builder.append(", ");

      builder.append("cy=");
      builder.append(this.cy_);      builder.append(", ");

      builder.append("radial=");
      builder.append(this.radial_);      builder.append(", ");

      builder.append("t1=");
      builder.append(this.t1_);      builder.append(", ");

      builder.append("t2=");
      builder.append(this.t2_);
      builder.append("}");
      return builder.toString();
   }
}
