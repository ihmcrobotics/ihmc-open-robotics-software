package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * This message allows the user to precisely choose the priority of each component of a taskspace command.
       */
public class WeightMatrix3DMessage extends Packet<WeightMatrix3DMessage> implements Settable<WeightMatrix3DMessage>, EpsilonComparable<WeightMatrix3DMessage>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   /**
            * The ID of the reference frame defining the weight frame.
            * This reference frame defines the x axis, y axis, z axis for the weights.
            * This frame is optional. It is preferable to provide it when possible, but when it is absent, i.e. equal to 0,
            * the weight matrix will then be generated regardless to what frame is it used in.
            */
   public long weight_frame_id_;

   /**
            * Specifies the qp weight for the x-axis, if set to NaN or to a negative value the controller will use the default weight for this axis. The weight is -1 by default.
            */
   public double x_weight_ = -1.0;

   /**
            * Specifies the qp weight for the y-axis, if set to NaN or to a negative value the controller will use the default weight for this axis. The weight is -1 by default.
            */
   public double y_weight_ = -1.0;

   /**
            * Specifies the qp weight for the z-axis, if set to NaN or to a negative value the controller will use the default weight for this axis. The weight is -1 by default.
            */
   public double z_weight_ = -1.0;

   public WeightMatrix3DMessage()
   {






   }

   public WeightMatrix3DMessage(WeightMatrix3DMessage other)
   {
      this();
      set(other);
   }

   public void set(WeightMatrix3DMessage other)
   {

      sequence_id_ = other.sequence_id_;


      weight_frame_id_ = other.weight_frame_id_;


      x_weight_ = other.x_weight_;


      y_weight_ = other.y_weight_;


      z_weight_ = other.z_weight_;

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


   /**
            * The ID of the reference frame defining the weight frame.
            * This reference frame defines the x axis, y axis, z axis for the weights.
            * This frame is optional. It is preferable to provide it when possible, but when it is absent, i.e. equal to 0,
            * the weight matrix will then be generated regardless to what frame is it used in.
            */
   public void setWeightFrameId(long weight_frame_id)
   {
      weight_frame_id_ = weight_frame_id;
   }
   /**
            * The ID of the reference frame defining the weight frame.
            * This reference frame defines the x axis, y axis, z axis for the weights.
            * This frame is optional. It is preferable to provide it when possible, but when it is absent, i.e. equal to 0,
            * the weight matrix will then be generated regardless to what frame is it used in.
            */
   public long getWeightFrameId()
   {
      return weight_frame_id_;
   }


   /**
            * Specifies the qp weight for the x-axis, if set to NaN or to a negative value the controller will use the default weight for this axis. The weight is -1 by default.
            */
   public void setXWeight(double x_weight)
   {
      x_weight_ = x_weight;
   }
   /**
            * Specifies the qp weight for the x-axis, if set to NaN or to a negative value the controller will use the default weight for this axis. The weight is -1 by default.
            */
   public double getXWeight()
   {
      return x_weight_;
   }


   /**
            * Specifies the qp weight for the y-axis, if set to NaN or to a negative value the controller will use the default weight for this axis. The weight is -1 by default.
            */
   public void setYWeight(double y_weight)
   {
      y_weight_ = y_weight;
   }
   /**
            * Specifies the qp weight for the y-axis, if set to NaN or to a negative value the controller will use the default weight for this axis. The weight is -1 by default.
            */
   public double getYWeight()
   {
      return y_weight_;
   }


   /**
            * Specifies the qp weight for the z-axis, if set to NaN or to a negative value the controller will use the default weight for this axis. The weight is -1 by default.
            */
   public void setZWeight(double z_weight)
   {
      z_weight_ = z_weight;
   }
   /**
            * Specifies the qp weight for the z-axis, if set to NaN or to a negative value the controller will use the default weight for this axis. The weight is -1 by default.
            */
   public double getZWeight()
   {
      return z_weight_;
   }


   public static Supplier<WeightMatrix3DMessagePubSubType> getPubSubType()
   {
      return WeightMatrix3DMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return WeightMatrix3DMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(WeightMatrix3DMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.weight_frame_id_, other.weight_frame_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.x_weight_, other.x_weight_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.y_weight_, other.y_weight_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.z_weight_, other.z_weight_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof WeightMatrix3DMessage)) return false;

      WeightMatrix3DMessage otherMyClass = (WeightMatrix3DMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.weight_frame_id_ != otherMyClass.weight_frame_id_) return false;


      if(this.x_weight_ != otherMyClass.x_weight_) return false;


      if(this.y_weight_ != otherMyClass.y_weight_) return false;


      if(this.z_weight_ != otherMyClass.z_weight_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("WeightMatrix3DMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("weight_frame_id=");
      builder.append(this.weight_frame_id_);      builder.append(", ");

      builder.append("x_weight=");
      builder.append(this.x_weight_);      builder.append(", ");

      builder.append("y_weight=");
      builder.append(this.y_weight_);      builder.append(", ");

      builder.append("z_weight=");
      builder.append(this.z_weight_);
      builder.append("}");
      return builder.toString();
   }
}
