package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * The controller sends this message to notify that it has shifted the remaining footsteps to be executed due to some execution error.
       */
public class PlanOffsetStatus extends Packet<PlanOffsetStatus> implements Settable<PlanOffsetStatus>, EpsilonComparable<PlanOffsetStatus>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   /**
            * The amount by which the remaining footsteps have been translated.
            */
   public us.ihmc.euclid.tuple3D.Vector3D offset_vector_;

   public PlanOffsetStatus()
   {


      offset_vector_ = new us.ihmc.euclid.tuple3D.Vector3D();

   }

   public PlanOffsetStatus(PlanOffsetStatus other)
   {
      this();
      set(other);
   }

   public void set(PlanOffsetStatus other)
   {

      sequence_id_ = other.sequence_id_;


      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.offset_vector_, offset_vector_);
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
            * The amount by which the remaining footsteps have been translated.
            */
   public us.ihmc.euclid.tuple3D.Vector3D getOffsetVector()
   {
      return offset_vector_;
   }


   public static Supplier<PlanOffsetStatusPubSubType> getPubSubType()
   {
      return PlanOffsetStatusPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return PlanOffsetStatusPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(PlanOffsetStatus other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!this.offset_vector_.epsilonEquals(other.offset_vector_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof PlanOffsetStatus)) return false;

      PlanOffsetStatus otherMyClass = (PlanOffsetStatus) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if (!this.offset_vector_.equals(otherMyClass.offset_vector_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("PlanOffsetStatus {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("offset_vector=");
      builder.append(this.offset_vector_);
      builder.append("}");
      return builder.toString();
   }
}
