package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC footstep planner API.
       * The footstep planner sends this message to notify the user of the status of planning.
       */
public class FootstepPlannerStatusMessage extends Packet<FootstepPlannerStatusMessage> implements Settable<FootstepPlannerStatusMessage>, EpsilonComparable<FootstepPlannerStatusMessage>
{
   public static final byte IDLE = (byte) 0;
   public static final byte PLANNING_PATH = (byte) 1;
   public static final byte PLANNING_STEP = (byte) 2;
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Status of planning.
            */
   public byte planning_status_ = (byte) 255;

   public FootstepPlannerStatusMessage()
   {
   }

   public FootstepPlannerStatusMessage(FootstepPlannerStatusMessage other)
   {
      this();
      set(other);
   }

   public void set(FootstepPlannerStatusMessage other)
   {
      sequence_id_ = other.sequence_id_;

      planning_status_ = other.planning_status_;

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
            * Status of planning.
            */
   public void setPlanningStatus(byte planning_status)
   {
      planning_status_ = planning_status;
   }
   /**
            * Status of planning.
            */
   public byte getPlanningStatus()
   {
      return planning_status_;
   }


   public static Supplier<FootstepPlannerStatusMessagePubSubType> getPubSubType()
   {
      return FootstepPlannerStatusMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FootstepPlannerStatusMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FootstepPlannerStatusMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.planning_status_, other.planning_status_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FootstepPlannerStatusMessage)) return false;

      FootstepPlannerStatusMessage otherMyClass = (FootstepPlannerStatusMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.planning_status_ != otherMyClass.planning_status_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FootstepPlannerStatusMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("planning_status=");
      builder.append(this.planning_status_);
      builder.append("}");
      return builder.toString();
   }
}
