package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC footstep planning module.
       */
public class FootstepPlannerActionMessage extends Packet<FootstepPlannerActionMessage> implements Settable<FootstepPlannerActionMessage>, EpsilonComparable<FootstepPlannerActionMessage>
{

   /**
          * Halts the footstep planner
          */
   public static final byte FOOTSTEP_PLANNER_REQUESTED_ACTION_HALT = (byte) 0;

   /**
          * Planner will publish it's current parameters
          */
   public static final byte FOOTSTEP_PLANNER_REQUESTED_ACTION_PUBLISH_PARAMETERS = (byte) 1;

   public byte requested_action_ = (byte) 255;

   public FootstepPlannerActionMessage()
   {


   }

   public FootstepPlannerActionMessage(FootstepPlannerActionMessage other)
   {
      this();
      set(other);
   }

   public void set(FootstepPlannerActionMessage other)
   {

      requested_action_ = other.requested_action_;

   }


   public void setRequestedAction(byte requested_action)
   {
      requested_action_ = requested_action;
   }
   public byte getRequestedAction()
   {
      return requested_action_;
   }


   public static Supplier<FootstepPlannerActionMessagePubSubType> getPubSubType()
   {
      return FootstepPlannerActionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FootstepPlannerActionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FootstepPlannerActionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.requested_action_, other.requested_action_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FootstepPlannerActionMessage)) return false;

      FootstepPlannerActionMessage otherMyClass = (FootstepPlannerActionMessage) other;


      if(this.requested_action_ != otherMyClass.requested_action_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FootstepPlannerActionMessage {");

      builder.append("requested_action=");
      builder.append(this.requested_action_);
      builder.append("}");
      return builder.toString();
   }
}
