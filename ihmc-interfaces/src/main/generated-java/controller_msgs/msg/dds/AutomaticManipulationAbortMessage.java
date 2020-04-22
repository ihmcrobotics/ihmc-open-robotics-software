package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * Request the controller to enable the automatic manipulation abort feature.
       * When enabled, any arm trajectory will get aborted as soon as the balance controller has a large tracking error.
       */
public class AutomaticManipulationAbortMessage extends Packet<AutomaticManipulationAbortMessage> implements Settable<AutomaticManipulationAbortMessage>, EpsilonComparable<AutomaticManipulationAbortMessage>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public boolean enable_;

   public AutomaticManipulationAbortMessage()
   {



   }

   public AutomaticManipulationAbortMessage(AutomaticManipulationAbortMessage other)
   {
      this();
      set(other);
   }

   public void set(AutomaticManipulationAbortMessage other)
   {

      sequence_id_ = other.sequence_id_;


      enable_ = other.enable_;

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


   public void setEnable(boolean enable)
   {
      enable_ = enable;
   }
   public boolean getEnable()
   {
      return enable_;
   }


   public static Supplier<AutomaticManipulationAbortMessagePubSubType> getPubSubType()
   {
      return AutomaticManipulationAbortMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return AutomaticManipulationAbortMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(AutomaticManipulationAbortMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.enable_, other.enable_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof AutomaticManipulationAbortMessage)) return false;

      AutomaticManipulationAbortMessage otherMyClass = (AutomaticManipulationAbortMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.enable_ != otherMyClass.enable_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AutomaticManipulationAbortMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("enable=");
      builder.append(this.enable_);
      builder.append("}");
      return builder.toString();
   }
}
