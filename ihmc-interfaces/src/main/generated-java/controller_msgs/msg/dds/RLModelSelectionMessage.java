package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
      * This message is used to select which RL model the controller should run
      */
public class RLModelSelectionMessage extends Packet<RLModelSelectionMessage> implements Settable<RLModelSelectionMessage>, EpsilonComparable<RLModelSelectionMessage>
{
   /**
            * Desired model to be used, wrt array position
            */
   public byte desired_model_;
   public boolean execute_desired_model_;

   public RLModelSelectionMessage()
   {
   }

   public RLModelSelectionMessage(RLModelSelectionMessage other)
   {
      this();
      set(other);
   }

   public void set(RLModelSelectionMessage other)
   {
      desired_model_ = other.desired_model_;

      execute_desired_model_ = other.execute_desired_model_;

   }

   /**
            * Desired model to be used, wrt array position
            */
   public void setDesiredModel(byte desired_model)
   {
      desired_model_ = desired_model;
   }
   /**
            * Desired model to be used, wrt array position
            */
   public byte getDesiredModel()
   {
      return desired_model_;
   }

   public void setExecuteDesiredModel(boolean execute_desired_model)
   {
      execute_desired_model_ = execute_desired_model;
   }
   public boolean getExecuteDesiredModel()
   {
      return execute_desired_model_;
   }


   public static Supplier<RLModelSelectionMessagePubSubType> getPubSubType()
   {
      return RLModelSelectionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return RLModelSelectionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(RLModelSelectionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.desired_model_, other.desired_model_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.execute_desired_model_, other.execute_desired_model_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof RLModelSelectionMessage)) return false;

      RLModelSelectionMessage otherMyClass = (RLModelSelectionMessage) other;

      if(this.desired_model_ != otherMyClass.desired_model_) return false;

      if(this.execute_desired_model_ != otherMyClass.execute_desired_model_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("RLModelSelectionMessage {");
      builder.append("desired_model=");
      builder.append(this.desired_model_);      builder.append(", ");
      builder.append("execute_desired_model=");
      builder.append(this.execute_desired_model_);
      builder.append("}");
      return builder.toString();
   }
}
