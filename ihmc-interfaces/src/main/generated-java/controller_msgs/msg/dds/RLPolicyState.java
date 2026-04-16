package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message represents the current policy and available policies for RL
       */
public class RLPolicyState extends Packet<RLPolicyState> implements Settable<RLPolicyState>, EpsilonComparable<RLPolicyState>
{
   /**
            * Current model being used, wrt array of available models
            */
   public byte current_model_;
   /**
            * Array of available models to choose from, with the number of models in the array
            */
   public byte num_available_models_;
   public us.ihmc.idl.IDLSequence.StringBuilderHolder  available_models_;

   public RLPolicyState()
   {
      available_models_ = new us.ihmc.idl.IDLSequence.StringBuilderHolder (16, "type_d");
   }

   public RLPolicyState(RLPolicyState other)
   {
      this();
      set(other);
   }

   public void set(RLPolicyState other)
   {
      current_model_ = other.current_model_;

      num_available_models_ = other.num_available_models_;

      available_models_.set(other.available_models_);
   }

   /**
            * Current model being used, wrt array of available models
            */
   public void setCurrentModel(byte current_model)
   {
      current_model_ = current_model;
   }
   /**
            * Current model being used, wrt array of available models
            */
   public byte getCurrentModel()
   {
      return current_model_;
   }

   /**
            * Array of available models to choose from, with the number of models in the array
            */
   public void setNumAvailableModels(byte num_available_models)
   {
      num_available_models_ = num_available_models;
   }
   /**
            * Array of available models to choose from, with the number of models in the array
            */
   public byte getNumAvailableModels()
   {
      return num_available_models_;
   }


   public us.ihmc.idl.IDLSequence.StringBuilderHolder  getAvailableModels()
   {
      return available_models_;
   }


   public static Supplier<RLPolicyStatePubSubType> getPubSubType()
   {
      return RLPolicyStatePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return RLPolicyStatePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(RLPolicyState other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_model_, other.current_model_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.num_available_models_, other.num_available_models_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilderSequence(this.available_models_, other.available_models_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof RLPolicyState)) return false;

      RLPolicyState otherMyClass = (RLPolicyState) other;

      if(this.current_model_ != otherMyClass.current_model_) return false;

      if(this.num_available_models_ != otherMyClass.num_available_models_) return false;

      if (!this.available_models_.equals(otherMyClass.available_models_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("RLPolicyState {");
      builder.append("current_model=");
      builder.append(this.current_model_);      builder.append(", ");
      builder.append("num_available_models=");
      builder.append(this.num_available_models_);      builder.append(", ");
      builder.append("available_models=");
      builder.append(this.available_models_);
      builder.append("}");
      return builder.toString();
   }
}
