package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class PilotInterfacePacket extends Packet<PilotInterfacePacket> implements Settable<PilotInterfacePacket>, EpsilonComparable<PilotInterfacePacket>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public int behaviour_state_;

   public int requested_behavior_state_;

   public int desired_step_type_;

   public int desired_step_length_type_;

   public int desired_step_stairs_type_;

   public boolean desired_step_continous_walk_;

   public int desired_steps_to_take_;

   public boolean execute_behavior_;

   public int desired_slope_step_type_;

   public int current_pilot_state_;

   public PilotInterfacePacket()
   {












   }

   public PilotInterfacePacket(PilotInterfacePacket other)
   {
      this();
      set(other);
   }

   public void set(PilotInterfacePacket other)
   {

      sequence_id_ = other.sequence_id_;


      behaviour_state_ = other.behaviour_state_;


      requested_behavior_state_ = other.requested_behavior_state_;


      desired_step_type_ = other.desired_step_type_;


      desired_step_length_type_ = other.desired_step_length_type_;


      desired_step_stairs_type_ = other.desired_step_stairs_type_;


      desired_step_continous_walk_ = other.desired_step_continous_walk_;


      desired_steps_to_take_ = other.desired_steps_to_take_;


      execute_behavior_ = other.execute_behavior_;


      desired_slope_step_type_ = other.desired_slope_step_type_;


      current_pilot_state_ = other.current_pilot_state_;

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


   public void setBehaviourState(int behaviour_state)
   {
      behaviour_state_ = behaviour_state;
   }
   public int getBehaviourState()
   {
      return behaviour_state_;
   }


   public void setRequestedBehaviorState(int requested_behavior_state)
   {
      requested_behavior_state_ = requested_behavior_state;
   }
   public int getRequestedBehaviorState()
   {
      return requested_behavior_state_;
   }


   public void setDesiredStepType(int desired_step_type)
   {
      desired_step_type_ = desired_step_type;
   }
   public int getDesiredStepType()
   {
      return desired_step_type_;
   }


   public void setDesiredStepLengthType(int desired_step_length_type)
   {
      desired_step_length_type_ = desired_step_length_type;
   }
   public int getDesiredStepLengthType()
   {
      return desired_step_length_type_;
   }


   public void setDesiredStepStairsType(int desired_step_stairs_type)
   {
      desired_step_stairs_type_ = desired_step_stairs_type;
   }
   public int getDesiredStepStairsType()
   {
      return desired_step_stairs_type_;
   }


   public void setDesiredStepContinousWalk(boolean desired_step_continous_walk)
   {
      desired_step_continous_walk_ = desired_step_continous_walk;
   }
   public boolean getDesiredStepContinousWalk()
   {
      return desired_step_continous_walk_;
   }


   public void setDesiredStepsToTake(int desired_steps_to_take)
   {
      desired_steps_to_take_ = desired_steps_to_take;
   }
   public int getDesiredStepsToTake()
   {
      return desired_steps_to_take_;
   }


   public void setExecuteBehavior(boolean execute_behavior)
   {
      execute_behavior_ = execute_behavior;
   }
   public boolean getExecuteBehavior()
   {
      return execute_behavior_;
   }


   public void setDesiredSlopeStepType(int desired_slope_step_type)
   {
      desired_slope_step_type_ = desired_slope_step_type;
   }
   public int getDesiredSlopeStepType()
   {
      return desired_slope_step_type_;
   }


   public void setCurrentPilotState(int current_pilot_state)
   {
      current_pilot_state_ = current_pilot_state;
   }
   public int getCurrentPilotState()
   {
      return current_pilot_state_;
   }


   public static Supplier<PilotInterfacePacketPubSubType> getPubSubType()
   {
      return PilotInterfacePacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return PilotInterfacePacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(PilotInterfacePacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.behaviour_state_, other.behaviour_state_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.requested_behavior_state_, other.requested_behavior_state_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.desired_step_type_, other.desired_step_type_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.desired_step_length_type_, other.desired_step_length_type_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.desired_step_stairs_type_, other.desired_step_stairs_type_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.desired_step_continous_walk_, other.desired_step_continous_walk_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.desired_steps_to_take_, other.desired_steps_to_take_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.execute_behavior_, other.execute_behavior_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.desired_slope_step_type_, other.desired_slope_step_type_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_pilot_state_, other.current_pilot_state_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof PilotInterfacePacket)) return false;

      PilotInterfacePacket otherMyClass = (PilotInterfacePacket) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.behaviour_state_ != otherMyClass.behaviour_state_) return false;


      if(this.requested_behavior_state_ != otherMyClass.requested_behavior_state_) return false;


      if(this.desired_step_type_ != otherMyClass.desired_step_type_) return false;


      if(this.desired_step_length_type_ != otherMyClass.desired_step_length_type_) return false;


      if(this.desired_step_stairs_type_ != otherMyClass.desired_step_stairs_type_) return false;


      if(this.desired_step_continous_walk_ != otherMyClass.desired_step_continous_walk_) return false;


      if(this.desired_steps_to_take_ != otherMyClass.desired_steps_to_take_) return false;


      if(this.execute_behavior_ != otherMyClass.execute_behavior_) return false;


      if(this.desired_slope_step_type_ != otherMyClass.desired_slope_step_type_) return false;


      if(this.current_pilot_state_ != otherMyClass.current_pilot_state_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("PilotInterfacePacket {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("behaviour_state=");
      builder.append(this.behaviour_state_);      builder.append(", ");

      builder.append("requested_behavior_state=");
      builder.append(this.requested_behavior_state_);      builder.append(", ");

      builder.append("desired_step_type=");
      builder.append(this.desired_step_type_);      builder.append(", ");

      builder.append("desired_step_length_type=");
      builder.append(this.desired_step_length_type_);      builder.append(", ");

      builder.append("desired_step_stairs_type=");
      builder.append(this.desired_step_stairs_type_);      builder.append(", ");

      builder.append("desired_step_continous_walk=");
      builder.append(this.desired_step_continous_walk_);      builder.append(", ");

      builder.append("desired_steps_to_take=");
      builder.append(this.desired_steps_to_take_);      builder.append(", ");

      builder.append("execute_behavior=");
      builder.append(this.execute_behavior_);      builder.append(", ");

      builder.append("desired_slope_step_type=");
      builder.append(this.desired_slope_step_type_);      builder.append(", ");

      builder.append("current_pilot_state=");
      builder.append(this.current_pilot_state_);
      builder.append("}");
      return builder.toString();
   }
}
