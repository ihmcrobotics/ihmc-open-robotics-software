package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Message used to configure the IHMC whole-body trajetory planner.
       * Main usage is the IHMC WholeBodyTrajectoryToolbox.
       */
public class WholeBodyTrajectoryToolboxConfigurationMessage extends Packet<WholeBodyTrajectoryToolboxConfigurationMessage> implements Settable<WholeBodyTrajectoryToolboxConfigurationMessage>, EpsilonComparable<WholeBodyTrajectoryToolboxConfigurationMessage>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public int number_of_initial_guesses_ = -1;

   public int maximum_expansion_size_ = -1;

   public controller_msgs.msg.dds.KinematicsToolboxOutputStatus initial_configuration_;

   public WholeBodyTrajectoryToolboxConfigurationMessage()
   {




      initial_configuration_ = new controller_msgs.msg.dds.KinematicsToolboxOutputStatus();

   }

   public WholeBodyTrajectoryToolboxConfigurationMessage(WholeBodyTrajectoryToolboxConfigurationMessage other)
   {
      this();
      set(other);
   }

   public void set(WholeBodyTrajectoryToolboxConfigurationMessage other)
   {

      sequence_id_ = other.sequence_id_;


      number_of_initial_guesses_ = other.number_of_initial_guesses_;


      maximum_expansion_size_ = other.maximum_expansion_size_;


      controller_msgs.msg.dds.KinematicsToolboxOutputStatusPubSubType.staticCopy(other.initial_configuration_, initial_configuration_);
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


   public void setNumberOfInitialGuesses(int number_of_initial_guesses)
   {
      number_of_initial_guesses_ = number_of_initial_guesses;
   }
   public int getNumberOfInitialGuesses()
   {
      return number_of_initial_guesses_;
   }


   public void setMaximumExpansionSize(int maximum_expansion_size)
   {
      maximum_expansion_size_ = maximum_expansion_size;
   }
   public int getMaximumExpansionSize()
   {
      return maximum_expansion_size_;
   }



   public controller_msgs.msg.dds.KinematicsToolboxOutputStatus getInitialConfiguration()
   {
      return initial_configuration_;
   }


   public static Supplier<WholeBodyTrajectoryToolboxConfigurationMessagePubSubType> getPubSubType()
   {
      return WholeBodyTrajectoryToolboxConfigurationMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return WholeBodyTrajectoryToolboxConfigurationMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(WholeBodyTrajectoryToolboxConfigurationMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.number_of_initial_guesses_, other.number_of_initial_guesses_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_expansion_size_, other.maximum_expansion_size_, epsilon)) return false;


      if (!this.initial_configuration_.epsilonEquals(other.initial_configuration_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof WholeBodyTrajectoryToolboxConfigurationMessage)) return false;

      WholeBodyTrajectoryToolboxConfigurationMessage otherMyClass = (WholeBodyTrajectoryToolboxConfigurationMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.number_of_initial_guesses_ != otherMyClass.number_of_initial_guesses_) return false;


      if(this.maximum_expansion_size_ != otherMyClass.maximum_expansion_size_) return false;


      if (!this.initial_configuration_.equals(otherMyClass.initial_configuration_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("WholeBodyTrajectoryToolboxConfigurationMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("number_of_initial_guesses=");
      builder.append(this.number_of_initial_guesses_);      builder.append(", ");

      builder.append("maximum_expansion_size=");
      builder.append(this.maximum_expansion_size_);      builder.append(", ");

      builder.append("initial_configuration=");
      builder.append(this.initial_configuration_);
      builder.append("}");
      return builder.toString();
   }
}
