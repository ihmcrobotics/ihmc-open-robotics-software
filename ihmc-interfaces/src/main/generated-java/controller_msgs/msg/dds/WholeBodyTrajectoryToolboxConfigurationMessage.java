package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * Message used to configure the IHMC whole-body trajetory planner.
 * Main usage is the IHMC WholeBodyTrajectoryToolbox.
 */
public class WholeBodyTrajectoryToolboxConfigurationMessage extends Packet<WholeBodyTrajectoryToolboxConfigurationMessage>
      implements Settable<WholeBodyTrajectoryToolboxConfigurationMessage>, EpsilonComparable<WholeBodyTrajectoryToolboxConfigurationMessage>
{
   public int number_of_initial_guesses_ = -1;
   public int maximum_expansion_size_ = -1;
   public controller_msgs.msg.dds.KinematicsToolboxOutputStatus initial_configuration_;

   public WholeBodyTrajectoryToolboxConfigurationMessage()
   {

      initial_configuration_ = new controller_msgs.msg.dds.KinematicsToolboxOutputStatus();
   }

   public WholeBodyTrajectoryToolboxConfigurationMessage(WholeBodyTrajectoryToolboxConfigurationMessage other)
   {
      set(other);
   }

   public void set(WholeBodyTrajectoryToolboxConfigurationMessage other)
   {
      number_of_initial_guesses_ = other.number_of_initial_guesses_;

      maximum_expansion_size_ = other.maximum_expansion_size_;

      controller_msgs.msg.dds.KinematicsToolboxOutputStatusPubSubType.staticCopy(other.initial_configuration_, initial_configuration_);
   }

   public int getNumberOfInitialGuesses()
   {
      return number_of_initial_guesses_;
   }

   public void setNumberOfInitialGuesses(int number_of_initial_guesses)
   {
      number_of_initial_guesses_ = number_of_initial_guesses;
   }

   public int getMaximumExpansionSize()
   {
      return maximum_expansion_size_;
   }

   public void setMaximumExpansionSize(int maximum_expansion_size)
   {
      maximum_expansion_size_ = maximum_expansion_size;
   }

   public controller_msgs.msg.dds.KinematicsToolboxOutputStatus getInitialConfiguration()
   {
      return initial_configuration_;
   }

   @Override
   public boolean epsilonEquals(WholeBodyTrajectoryToolboxConfigurationMessage other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.number_of_initial_guesses_, other.number_of_initial_guesses_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_expansion_size_, other.maximum_expansion_size_, epsilon))
         return false;

      if (!this.initial_configuration_.epsilonEquals(other.initial_configuration_, epsilon))
         return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;
      if (!(other instanceof WholeBodyTrajectoryToolboxConfigurationMessage))
         return false;

      WholeBodyTrajectoryToolboxConfigurationMessage otherMyClass = (WholeBodyTrajectoryToolboxConfigurationMessage) other;

      if (this.number_of_initial_guesses_ != otherMyClass.number_of_initial_guesses_)
         return false;

      if (this.maximum_expansion_size_ != otherMyClass.maximum_expansion_size_)
         return false;

      if (!this.initial_configuration_.equals(otherMyClass.initial_configuration_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("WholeBodyTrajectoryToolboxConfigurationMessage {");
      builder.append("number_of_initial_guesses=");
      builder.append(this.number_of_initial_guesses_);

      builder.append(", ");
      builder.append("maximum_expansion_size=");
      builder.append(this.maximum_expansion_size_);

      builder.append(", ");
      builder.append("initial_configuration=");
      builder.append(this.initial_configuration_);

      builder.append("}");
      return builder.toString();
   }
}