package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * This message is part of the IHMC whole-body controller API.
 * Request the controller to enable the automatic manipulation abort feature.
 * When enabled, any arm trajectory will get aborted as soon as the balance controller has a large tracking error.
 */
public class AutomaticManipulationAbortMessage extends Packet<AutomaticManipulationAbortMessage>
      implements Settable<AutomaticManipulationAbortMessage>, EpsilonComparable<AutomaticManipulationAbortMessage>
{
   public boolean enable_;

   public AutomaticManipulationAbortMessage()
   {
   }

   public AutomaticManipulationAbortMessage(AutomaticManipulationAbortMessage other)
   {
      set(other);
   }

   public void set(AutomaticManipulationAbortMessage other)
   {
      enable_ = other.enable_;
   }

   public boolean getEnable()
   {
      return enable_;
   }

   public void setEnable(boolean enable)
   {
      enable_ = enable;
   }

   @Override
   public boolean epsilonEquals(AutomaticManipulationAbortMessage other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.enable_, other.enable_, epsilon))
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
      if (!(other instanceof AutomaticManipulationAbortMessage))
         return false;

      AutomaticManipulationAbortMessage otherMyClass = (AutomaticManipulationAbortMessage) other;

      if (this.enable_ != otherMyClass.enable_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AutomaticManipulationAbortMessage {");
      builder.append("enable=");
      builder.append(this.enable_);

      builder.append("}");
      return builder.toString();
   }
}
