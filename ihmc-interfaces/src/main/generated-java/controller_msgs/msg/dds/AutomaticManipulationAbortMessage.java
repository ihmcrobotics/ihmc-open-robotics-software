package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

/**
 * This message is part of the IHMC whole-body controller API. Request the controller to enable the
 * automatic manipulation abort feature. When enabled, any arm trajectory will get aborted as soon
 * as the balance controller has a large tracking error.
 */
public class AutomaticManipulationAbortMessage extends Packet<AutomaticManipulationAbortMessage>
      implements Settable<AutomaticManipulationAbortMessage>, EpsilonComparable<AutomaticManipulationAbortMessage>
{
   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header header_;
   public boolean enable_;

   public AutomaticManipulationAbortMessage()
   {
      header_ = new std_msgs.msg.dds.Header();
   }

   public AutomaticManipulationAbortMessage(AutomaticManipulationAbortMessage other)
   {
      this();
      set(other);
   }

   public void set(AutomaticManipulationAbortMessage other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      enable_ = other.enable_;

   }

   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }

   public void setEnable(boolean enable)
   {
      enable_ = enable;
   }

   public boolean getEnable()
   {
      return enable_;
   }

   @Override
   public boolean epsilonEquals(AutomaticManipulationAbortMessage other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon))
         return false;
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

      if (!this.header_.equals(otherMyClass.header_))
         return false;
      if (this.enable_ != otherMyClass.enable_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AutomaticManipulationAbortMessage {");
      builder.append("header=");
      builder.append(this.header_);
      builder.append(", ");
      builder.append("enable=");
      builder.append(this.enable_);
      builder.append("}");
      return builder.toString();
   }
}
