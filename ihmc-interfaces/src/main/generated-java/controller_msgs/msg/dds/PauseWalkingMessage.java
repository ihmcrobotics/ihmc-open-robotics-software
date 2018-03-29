package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

/**
 * This message is part of the IHMC whole-body controller API. This message pauses the execution of
 * a list of footsteps. If this message is in the middle of executing a footstep, the robot will
 * finish the step and pause when back in double support.
 */
public class PauseWalkingMessage extends Packet<PauseWalkingMessage> implements Settable<PauseWalkingMessage>, EpsilonComparable<PauseWalkingMessage>
{
   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header header_;
   /**
    * True to pause walking, false to unpause and resume an existing footstep plan.
    */
   public boolean pause_;

   public PauseWalkingMessage()
   {
      header_ = new std_msgs.msg.dds.Header();
   }

   public PauseWalkingMessage(PauseWalkingMessage other)
   {
      this();
      set(other);
   }

   public void set(PauseWalkingMessage other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      pause_ = other.pause_;

   }

   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }

   /**
    * True to pause walking, false to unpause and resume an existing footstep plan.
    */
   public void setPause(boolean pause)
   {
      pause_ = pause;
   }

   /**
    * True to pause walking, false to unpause and resume an existing footstep plan.
    */
   public boolean getPause()
   {
      return pause_;
   }

   @Override
   public boolean epsilonEquals(PauseWalkingMessage other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon))
         return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.pause_, other.pause_, epsilon))
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
      if (!(other instanceof PauseWalkingMessage))
         return false;

      PauseWalkingMessage otherMyClass = (PauseWalkingMessage) other;

      if (!this.header_.equals(otherMyClass.header_))
         return false;
      if (this.pause_ != otherMyClass.pause_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("PauseWalkingMessage {");
      builder.append("header=");
      builder.append(this.header_);
      builder.append(", ");
      builder.append("pause=");
      builder.append(this.pause_);
      builder.append("}");
      return builder.toString();
   }
}
