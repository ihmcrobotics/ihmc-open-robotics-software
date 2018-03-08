package controller_msgs.msg.dds;

import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * This message is part of the IHMC whole-body controller API.
 * This message pauses the execution of a list of footsteps.
 * If this message is in the middle of executing a footstep, the robot will finish the step and pause when back in double support.
 */
public class PauseWalkingMessage implements Settable<PauseWalkingMessage>, EpsilonComparable<PauseWalkingMessage>
{
   /**
    * True to pause walking, false to unpause and resume an existing footstep plan.
    */
   private boolean pause_;

   public PauseWalkingMessage()
   {
   }

   public PauseWalkingMessage(PauseWalkingMessage other)
   {
      set(other);
   }

   public void set(PauseWalkingMessage other)
   {
      pause_ = other.pause_;
   }

   /**
    * True to pause walking, false to unpause and resume an existing footstep plan.
    */
   public boolean getPause()
   {
      return pause_;
   }

   /**
    * True to pause walking, false to unpause and resume an existing footstep plan.
    */
   public void setPause(boolean pause)
   {
      pause_ = pause;
   }

   @Override
   public boolean epsilonEquals(PauseWalkingMessage other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

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

      if (this.pause_ != otherMyClass.pause_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("PauseWalkingMessage {");
      builder.append("pause=");
      builder.append(this.pause_);

      builder.append("}");
      return builder.toString();
   }
}