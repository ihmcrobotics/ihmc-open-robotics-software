package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * This message is part of the IHMC whole-body controller API.
 * Configure the controller manipulation and pelvis managers.
 */
public class PrepareForLocomotionMessage extends Packet<PrepareForLocomotionMessage>
      implements Settable<PrepareForLocomotionMessage>, EpsilonComparable<PrepareForLocomotionMessage>
{
   /**
    * When true, the controller will cancel any arm trajectory in progress, if any, before starting to walk.
    */
   public boolean prepare_manipulation_ = true;
   /**
    * When true, the controller will cancel any pelvis trajectory in progress, if any, before starting to walk.
    */
   public boolean prepare_pelvis_ = true;

   public PrepareForLocomotionMessage()
   {

   }

   public PrepareForLocomotionMessage(PrepareForLocomotionMessage other)
   {
      set(other);
   }

   public void set(PrepareForLocomotionMessage other)
   {
      prepare_manipulation_ = other.prepare_manipulation_;

      prepare_pelvis_ = other.prepare_pelvis_;
   }

   /**
    * When true, the controller will cancel any arm trajectory in progress, if any, before starting to walk.
    */
   public boolean getPrepareManipulation()
   {
      return prepare_manipulation_;
   }

   /**
    * When true, the controller will cancel any arm trajectory in progress, if any, before starting to walk.
    */
   public void setPrepareManipulation(boolean prepare_manipulation)
   {
      prepare_manipulation_ = prepare_manipulation;
   }

   /**
    * When true, the controller will cancel any pelvis trajectory in progress, if any, before starting to walk.
    */
   public boolean getPreparePelvis()
   {
      return prepare_pelvis_;
   }

   /**
    * When true, the controller will cancel any pelvis trajectory in progress, if any, before starting to walk.
    */
   public void setPreparePelvis(boolean prepare_pelvis)
   {
      prepare_pelvis_ = prepare_pelvis;
   }

   @Override
   public boolean epsilonEquals(PrepareForLocomotionMessage other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.prepare_manipulation_, other.prepare_manipulation_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.prepare_pelvis_, other.prepare_pelvis_, epsilon))
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
      if (!(other instanceof PrepareForLocomotionMessage))
         return false;

      PrepareForLocomotionMessage otherMyClass = (PrepareForLocomotionMessage) other;

      if (this.prepare_manipulation_ != otherMyClass.prepare_manipulation_)
         return false;

      if (this.prepare_pelvis_ != otherMyClass.prepare_pelvis_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("PrepareForLocomotionMessage {");
      builder.append("prepare_manipulation=");
      builder.append(this.prepare_manipulation_);

      builder.append(", ");
      builder.append("prepare_pelvis=");
      builder.append(this.prepare_pelvis_);

      builder.append("}");
      return builder.toString();
   }
}
