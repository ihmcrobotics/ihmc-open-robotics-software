package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

public class PilotInterfaceActionPacket extends Packet<PilotInterfaceActionPacket>
      implements Settable<PilotInterfaceActionPacket>, EpsilonComparable<PilotInterfaceActionPacket>
{
   public byte pilot_action_;

   public PilotInterfaceActionPacket()
   {
   }

   public PilotInterfaceActionPacket(PilotInterfaceActionPacket other)
   {
      set(other);
   }

   public void set(PilotInterfaceActionPacket other)
   {
      pilot_action_ = other.pilot_action_;

   }

   public void setPilotAction(byte pilot_action)
   {
      pilot_action_ = pilot_action;
   }

   public byte getPilotAction()
   {
      return pilot_action_;
   }

   @Override
   public boolean epsilonEquals(PilotInterfaceActionPacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.pilot_action_, other.pilot_action_, epsilon))
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
      if (!(other instanceof PilotInterfaceActionPacket))
         return false;

      PilotInterfaceActionPacket otherMyClass = (PilotInterfaceActionPacket) other;

      if (this.pilot_action_ != otherMyClass.pilot_action_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("PilotInterfaceActionPacket {");
      builder.append("pilot_action=");
      builder.append(this.pilot_action_);
      builder.append("}");
      return builder.toString();
   }
}
