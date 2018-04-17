package us.ihmc.robotDataLogger;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

public class ClearLogRequest extends Packet<ClearLogRequest> implements Settable<ClearLogRequest>, EpsilonComparable<ClearLogRequest>
{
   public java.lang.StringBuilder guid_;

   public ClearLogRequest()
   {
      guid_ = new java.lang.StringBuilder(255);
   }

   public ClearLogRequest(ClearLogRequest other)
   {
      this();
      set(other);
   }

   public void set(ClearLogRequest other)
   {
      guid_.setLength(0);
      guid_.append(other.guid_);
   }

   public void setGuid(java.lang.String guid)
   {
      guid_.setLength(0);
      guid_.append(guid);
   }

   public java.lang.String getGuidAsString()
   {
      return getGuid().toString();
   }
   public java.lang.StringBuilder getGuid()
   {
      return guid_;
   }


   @Override
   public boolean epsilonEquals(ClearLogRequest other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.guid_, other.guid_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ClearLogRequest)) return false;

      ClearLogRequest otherMyClass = (ClearLogRequest) other;

      if (!us.ihmc.idl.IDLTools.equals(this.guid_, otherMyClass.guid_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ClearLogRequest {");
      builder.append("guid=");
      builder.append(this.guid_);
      builder.append("}");
      return builder.toString();
   }
}
