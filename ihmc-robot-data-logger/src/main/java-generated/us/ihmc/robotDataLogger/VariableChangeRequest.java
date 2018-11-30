package us.ihmc.robotDataLogger;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class VariableChangeRequest extends Packet<VariableChangeRequest> implements Settable<VariableChangeRequest>, EpsilonComparable<VariableChangeRequest>
{
   public int variableID_;
   public double requestedValue_;

   public VariableChangeRequest()
   {
   }

   public VariableChangeRequest(VariableChangeRequest other)
   {
      this();
      set(other);
   }

   public void set(VariableChangeRequest other)
   {
      variableID_ = other.variableID_;

      requestedValue_ = other.requestedValue_;

   }

   public void setVariableID(int variableID)
   {
      variableID_ = variableID;
   }
   public int getVariableID()
   {
      return variableID_;
   }

   public void setRequestedValue(double requestedValue)
   {
      requestedValue_ = requestedValue;
   }
   public double getRequestedValue()
   {
      return requestedValue_;
   }


   public static Supplier<VariableChangeRequestPubSubType> getPubSubType()
   {
      return VariableChangeRequestPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return VariableChangeRequestPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(VariableChangeRequest other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.variableID_, other.variableID_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.requestedValue_, other.requestedValue_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof VariableChangeRequest)) return false;

      VariableChangeRequest otherMyClass = (VariableChangeRequest) other;

      if(this.variableID_ != otherMyClass.variableID_) return false;

      if(this.requestedValue_ != otherMyClass.requestedValue_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("VariableChangeRequest {");
      builder.append("variableID=");
      builder.append(this.variableID_);      builder.append(", ");
      builder.append("requestedValue=");
      builder.append(this.requestedValue_);
      builder.append("}");
      return builder.toString();
   }
}
