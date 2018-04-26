package us.ihmc.robotDataLogger;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class JointDefinition extends Packet<JointDefinition> implements Settable<JointDefinition>, EpsilonComparable<JointDefinition>
{
   public java.lang.StringBuilder name_;
   public us.ihmc.robotDataLogger.JointType type_;

   public JointDefinition()
   {
      name_ = new java.lang.StringBuilder(255);
   }

   public JointDefinition(JointDefinition other)
   {
      this();
      set(other);
   }

   public void set(JointDefinition other)
   {
      name_.setLength(0);
      name_.append(other.name_);

      type_ = other.type_;

   }

   public void setName(java.lang.String name)
   {
      name_.setLength(0);
      name_.append(name);
   }

   public java.lang.String getNameAsString()
   {
      return getName().toString();
   }
   public java.lang.StringBuilder getName()
   {
      return name_;
   }

   public void setType(us.ihmc.robotDataLogger.JointType type)
   {
      type_ = type;
   }
   public us.ihmc.robotDataLogger.JointType getType()
   {
      return type_;
   }


   public static Supplier<JointDefinitionPubSubType> getPubSubType()
   {
      return JointDefinitionPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return JointDefinitionPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(JointDefinition other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.name_, other.name_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsEnum(this.type_, other.type_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof JointDefinition)) return false;

      JointDefinition otherMyClass = (JointDefinition) other;

      if (!us.ihmc.idl.IDLTools.equals(this.name_, otherMyClass.name_)) return false;

      if(this.type_ != otherMyClass.type_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("JointDefinition {");
      builder.append("name=");
      builder.append(this.name_);      builder.append(", ");
      builder.append("type=");
      builder.append(this.type_);
      builder.append("}");
      return builder.toString();
   }
}
