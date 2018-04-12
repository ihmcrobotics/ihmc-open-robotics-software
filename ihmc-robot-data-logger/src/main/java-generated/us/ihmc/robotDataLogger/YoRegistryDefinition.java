package us.ihmc.robotDataLogger;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

public class YoRegistryDefinition extends Packet<YoRegistryDefinition> implements Settable<YoRegistryDefinition>, EpsilonComparable<YoRegistryDefinition>
{
   public int parent_;
   public java.lang.StringBuilder name_;

   public YoRegistryDefinition()
   {
      name_ = new java.lang.StringBuilder(255);
   }

   public YoRegistryDefinition(YoRegistryDefinition other)
   {
      this();
      set(other);
   }

   public void set(YoRegistryDefinition other)
   {
      parent_ = other.parent_;

      name_.setLength(0);
      name_.append(other.name_);

   }

   public void setParent(int parent)
   {
      parent_ = parent;
   }
   public int getParent()
   {
      return parent_;
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


   @Override
   public boolean epsilonEquals(YoRegistryDefinition other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.parent_, other.parent_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.name_, other.name_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof YoRegistryDefinition)) return false;

      YoRegistryDefinition otherMyClass = (YoRegistryDefinition) other;

      if(this.parent_ != otherMyClass.parent_) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.name_, otherMyClass.name_)) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("YoRegistryDefinition {");
      builder.append("parent=");
      builder.append(this.parent_);      builder.append(", ");
      builder.append("name=");
      builder.append(this.name_);
      builder.append("}");
      return builder.toString();
   }
}
