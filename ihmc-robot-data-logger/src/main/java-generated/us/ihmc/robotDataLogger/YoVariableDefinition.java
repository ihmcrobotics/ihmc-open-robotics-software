package us.ihmc.robotDataLogger;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

public class YoVariableDefinition extends Packet<YoVariableDefinition> implements Settable<YoVariableDefinition>, EpsilonComparable<YoVariableDefinition>
{
   public java.lang.StringBuilder name_;
   public java.lang.StringBuilder description_;
   public us.ihmc.robotDataLogger.YoType type_;
   public int registry_;
   public int enumType_;
   public boolean allowNullValues_;
   public boolean isParameter_;
   public double min_;
   public double max_;
   public us.ihmc.robotDataLogger.LoadStatus loadStatus_;

   public YoVariableDefinition()
   {
      name_ = new java.lang.StringBuilder(255);

      description_ = new java.lang.StringBuilder(255);
   }

   public YoVariableDefinition(YoVariableDefinition other)
   {
      set(other);
   }

   public void set(YoVariableDefinition other)
   {
      name_.setLength(0);
      name_.append(other.name_);

      description_.setLength(0);
      description_.append(other.description_);

      type_ = other.type_;

      registry_ = other.registry_;

      enumType_ = other.enumType_;

      allowNullValues_ = other.allowNullValues_;

      isParameter_ = other.isParameter_;

      min_ = other.min_;

      max_ = other.max_;

      loadStatus_ = other.loadStatus_;
   }

   public java.lang.String getNameAsString()
   {
      return getName().toString();
   }

   public java.lang.StringBuilder getName()
   {
      return name_;
   }

   public void setName(java.lang.String name)
   {
      name_.setLength(0);
      name_.append(name);
   }

   public java.lang.String getDescriptionAsString()
   {
      return getDescription().toString();
   }

   public java.lang.StringBuilder getDescription()
   {
      return description_;
   }

   public void setDescription(java.lang.String description)
   {
      description_.setLength(0);
      description_.append(description);
   }

   public us.ihmc.robotDataLogger.YoType getType()
   {
      return type_;
   }

   public void setType(us.ihmc.robotDataLogger.YoType type)
   {
      type_ = type;
   }

   public int getRegistry()
   {
      return registry_;
   }

   public void setRegistry(int registry)
   {
      registry_ = registry;
   }

   public int getEnumType()
   {
      return enumType_;
   }

   public void setEnumType(int enumType)
   {
      enumType_ = enumType;
   }

   public boolean getAllowNullValues()
   {
      return allowNullValues_;
   }

   public void setAllowNullValues(boolean allowNullValues)
   {
      allowNullValues_ = allowNullValues;
   }

   public boolean getIsParameter()
   {
      return isParameter_;
   }

   public void setIsParameter(boolean isParameter)
   {
      isParameter_ = isParameter;
   }

   public double getMin()
   {
      return min_;
   }

   public void setMin(double min)
   {
      min_ = min;
   }

   public double getMax()
   {
      return max_;
   }

   public void setMax(double max)
   {
      max_ = max;
   }

   public us.ihmc.robotDataLogger.LoadStatus getLoadStatus()
   {
      return loadStatus_;
   }

   public void setLoadStatus(us.ihmc.robotDataLogger.LoadStatus loadStatus)
   {
      loadStatus_ = loadStatus;
   }

   @Override
   public boolean epsilonEquals(YoVariableDefinition other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.name_, other.name_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.description_, other.description_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsEnum(this.type_, other.type_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.registry_, other.registry_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.enumType_, other.enumType_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.allowNullValues_, other.allowNullValues_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.isParameter_, other.isParameter_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.min_, other.min_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.max_, other.max_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsEnum(this.loadStatus_, other.loadStatus_, epsilon))
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
      if (!(other instanceof YoVariableDefinition))
         return false;

      YoVariableDefinition otherMyClass = (YoVariableDefinition) other;

      if (!us.ihmc.idl.IDLTools.equals(this.name_, otherMyClass.name_))
         return false;

      if (!us.ihmc.idl.IDLTools.equals(this.description_, otherMyClass.description_))
         return false;

      if (this.type_ != otherMyClass.type_)
         return false;

      if (this.registry_ != otherMyClass.registry_)
         return false;

      if (this.enumType_ != otherMyClass.enumType_)
         return false;

      if (this.allowNullValues_ != otherMyClass.allowNullValues_)
         return false;

      if (this.isParameter_ != otherMyClass.isParameter_)
         return false;

      if (this.min_ != otherMyClass.min_)
         return false;

      if (this.max_ != otherMyClass.max_)
         return false;

      if (this.loadStatus_ != otherMyClass.loadStatus_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("YoVariableDefinition {");
      builder.append("name=");
      builder.append(this.name_);

      builder.append(", ");
      builder.append("description=");
      builder.append(this.description_);

      builder.append(", ");
      builder.append("type=");
      builder.append(this.type_);

      builder.append(", ");
      builder.append("registry=");
      builder.append(this.registry_);

      builder.append(", ");
      builder.append("enumType=");
      builder.append(this.enumType_);

      builder.append(", ");
      builder.append("allowNullValues=");
      builder.append(this.allowNullValues_);

      builder.append(", ");
      builder.append("isParameter=");
      builder.append(this.isParameter_);

      builder.append(", ");
      builder.append("min=");
      builder.append(this.min_);

      builder.append(", ");
      builder.append("max=");
      builder.append(this.max_);

      builder.append(", ");
      builder.append("loadStatus=");
      builder.append(this.loadStatus_);

      builder.append("}");
      return builder.toString();
   }
}