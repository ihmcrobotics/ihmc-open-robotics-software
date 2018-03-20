package controller_msgs.msg.dds;

import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

public class SetDoubleParameter implements Settable<SetDoubleParameter>, EpsilonComparable<SetDoubleParameter>
{
   private java.lang.StringBuilder parameter_name_;
   private double parameter_value_;

   public SetDoubleParameter()
   {
      parameter_name_ = new java.lang.StringBuilder(255);
   }

   public SetDoubleParameter(SetDoubleParameter other)
   {
      set(other);
   }

   public void set(SetDoubleParameter other)
   {
      parameter_name_.setLength(0);
      parameter_name_.append(other.parameter_name_);

      parameter_value_ = other.parameter_value_;
   }

   public java.lang.String getParameterNameAsString()
   {
      return getParameterName().toString();
   }

   public java.lang.StringBuilder getParameterName()
   {
      return parameter_name_;
   }

   public void setParameterName(java.lang.String parameter_name)
   {
      parameter_name_.setLength(0);
      parameter_name_.append(parameter_name);
   }

   public double getParameterValue()
   {
      return parameter_value_;
   }

   public void setParameterValue(double parameter_value)
   {
      parameter_value_ = parameter_value;
   }

   @Override
   public boolean epsilonEquals(SetDoubleParameter other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.parameter_name_, other.parameter_name_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.parameter_value_, other.parameter_value_, epsilon))
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
      if (!(other instanceof SetDoubleParameter))
         return false;

      SetDoubleParameter otherMyClass = (SetDoubleParameter) other;

      if (!us.ihmc.idl.IDLTools.equals(this.parameter_name_, otherMyClass.parameter_name_))
         return false;

      if (this.parameter_value_ != otherMyClass.parameter_value_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("SetDoubleParameter {");
      builder.append("parameter_name=");
      builder.append(this.parameter_name_);

      builder.append(", ");
      builder.append("parameter_value=");
      builder.append(this.parameter_value_);

      builder.append("}");
      return builder.toString();
   }
}