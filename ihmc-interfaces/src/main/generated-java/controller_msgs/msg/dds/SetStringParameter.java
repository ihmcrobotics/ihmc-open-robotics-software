package controller_msgs.msg.dds;

/**
 * Definition of the class "SetStringParameter" defined in SetStringParameter_.idl.
 *
 * This file was automatically generated from SetStringParameter_.idl by us.ihmc.idl.generator.IDLGenerator.
 * Do not update this file directly, edit SetStringParameter_.idl instead.
 */
public class SetStringParameter
{
   private java.lang.StringBuilder parameter_name_;
   private java.lang.StringBuilder parameter_value_;

   public SetStringParameter()
   {
      parameter_name_ = new java.lang.StringBuilder(255);
      parameter_value_ = new java.lang.StringBuilder(255);
   }

   public void set(SetStringParameter other)
   {
      parameter_name_.setLength(0);
      parameter_name_.append(other.parameter_name_);
      parameter_value_.setLength(0);
      parameter_value_.append(other.parameter_value_);
   }

   public java.lang.String getParameter_nameAsString()
   {
      return getParameter_name().toString();
   }

   public java.lang.StringBuilder getParameter_name()
   {
      return parameter_name_;
   }

   public void setParameter_name(String parameter_name)
   {
      parameter_name_.setLength(0);
      parameter_name_.append(parameter_name);
   }

   public java.lang.String getParameter_valueAsString()
   {
      return getParameter_value().toString();
   }

   public java.lang.StringBuilder getParameter_value()
   {
      return parameter_value_;
   }

   public void setParameter_value(String parameter_value)
   {
      parameter_value_.setLength(0);
      parameter_value_.append(parameter_value);
   }

   @Override
   public boolean equals(java.lang.Object other)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;
      if (!(other instanceof SetStringParameter))
         return false;
      SetStringParameter otherMyClass = (SetStringParameter) other;
      boolean returnedValue = true;

      returnedValue &= us.ihmc.idl.IDLTools.equals(this.parameter_name_, otherMyClass.parameter_name_);

      returnedValue &= us.ihmc.idl.IDLTools.equals(this.parameter_value_, otherMyClass.parameter_value_);

      return returnedValue;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("SetStringParameter {");
      builder.append("parameter_name=");
      builder.append(this.parameter_name_);

      builder.append(", ");
      builder.append("parameter_value=");
      builder.append(this.parameter_value_);

      builder.append("}");
      return builder.toString();
   }
}