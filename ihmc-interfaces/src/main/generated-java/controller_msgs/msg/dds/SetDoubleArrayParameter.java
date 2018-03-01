package controller_msgs.msg.dds;

import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * Definition of the class "SetDoubleArrayParameter" defined in SetDoubleArrayParameter_.idl.
 *
 * This file was automatically generated from SetDoubleArrayParameter_.idl by us.ihmc.idl.generator.IDLGenerator.
 * Do not update this file directly, edit SetDoubleArrayParameter_.idl instead.
 */
public class SetDoubleArrayParameter implements Settable<SetDoubleArrayParameter>, EpsilonComparable<SetDoubleArrayParameter>
{
   private java.lang.StringBuilder parameter_name_;
   private us.ihmc.idl.IDLSequence.Double parameter_value_;

   public SetDoubleArrayParameter()
   {
      parameter_name_ = new java.lang.StringBuilder(255);

      parameter_value_ = new us.ihmc.idl.IDLSequence.Double(100, "type_6");
   }

   public SetDoubleArrayParameter(SetDoubleArrayParameter other)
   {
      set(other);
   }

   public void set(SetDoubleArrayParameter other)
   {
      parameter_name_.setLength(0);
      parameter_name_.append(other.parameter_name_);

      parameter_value_.set(other.parameter_value_);
   }

   public java.lang.String getParameterNameAsString()
   {
      return getParameterName().toString();
   }

   public java.lang.StringBuilder getParameterName()
   {
      return parameter_name_;
   }

   public void setParameterName(String parameter_name)
   {
      parameter_name_.setLength(0);
      parameter_name_.append(parameter_name);
   }

   public us.ihmc.idl.IDLSequence.Double getParameterValue()
   {
      return parameter_value_;
   }

   @Override
   public boolean epsilonEquals(SetDoubleArrayParameter other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.parameter_name_, other.parameter_name_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsDoubleSequence(this.parameter_value_, other.parameter_value_, epsilon))
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
      if (!(other instanceof SetDoubleArrayParameter))
         return false;

      SetDoubleArrayParameter otherMyClass = (SetDoubleArrayParameter) other;

      if (!us.ihmc.idl.IDLTools.equals(this.parameter_name_, otherMyClass.parameter_name_))
         return false;

      if (!this.parameter_value_.equals(otherMyClass.parameter_value_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("SetDoubleArrayParameter {");
      builder.append("parameter_name=");
      builder.append(this.parameter_name_);

      builder.append(", ");
      builder.append("parameter_value=");
      builder.append(this.parameter_value_);

      builder.append("}");
      return builder.toString();
   }
}