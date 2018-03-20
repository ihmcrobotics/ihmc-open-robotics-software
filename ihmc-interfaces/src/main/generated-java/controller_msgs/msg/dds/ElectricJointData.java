package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

public class ElectricJointData extends Packet<ElectricJointData> implements Settable<ElectricJointData>, EpsilonComparable<ElectricJointData>
{
   public boolean joint_enabled_;
   public float joint_temperature_;
   public float joint_current_;

   public ElectricJointData()
   {

   }

   public ElectricJointData(ElectricJointData other)
   {
      set(other);
   }

   public void set(ElectricJointData other)
   {
      joint_enabled_ = other.joint_enabled_;

      joint_temperature_ = other.joint_temperature_;

      joint_current_ = other.joint_current_;
   }

   public boolean getJointEnabled()
   {
      return joint_enabled_;
   }

   public void setJointEnabled(boolean joint_enabled)
   {
      joint_enabled_ = joint_enabled;
   }

   public float getJointTemperature()
   {
      return joint_temperature_;
   }

   public void setJointTemperature(float joint_temperature)
   {
      joint_temperature_ = joint_temperature;
   }

   public float getJointCurrent()
   {
      return joint_current_;
   }

   public void setJointCurrent(float joint_current)
   {
      joint_current_ = joint_current;
   }

   @Override
   public boolean epsilonEquals(ElectricJointData other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.joint_enabled_, other.joint_enabled_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.joint_temperature_, other.joint_temperature_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.joint_current_, other.joint_current_, epsilon))
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
      if (!(other instanceof ElectricJointData))
         return false;

      ElectricJointData otherMyClass = (ElectricJointData) other;

      if (this.joint_enabled_ != otherMyClass.joint_enabled_)
         return false;

      if (this.joint_temperature_ != otherMyClass.joint_temperature_)
         return false;

      if (this.joint_current_ != otherMyClass.joint_current_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ElectricJointData {");
      builder.append("joint_enabled=");
      builder.append(this.joint_enabled_);

      builder.append(", ");
      builder.append("joint_temperature=");
      builder.append(this.joint_temperature_);

      builder.append(", ");
      builder.append("joint_current=");
      builder.append(this.joint_current_);

      builder.append("}");
      return builder.toString();
   }
}