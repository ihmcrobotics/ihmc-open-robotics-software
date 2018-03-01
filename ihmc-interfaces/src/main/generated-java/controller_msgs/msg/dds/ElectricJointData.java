package controller_msgs.msg.dds;

/**
 * Definition of the class "ElectricJointData" defined in ElectricJointData_.idl.
 *
 * This file was automatically generated from ElectricJointData_.idl by us.ihmc.idl.generator.IDLGenerator.
 * Do not update this file directly, edit ElectricJointData_.idl instead.
 */
public class ElectricJointData
{
   private boolean joint_enabled_;
   private float joint_temperature_;
   private float joint_current_;

   public ElectricJointData()
   {

   }

   public void set(ElectricJointData other)
   {
      joint_enabled_ = other.joint_enabled_;
      joint_temperature_ = other.joint_temperature_;
      joint_current_ = other.joint_current_;
   }

   public boolean getJoint_enabled()
   {
      return joint_enabled_;
   }

   public void setJoint_enabled(boolean joint_enabled)
   {
      joint_enabled_ = joint_enabled;
   }

   public float getJoint_temperature()
   {
      return joint_temperature_;
   }

   public void setJoint_temperature(float joint_temperature)
   {
      joint_temperature_ = joint_temperature;
   }

   public float getJoint_current()
   {
      return joint_current_;
   }

   public void setJoint_current(float joint_current)
   {
      joint_current_ = joint_current;
   }

   @Override
   public boolean equals(java.lang.Object other)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;
      if (!(other instanceof ElectricJointData))
         return false;
      ElectricJointData otherMyClass = (ElectricJointData) other;
      boolean returnedValue = true;

      returnedValue &= this.joint_enabled_ == otherMyClass.joint_enabled_;

      returnedValue &= this.joint_temperature_ == otherMyClass.joint_temperature_;

      returnedValue &= this.joint_current_ == otherMyClass.joint_current_;

      return returnedValue;
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