package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

/**
 * This message is part of the IHMC whole-body inverse kinematics module.
 */
public class HumanoidKinematicsToolboxConfigurationMessage extends Packet<HumanoidKinematicsToolboxConfigurationMessage>
      implements Settable<HumanoidKinematicsToolboxConfigurationMessage>, EpsilonComparable<HumanoidKinematicsToolboxConfigurationMessage>
{
   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header header_;
   /**
    * When set to true, the solver will hold the current x and y coordinates of the center of mass.
    * By 'current', it means that the solver will use the robot configuration data broadcasted by
    * the controller to obtain the center of mass position.
    */
   public boolean hold_current_center_of_mass_xy_position_ = true;
   /**
    * When set to true, the solver will hold the pose of the active support foot/feet.
    */
   public boolean hold_support_foot_positions_ = true;

   public HumanoidKinematicsToolboxConfigurationMessage()
   {
      header_ = new std_msgs.msg.dds.Header();
   }

   public HumanoidKinematicsToolboxConfigurationMessage(HumanoidKinematicsToolboxConfigurationMessage other)
   {
      this();
      set(other);
   }

   public void set(HumanoidKinematicsToolboxConfigurationMessage other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      hold_current_center_of_mass_xy_position_ = other.hold_current_center_of_mass_xy_position_;

      hold_support_foot_positions_ = other.hold_support_foot_positions_;

   }

   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }

   /**
    * When set to true, the solver will hold the current x and y coordinates of the center of mass.
    * By 'current', it means that the solver will use the robot configuration data broadcasted by
    * the controller to obtain the center of mass position.
    */
   public void setHoldCurrentCenterOfMassXyPosition(boolean hold_current_center_of_mass_xy_position)
   {
      hold_current_center_of_mass_xy_position_ = hold_current_center_of_mass_xy_position;
   }

   /**
    * When set to true, the solver will hold the current x and y coordinates of the center of mass.
    * By 'current', it means that the solver will use the robot configuration data broadcasted by
    * the controller to obtain the center of mass position.
    */
   public boolean getHoldCurrentCenterOfMassXyPosition()
   {
      return hold_current_center_of_mass_xy_position_;
   }

   /**
    * When set to true, the solver will hold the pose of the active support foot/feet.
    */
   public void setHoldSupportFootPositions(boolean hold_support_foot_positions)
   {
      hold_support_foot_positions_ = hold_support_foot_positions;
   }

   /**
    * When set to true, the solver will hold the pose of the active support foot/feet.
    */
   public boolean getHoldSupportFootPositions()
   {
      return hold_support_foot_positions_;
   }

   @Override
   public boolean epsilonEquals(HumanoidKinematicsToolboxConfigurationMessage other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon))
         return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.hold_current_center_of_mass_xy_position_, other.hold_current_center_of_mass_xy_position_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.hold_support_foot_positions_, other.hold_support_foot_positions_, epsilon))
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
      if (!(other instanceof HumanoidKinematicsToolboxConfigurationMessage))
         return false;

      HumanoidKinematicsToolboxConfigurationMessage otherMyClass = (HumanoidKinematicsToolboxConfigurationMessage) other;

      if (!this.header_.equals(otherMyClass.header_))
         return false;
      if (this.hold_current_center_of_mass_xy_position_ != otherMyClass.hold_current_center_of_mass_xy_position_)
         return false;

      if (this.hold_support_foot_positions_ != otherMyClass.hold_support_foot_positions_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("HumanoidKinematicsToolboxConfigurationMessage {");
      builder.append("header=");
      builder.append(this.header_);
      builder.append(", ");
      builder.append("hold_current_center_of_mass_xy_position=");
      builder.append(this.hold_current_center_of_mass_xy_position_);
      builder.append(", ");
      builder.append("hold_support_foot_positions=");
      builder.append(this.hold_support_foot_positions_);
      builder.append("}");
      return builder.toString();
   }
}
