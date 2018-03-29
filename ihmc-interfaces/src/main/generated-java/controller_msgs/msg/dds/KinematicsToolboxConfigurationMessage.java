package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * This message is part of the IHMC hole-body inverse kinematics module.
 * It contains auxiliary information that allows to further customized the behavior of the solver.
 */
public class KinematicsToolboxConfigurationMessage extends Packet<KinematicsToolboxConfigurationMessage>
      implements Settable<KinematicsToolboxConfigurationMessage>, EpsilonComparable<KinematicsToolboxConfigurationMessage>
{
   /**
    * When provided, the solver will attempt to find the solution that is the closest to the privileged configuration.
    */
   public us.ihmc.euclid.tuple3D.Point3D privileged_root_joint_position_;
   /**
    * When provided, the solver will attempt to find the solution that is the closest to the privileged configuration.
    */
   public us.ihmc.euclid.tuple4D.Quaternion privileged_root_joint_orientation_;
   /**
    * This array is used to identify to which joint each angle in privileged_joint_angles belongs to.
    * See AbstractInverseDynamicsJoint.getNameBaseHashCode() for the computation of a joint hash code.
    */
   public us.ihmc.idl.IDLSequence.Long privileged_joint_name_based_hash_codes_;
   /**
    * When provided, the solver will attempt to find the solution that is the closest to the privileged configuration.
    */
   public us.ihmc.idl.IDLSequence.Float privileged_joint_angles_;

   public KinematicsToolboxConfigurationMessage()
   {
      privileged_root_joint_position_ = new us.ihmc.euclid.tuple3D.Point3D();
      privileged_root_joint_orientation_ = new us.ihmc.euclid.tuple4D.Quaternion();
      privileged_joint_name_based_hash_codes_ = new us.ihmc.idl.IDLSequence.Long(100, "type_11");

      privileged_joint_angles_ = new us.ihmc.idl.IDLSequence.Float(100, "type_5");
   }

   public KinematicsToolboxConfigurationMessage(KinematicsToolboxConfigurationMessage other)
   {
      set(other);
   }

   public void set(KinematicsToolboxConfigurationMessage other)
   {
      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.privileged_root_joint_position_, privileged_root_joint_position_);
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.privileged_root_joint_orientation_, privileged_root_joint_orientation_);
      privileged_joint_name_based_hash_codes_.set(other.privileged_joint_name_based_hash_codes_);
      privileged_joint_angles_.set(other.privileged_joint_angles_);
   }

   /**
    * When provided, the solver will attempt to find the solution that is the closest to the privileged configuration.
    */
   public us.ihmc.euclid.tuple3D.Point3D getPrivilegedRootJointPosition()
   {
      return privileged_root_joint_position_;
   }

   /**
    * When provided, the solver will attempt to find the solution that is the closest to the privileged configuration.
    */
   public us.ihmc.euclid.tuple4D.Quaternion getPrivilegedRootJointOrientation()
   {
      return privileged_root_joint_orientation_;
   }

   /**
    * This array is used to identify to which joint each angle in privileged_joint_angles belongs to.
    * See AbstractInverseDynamicsJoint.getNameBaseHashCode() for the computation of a joint hash code.
    */
   public us.ihmc.idl.IDLSequence.Long getPrivilegedJointNameBasedHashCodes()
   {
      return privileged_joint_name_based_hash_codes_;
   }

   /**
    * When provided, the solver will attempt to find the solution that is the closest to the privileged configuration.
    */
   public us.ihmc.idl.IDLSequence.Float getPrivilegedJointAngles()
   {
      return privileged_joint_angles_;
   }

   @Override
   public boolean epsilonEquals(KinematicsToolboxConfigurationMessage other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.privileged_root_joint_position_.epsilonEquals(other.privileged_root_joint_position_, epsilon))
         return false;

      if (!this.privileged_root_joint_orientation_.epsilonEquals(other.privileged_root_joint_orientation_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsLongSequence(this.privileged_joint_name_based_hash_codes_, other.privileged_joint_name_based_hash_codes_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.privileged_joint_angles_, other.privileged_joint_angles_, epsilon))
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
      if (!(other instanceof KinematicsToolboxConfigurationMessage))
         return false;

      KinematicsToolboxConfigurationMessage otherMyClass = (KinematicsToolboxConfigurationMessage) other;

      if (!this.privileged_root_joint_position_.equals(otherMyClass.privileged_root_joint_position_))
         return false;

      if (!this.privileged_root_joint_orientation_.equals(otherMyClass.privileged_root_joint_orientation_))
         return false;

      if (!this.privileged_joint_name_based_hash_codes_.equals(otherMyClass.privileged_joint_name_based_hash_codes_))
         return false;

      if (!this.privileged_joint_angles_.equals(otherMyClass.privileged_joint_angles_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("KinematicsToolboxConfigurationMessage {");
      builder.append("privileged_root_joint_position=");
      builder.append(this.privileged_root_joint_position_);

      builder.append(", ");
      builder.append("privileged_root_joint_orientation=");
      builder.append(this.privileged_root_joint_orientation_);

      builder.append(", ");
      builder.append("privileged_joint_name_based_hash_codes=");
      builder.append(this.privileged_joint_name_based_hash_codes_);

      builder.append(", ");
      builder.append("privileged_joint_angles=");
      builder.append(this.privileged_joint_angles_);

      builder.append("}");
      return builder.toString();
   }
}
