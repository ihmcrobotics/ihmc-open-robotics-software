package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * General purpose message normally used to report the solution of a whole-body inverse kinematics solver.
 * Main usage is for the IHMC KinematicsToolbox.
 */
public class KinematicsToolboxOutputStatus extends Packet<KinematicsToolboxOutputStatus>
      implements Settable<KinematicsToolboxOutputStatus>, EpsilonComparable<KinematicsToolboxOutputStatus>
{
   public int joint_name_hash_;
   public us.ihmc.idl.IDLSequence.Float desired_joint_angles_;
   public us.ihmc.euclid.tuple3D.Vector3D desired_root_translation_;
   public us.ihmc.euclid.tuple4D.Quaternion desired_root_orientation_;
   public double solution_quality_;

   public KinematicsToolboxOutputStatus()
   {

      desired_joint_angles_ = new us.ihmc.idl.IDLSequence.Float(100, "type_5");

      desired_root_translation_ = new us.ihmc.euclid.tuple3D.Vector3D();
      desired_root_orientation_ = new us.ihmc.euclid.tuple4D.Quaternion();
   }

   public KinematicsToolboxOutputStatus(KinematicsToolboxOutputStatus other)
   {
      set(other);
   }

   public void set(KinematicsToolboxOutputStatus other)
   {
      joint_name_hash_ = other.joint_name_hash_;

      desired_joint_angles_.set(other.desired_joint_angles_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.desired_root_translation_, desired_root_translation_);
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.desired_root_orientation_, desired_root_orientation_);
      solution_quality_ = other.solution_quality_;
   }

   public int getJointNameHash()
   {
      return joint_name_hash_;
   }

   public void setJointNameHash(int joint_name_hash)
   {
      joint_name_hash_ = joint_name_hash;
   }

   public us.ihmc.idl.IDLSequence.Float getDesiredJointAngles()
   {
      return desired_joint_angles_;
   }

   public us.ihmc.euclid.tuple3D.Vector3D getDesiredRootTranslation()
   {
      return desired_root_translation_;
   }

   public us.ihmc.euclid.tuple4D.Quaternion getDesiredRootOrientation()
   {
      return desired_root_orientation_;
   }

   public double getSolutionQuality()
   {
      return solution_quality_;
   }

   public void setSolutionQuality(double solution_quality)
   {
      solution_quality_ = solution_quality;
   }

   @Override
   public boolean epsilonEquals(KinematicsToolboxOutputStatus other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.joint_name_hash_, other.joint_name_hash_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.desired_joint_angles_, other.desired_joint_angles_, epsilon))
         return false;

      if (!this.desired_root_translation_.epsilonEquals(other.desired_root_translation_, epsilon))
         return false;

      if (!this.desired_root_orientation_.epsilonEquals(other.desired_root_orientation_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.solution_quality_, other.solution_quality_, epsilon))
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
      if (!(other instanceof KinematicsToolboxOutputStatus))
         return false;

      KinematicsToolboxOutputStatus otherMyClass = (KinematicsToolboxOutputStatus) other;

      if (this.joint_name_hash_ != otherMyClass.joint_name_hash_)
         return false;

      if (!this.desired_joint_angles_.equals(otherMyClass.desired_joint_angles_))
         return false;

      if (!this.desired_root_translation_.equals(otherMyClass.desired_root_translation_))
         return false;

      if (!this.desired_root_orientation_.equals(otherMyClass.desired_root_orientation_))
         return false;

      if (this.solution_quality_ != otherMyClass.solution_quality_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("KinematicsToolboxOutputStatus {");
      builder.append("joint_name_hash=");
      builder.append(this.joint_name_hash_);

      builder.append(", ");
      builder.append("desired_joint_angles=");
      builder.append(this.desired_joint_angles_);

      builder.append(", ");
      builder.append("desired_root_translation=");
      builder.append(this.desired_root_translation_);

      builder.append(", ");
      builder.append("desired_root_orientation=");
      builder.append(this.desired_root_orientation_);

      builder.append(", ");
      builder.append("solution_quality=");
      builder.append(this.solution_quality_);

      builder.append("}");
      return builder.toString();
   }
}
