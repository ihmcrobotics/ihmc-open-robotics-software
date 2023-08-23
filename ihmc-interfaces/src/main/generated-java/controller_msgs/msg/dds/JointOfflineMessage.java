package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * This message notifies the controller of a joint going offline, which causes the controller to go into an emergency fall-prevention mode.
       */
public class JointOfflineMessage extends Packet<JointOfflineMessage> implements Settable<JointOfflineMessage>, EpsilonComparable<JointOfflineMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Hash-code of the joints which are offline. Joints that are currently offline and not included in this list are re-enabled.
            * The hash-code is computed from OneDoFJoint#hashcode().
            */
   public us.ihmc.idl.IDLSequence.Integer  joint_offline_hash_codes_;
   /**
            * Nominal left foot contact points (in sole frame) given the set of offline joints
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  nominal_left_foot_contact_points_2d_;
   /**
            * Nominal right foot contact points (in sole frame) given the set of offline joints
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  nominal_right_foot_contact_points_2d_;
   /**
            * The time to delay this command on the controller side before being executed.
            */
   public double execution_delay_time_;

   public JointOfflineMessage()
   {
      joint_offline_hash_codes_ = new us.ihmc.idl.IDLSequence.Integer (6, "type_2");

      nominal_left_foot_contact_points_2d_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D> (4, new geometry_msgs.msg.dds.PointPubSubType());
      nominal_right_foot_contact_points_2d_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D> (4, new geometry_msgs.msg.dds.PointPubSubType());

   }

   public JointOfflineMessage(JointOfflineMessage other)
   {
      this();
      set(other);
   }

   public void set(JointOfflineMessage other)
   {
      sequence_id_ = other.sequence_id_;

      joint_offline_hash_codes_.set(other.joint_offline_hash_codes_);
      nominal_left_foot_contact_points_2d_.set(other.nominal_left_foot_contact_points_2d_);
      nominal_right_foot_contact_points_2d_.set(other.nominal_right_foot_contact_points_2d_);
      execution_delay_time_ = other.execution_delay_time_;

   }

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public void setSequenceId(long sequence_id)
   {
      sequence_id_ = sequence_id;
   }
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long getSequenceId()
   {
      return sequence_id_;
   }


   /**
            * Hash-code of the joints which are offline. Joints that are currently offline and not included in this list are re-enabled.
            * The hash-code is computed from OneDoFJoint#hashcode().
            */
   public us.ihmc.idl.IDLSequence.Integer  getJointOfflineHashCodes()
   {
      return joint_offline_hash_codes_;
   }


   /**
            * Nominal left foot contact points (in sole frame) given the set of offline joints
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  getNominalLeftFootContactPoints2d()
   {
      return nominal_left_foot_contact_points_2d_;
   }


   /**
            * Nominal right foot contact points (in sole frame) given the set of offline joints
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  getNominalRightFootContactPoints2d()
   {
      return nominal_right_foot_contact_points_2d_;
   }

   /**
            * The time to delay this command on the controller side before being executed.
            */
   public void setExecutionDelayTime(double execution_delay_time)
   {
      execution_delay_time_ = execution_delay_time;
   }
   /**
            * The time to delay this command on the controller side before being executed.
            */
   public double getExecutionDelayTime()
   {
      return execution_delay_time_;
   }


   public static Supplier<JointOfflineMessagePubSubType> getPubSubType()
   {
      return JointOfflineMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return JointOfflineMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(JointOfflineMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsIntegerSequence(this.joint_offline_hash_codes_, other.joint_offline_hash_codes_, epsilon)) return false;

      if (this.nominal_left_foot_contact_points_2d_.size() != other.nominal_left_foot_contact_points_2d_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.nominal_left_foot_contact_points_2d_.size(); i++)
         {  if (!this.nominal_left_foot_contact_points_2d_.get(i).epsilonEquals(other.nominal_left_foot_contact_points_2d_.get(i), epsilon)) return false; }
      }

      if (this.nominal_right_foot_contact_points_2d_.size() != other.nominal_right_foot_contact_points_2d_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.nominal_right_foot_contact_points_2d_.size(); i++)
         {  if (!this.nominal_right_foot_contact_points_2d_.get(i).epsilonEquals(other.nominal_right_foot_contact_points_2d_.get(i), epsilon)) return false; }
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.execution_delay_time_, other.execution_delay_time_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof JointOfflineMessage)) return false;

      JointOfflineMessage otherMyClass = (JointOfflineMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if (!this.joint_offline_hash_codes_.equals(otherMyClass.joint_offline_hash_codes_)) return false;
      if (!this.nominal_left_foot_contact_points_2d_.equals(otherMyClass.nominal_left_foot_contact_points_2d_)) return false;
      if (!this.nominal_right_foot_contact_points_2d_.equals(otherMyClass.nominal_right_foot_contact_points_2d_)) return false;
      if(this.execution_delay_time_ != otherMyClass.execution_delay_time_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("JointOfflineMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("joint_offline_hash_codes=");
      builder.append(this.joint_offline_hash_codes_);      builder.append(", ");
      builder.append("nominal_left_foot_contact_points_2d=");
      builder.append(this.nominal_left_foot_contact_points_2d_);      builder.append(", ");
      builder.append("nominal_right_foot_contact_points_2d=");
      builder.append(this.nominal_right_foot_contact_points_2d_);      builder.append(", ");
      builder.append("execution_delay_time=");
      builder.append(this.execution_delay_time_);
      builder.append("}");
      return builder.toString();
   }
}
