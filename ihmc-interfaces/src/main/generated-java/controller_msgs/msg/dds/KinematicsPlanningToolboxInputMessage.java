package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class KinematicsPlanningToolboxInputMessage extends Packet<KinematicsPlanningToolboxInputMessage> implements Settable<KinematicsPlanningToolboxInputMessage>, EpsilonComparable<KinematicsPlanningToolboxInputMessage>
{

   /**
            * This message is part of the IHMC whole-body inverse kinematics module: KinematicsPlanningToolbox.
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   /**
            * This is a list of rigid body messages which is used in KinematicsPlanningToolbox.
            * It should composed of at least one message. Each message represent the input for solving the trajectory for one rigid-body.
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.KinematicsPlanningToolboxRigidBodyMessage>  rigid_body_messages_;

   /**
            * This is desired key frames for center of mass position and will be used in KinematicsPlanningToolbox.
            * In case this message is empty, the kinematics solver will keep center of mass position in its current location.
            */
   public controller_msgs.msg.dds.KinematicsPlanningToolboxCenterOfMassMessage center_of_mass_message_;

   /**
            * This is kinematics configuration message and will configure whole body inverse kinematics solver.
            * This is optional.
            */
   public controller_msgs.msg.dds.KinematicsToolboxConfigurationMessage kinematics_configuration_message_;

   public KinematicsPlanningToolboxInputMessage()
   {


      rigid_body_messages_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.KinematicsPlanningToolboxRigidBodyMessage> (100, new controller_msgs.msg.dds.KinematicsPlanningToolboxRigidBodyMessagePubSubType());

      center_of_mass_message_ = new controller_msgs.msg.dds.KinematicsPlanningToolboxCenterOfMassMessage();

      kinematics_configuration_message_ = new controller_msgs.msg.dds.KinematicsToolboxConfigurationMessage();

   }

   public KinematicsPlanningToolboxInputMessage(KinematicsPlanningToolboxInputMessage other)
   {
      this();
      set(other);
   }

   public void set(KinematicsPlanningToolboxInputMessage other)
   {

      sequence_id_ = other.sequence_id_;


      rigid_body_messages_.set(other.rigid_body_messages_);

      controller_msgs.msg.dds.KinematicsPlanningToolboxCenterOfMassMessagePubSubType.staticCopy(other.center_of_mass_message_, center_of_mass_message_);

      controller_msgs.msg.dds.KinematicsToolboxConfigurationMessagePubSubType.staticCopy(other.kinematics_configuration_message_, kinematics_configuration_message_);
   }


   /**
            * This message is part of the IHMC whole-body inverse kinematics module: KinematicsPlanningToolbox.
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public void setSequenceId(long sequence_id)
   {
      sequence_id_ = sequence_id;
   }
   /**
            * This message is part of the IHMC whole-body inverse kinematics module: KinematicsPlanningToolbox.
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long getSequenceId()
   {
      return sequence_id_;
   }



   /**
            * This is a list of rigid body messages which is used in KinematicsPlanningToolbox.
            * It should composed of at least one message. Each message represent the input for solving the trajectory for one rigid-body.
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.KinematicsPlanningToolboxRigidBodyMessage>  getRigidBodyMessages()
   {
      return rigid_body_messages_;
   }



   /**
            * This is desired key frames for center of mass position and will be used in KinematicsPlanningToolbox.
            * In case this message is empty, the kinematics solver will keep center of mass position in its current location.
            */
   public controller_msgs.msg.dds.KinematicsPlanningToolboxCenterOfMassMessage getCenterOfMassMessage()
   {
      return center_of_mass_message_;
   }



   /**
            * This is kinematics configuration message and will configure whole body inverse kinematics solver.
            * This is optional.
            */
   public controller_msgs.msg.dds.KinematicsToolboxConfigurationMessage getKinematicsConfigurationMessage()
   {
      return kinematics_configuration_message_;
   }


   public static Supplier<KinematicsPlanningToolboxInputMessagePubSubType> getPubSubType()
   {
      return KinematicsPlanningToolboxInputMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return KinematicsPlanningToolboxInputMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(KinematicsPlanningToolboxInputMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (this.rigid_body_messages_.size() != other.rigid_body_messages_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.rigid_body_messages_.size(); i++)
         {  if (!this.rigid_body_messages_.get(i).epsilonEquals(other.rigid_body_messages_.get(i), epsilon)) return false; }
      }


      if (!this.center_of_mass_message_.epsilonEquals(other.center_of_mass_message_, epsilon)) return false;

      if (!this.kinematics_configuration_message_.epsilonEquals(other.kinematics_configuration_message_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof KinematicsPlanningToolboxInputMessage)) return false;

      KinematicsPlanningToolboxInputMessage otherMyClass = (KinematicsPlanningToolboxInputMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if (!this.rigid_body_messages_.equals(otherMyClass.rigid_body_messages_)) return false;

      if (!this.center_of_mass_message_.equals(otherMyClass.center_of_mass_message_)) return false;

      if (!this.kinematics_configuration_message_.equals(otherMyClass.kinematics_configuration_message_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("KinematicsPlanningToolboxInputMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("rigid_body_messages=");
      builder.append(this.rigid_body_messages_);      builder.append(", ");

      builder.append("center_of_mass_message=");
      builder.append(this.center_of_mass_message_);      builder.append(", ");

      builder.append("kinematics_configuration_message=");
      builder.append(this.kinematics_configuration_message_);
      builder.append("}");
      return builder.toString();
   }
}
