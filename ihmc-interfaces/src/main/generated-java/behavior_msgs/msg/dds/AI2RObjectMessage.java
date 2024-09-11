package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * For use by external AI to modify the behavior tree and operate behaviors
       */
public class AI2RObjectMessage extends Packet<AI2RObjectMessage> implements Settable<AI2RObjectMessage>, EpsilonComparable<AI2RObjectMessage>
{
   /**
            * Name of the object
            */
   public java.lang.StringBuilder object_name_;
   /**
            * Pose of the object in world frame
            */
   public us.ihmc.euclid.geometry.Pose3D object_pose_in_world_;

   public AI2RObjectMessage()
   {
      object_name_ = new java.lang.StringBuilder(255);
      object_pose_in_world_ = new us.ihmc.euclid.geometry.Pose3D();
   }

   public AI2RObjectMessage(AI2RObjectMessage other)
   {
      this();
      set(other);
   }

   public void set(AI2RObjectMessage other)
   {
      object_name_.setLength(0);
      object_name_.append(other.object_name_);

      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.object_pose_in_world_, object_pose_in_world_);
   }

   /**
            * Name of the object
            */
   public void setObjectName(java.lang.String object_name)
   {
      object_name_.setLength(0);
      object_name_.append(object_name);
   }

   /**
            * Name of the object
            */
   public java.lang.String getObjectNameAsString()
   {
      return getObjectName().toString();
   }
   /**
            * Name of the object
            */
   public java.lang.StringBuilder getObjectName()
   {
      return object_name_;
   }


   /**
            * Pose of the object in world frame
            */
   public us.ihmc.euclid.geometry.Pose3D getObjectPoseInWorld()
   {
      return object_pose_in_world_;
   }


   public static Supplier<AI2RObjectMessagePubSubType> getPubSubType()
   {
      return AI2RObjectMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return AI2RObjectMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(AI2RObjectMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.object_name_, other.object_name_, epsilon)) return false;

      if (!this.object_pose_in_world_.epsilonEquals(other.object_pose_in_world_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof AI2RObjectMessage)) return false;

      AI2RObjectMessage otherMyClass = (AI2RObjectMessage) other;

      if (!us.ihmc.idl.IDLTools.equals(this.object_name_, otherMyClass.object_name_)) return false;

      if (!this.object_pose_in_world_.equals(otherMyClass.object_pose_in_world_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AI2RObjectMessage {");
      builder.append("object_name=");
      builder.append(this.object_name_);      builder.append(", ");
      builder.append("object_pose_in_world=");
      builder.append(this.object_pose_in_world_);
      builder.append("}");
      return builder.toString();
   }
}
