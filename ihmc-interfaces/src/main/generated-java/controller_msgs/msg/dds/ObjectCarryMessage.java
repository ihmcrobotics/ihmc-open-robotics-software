package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class ObjectCarryMessage extends Packet<ObjectCarryMessage> implements Settable<ObjectCarryMessage>, EpsilonComparable<ObjectCarryMessage>
{
   public us.ihmc.euclid.geometry.Pose3D object_pose_;
   public boolean object_is_carried_;

   public ObjectCarryMessage()
   {
      object_pose_ = new us.ihmc.euclid.geometry.Pose3D();
   }

   public ObjectCarryMessage(ObjectCarryMessage other)
   {
      this();
      set(other);
   }

   public void set(ObjectCarryMessage other)
   {
      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.object_pose_, object_pose_);
      object_is_carried_ = other.object_is_carried_;

   }


   public us.ihmc.euclid.geometry.Pose3D getObjectPose()
   {
      return object_pose_;
   }

   public void setObjectIsCarried(boolean object_is_carried)
   {
      object_is_carried_ = object_is_carried;
   }
   public boolean getObjectIsCarried()
   {
      return object_is_carried_;
   }


   public static Supplier<ObjectCarryMessagePubSubType> getPubSubType()
   {
      return ObjectCarryMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ObjectCarryMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ObjectCarryMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.object_pose_.epsilonEquals(other.object_pose_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.object_is_carried_, other.object_is_carried_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ObjectCarryMessage)) return false;

      ObjectCarryMessage otherMyClass = (ObjectCarryMessage) other;

      if (!this.object_pose_.equals(otherMyClass.object_pose_)) return false;
      if(this.object_is_carried_ != otherMyClass.object_is_carried_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ObjectCarryMessage {");
      builder.append("object_pose=");
      builder.append(this.object_pose_);      builder.append(", ");
      builder.append("object_is_carried=");
      builder.append(this.object_is_carried_);
      builder.append("}");
      return builder.toString();
   }
}
