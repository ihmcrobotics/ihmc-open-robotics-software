package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Status of robot, environment, and behaviors for use by external AI
       */
public class AI2RStatusMessage extends Packet<AI2RStatusMessage> implements Settable<AI2RStatusMessage>, EpsilonComparable<AI2RStatusMessage>
{
   /**
            * Pose of the robot.
            * Mid point between footsoles and facing the direction of the pelvis in world frame.
            * Z always points straight up.
            */
   public us.ihmc.euclid.geometry.Pose3D robot_mid_feet_under_pelvis_pose_in_world_;
   /**
            * List of objects in the scene
            */
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.AI2RObjectMessage>  objects_;
   /**
            * List of available behaviors (JSON file names)
            */
   public us.ihmc.idl.IDLSequence.StringBuilderHolder  available_behaviors_;

   public AI2RStatusMessage()
   {
      robot_mid_feet_under_pelvis_pose_in_world_ = new us.ihmc.euclid.geometry.Pose3D();
      objects_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.AI2RObjectMessage> (200, new behavior_msgs.msg.dds.AI2RObjectMessagePubSubType());
      available_behaviors_ = new us.ihmc.idl.IDLSequence.StringBuilderHolder (200, "type_d");

   }

   public AI2RStatusMessage(AI2RStatusMessage other)
   {
      this();
      set(other);
   }

   public void set(AI2RStatusMessage other)
   {
      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.robot_mid_feet_under_pelvis_pose_in_world_, robot_mid_feet_under_pelvis_pose_in_world_);
      objects_.set(other.objects_);
      available_behaviors_.set(other.available_behaviors_);
   }


   /**
            * Pose of the robot.
            * Mid point between footsoles and facing the direction of the pelvis in world frame.
            * Z always points straight up.
            */
   public us.ihmc.euclid.geometry.Pose3D getRobotMidFeetUnderPelvisPoseInWorld()
   {
      return robot_mid_feet_under_pelvis_pose_in_world_;
   }


   /**
            * List of objects in the scene
            */
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.AI2RObjectMessage>  getObjects()
   {
      return objects_;
   }


   /**
            * List of available behaviors (JSON file names)
            */
   public us.ihmc.idl.IDLSequence.StringBuilderHolder  getAvailableBehaviors()
   {
      return available_behaviors_;
   }


   public static Supplier<AI2RStatusMessagePubSubType> getPubSubType()
   {
      return AI2RStatusMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return AI2RStatusMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(AI2RStatusMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.robot_mid_feet_under_pelvis_pose_in_world_.epsilonEquals(other.robot_mid_feet_under_pelvis_pose_in_world_, epsilon)) return false;
      if (this.objects_.size() != other.objects_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.objects_.size(); i++)
         {  if (!this.objects_.get(i).epsilonEquals(other.objects_.get(i), epsilon)) return false; }
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilderSequence(this.available_behaviors_, other.available_behaviors_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof AI2RStatusMessage)) return false;

      AI2RStatusMessage otherMyClass = (AI2RStatusMessage) other;

      if (!this.robot_mid_feet_under_pelvis_pose_in_world_.equals(otherMyClass.robot_mid_feet_under_pelvis_pose_in_world_)) return false;
      if (!this.objects_.equals(otherMyClass.objects_)) return false;
      if (!this.available_behaviors_.equals(otherMyClass.available_behaviors_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AI2RStatusMessage {");
      builder.append("robot_mid_feet_under_pelvis_pose_in_world=");
      builder.append(this.robot_mid_feet_under_pelvis_pose_in_world_);      builder.append(", ");
      builder.append("objects=");
      builder.append(this.objects_);      builder.append(", ");
      builder.append("available_behaviors=");
      builder.append(this.available_behaviors_);
      builder.append("}");
      return builder.toString();
   }
}
