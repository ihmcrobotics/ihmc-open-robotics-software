package toolbox_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the KinematicsStreamingToolbox API.
       * Allows to specify the messages the toolbox should stream to the controller.
       */
public class KinematicsStreamingToolboxConfigurationMessage extends Packet<KinematicsStreamingToolboxConfigurationMessage> implements Settable<KinematicsStreamingToolboxConfigurationMessage>, EpsilonComparable<KinematicsStreamingToolboxConfigurationMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Whether the pelvis should pinned or free to move.
            * Default value is false.
            */
   public boolean lock_pelvis_;
   /**
            * Whether the chest should pinned or free to move.
            * Default value is false.
            */
   public boolean lock_chest_;
   /**
            * Whether the left arm should be controller in joint-space.
            * This is compatible with the hand task-space control.
            * Default value is true.
            */
   public boolean enable_left_arm_jointspace_ = true;
   /**
            * Whether the right arm should be controller in joint-space.
            * This is compatible with the hand task-space control.
            * Default value is true.
            */
   public boolean enable_right_arm_jointspace_ = true;
   /**
            * Whether the neck should be controller in joint-space.
            * Default value is true.
            * Whether the neck should be controller in joint-space.
            */
   public boolean enable_neck_jointspace_ = true;
   /**
            * Whether the left hand should be controlled in task-space.
            * This is compatible with the arm task-space control.
            * Default value is true.
            */
   public boolean enable_left_hand_taskspace_ = true;
   /**
            * Whether the right hand should be controlled in task-space.
            * This is compatible with the arm task-space control.
            * Default value is true.
            */
   public boolean enable_right_hand_taskspace_ = true;
   /**
            * Whether the chest orientation should be controlled.
            * Default value is true.
            */
   public boolean enable_chest_taskspace_ = true;
   /**
            * Whether the pelvis should be controlled in task-space.
            * Default value is true.
            */
   public boolean enable_pelvis_taskspace_ = true;
   /**
            * Reference frame in which the controller should be tracking the pose for the left hand as computed by the whole-body IK.
            * Default value is WORLD_FRAME, see FrameInformation for useful frame ids.
            */
   public long left_hand_trajectory_frame_id_ = 83766130;
   /**
            * Reference frame in which the controller should be tracking the pose for the right hand as computed by the whole-body IK.
            * Default value is WORLD_FRAME, see FrameInformation for useful frame ids.
            */
   public long right_hand_trajectory_frame_id_ = 83766130;
   /**
            * Reference frame in which the controller should be tracking the pose for the chest as computed by the whole-body IK.
            * Default value is WORLD_FRAME, see FrameInformation for useful frame ids.
            */
   public long chest_trajectory_frame_id_ = 83766130;
   /**
            * Reference frame in which the controller should be tracking the pose for the pelvis as computed by the whole-body IK.
            * Default value is WORLD_FRAME, see FrameInformation for useful frame ids.
            */
   public long pelvis_trajectory_frame_id_ = 83766130;

   public KinematicsStreamingToolboxConfigurationMessage()
   {
   }

   public KinematicsStreamingToolboxConfigurationMessage(KinematicsStreamingToolboxConfigurationMessage other)
   {
      this();
      set(other);
   }

   public void set(KinematicsStreamingToolboxConfigurationMessage other)
   {
      sequence_id_ = other.sequence_id_;

      lock_pelvis_ = other.lock_pelvis_;

      lock_chest_ = other.lock_chest_;

      enable_left_arm_jointspace_ = other.enable_left_arm_jointspace_;

      enable_right_arm_jointspace_ = other.enable_right_arm_jointspace_;

      enable_neck_jointspace_ = other.enable_neck_jointspace_;

      enable_left_hand_taskspace_ = other.enable_left_hand_taskspace_;

      enable_right_hand_taskspace_ = other.enable_right_hand_taskspace_;

      enable_chest_taskspace_ = other.enable_chest_taskspace_;

      enable_pelvis_taskspace_ = other.enable_pelvis_taskspace_;

      left_hand_trajectory_frame_id_ = other.left_hand_trajectory_frame_id_;

      right_hand_trajectory_frame_id_ = other.right_hand_trajectory_frame_id_;

      chest_trajectory_frame_id_ = other.chest_trajectory_frame_id_;

      pelvis_trajectory_frame_id_ = other.pelvis_trajectory_frame_id_;

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
            * Whether the pelvis should pinned or free to move.
            * Default value is false.
            */
   public void setLockPelvis(boolean lock_pelvis)
   {
      lock_pelvis_ = lock_pelvis;
   }
   /**
            * Whether the pelvis should pinned or free to move.
            * Default value is false.
            */
   public boolean getLockPelvis()
   {
      return lock_pelvis_;
   }

   /**
            * Whether the chest should pinned or free to move.
            * Default value is false.
            */
   public void setLockChest(boolean lock_chest)
   {
      lock_chest_ = lock_chest;
   }
   /**
            * Whether the chest should pinned or free to move.
            * Default value is false.
            */
   public boolean getLockChest()
   {
      return lock_chest_;
   }

   /**
            * Whether the left arm should be controller in joint-space.
            * This is compatible with the hand task-space control.
            * Default value is true.
            */
   public void setEnableLeftArmJointspace(boolean enable_left_arm_jointspace)
   {
      enable_left_arm_jointspace_ = enable_left_arm_jointspace;
   }
   /**
            * Whether the left arm should be controller in joint-space.
            * This is compatible with the hand task-space control.
            * Default value is true.
            */
   public boolean getEnableLeftArmJointspace()
   {
      return enable_left_arm_jointspace_;
   }

   /**
            * Whether the right arm should be controller in joint-space.
            * This is compatible with the hand task-space control.
            * Default value is true.
            */
   public void setEnableRightArmJointspace(boolean enable_right_arm_jointspace)
   {
      enable_right_arm_jointspace_ = enable_right_arm_jointspace;
   }
   /**
            * Whether the right arm should be controller in joint-space.
            * This is compatible with the hand task-space control.
            * Default value is true.
            */
   public boolean getEnableRightArmJointspace()
   {
      return enable_right_arm_jointspace_;
   }

   /**
            * Whether the neck should be controller in joint-space.
            * Default value is true.
            * Whether the neck should be controller in joint-space.
            */
   public void setEnableNeckJointspace(boolean enable_neck_jointspace)
   {
      enable_neck_jointspace_ = enable_neck_jointspace;
   }
   /**
            * Whether the neck should be controller in joint-space.
            * Default value is true.
            * Whether the neck should be controller in joint-space.
            */
   public boolean getEnableNeckJointspace()
   {
      return enable_neck_jointspace_;
   }

   /**
            * Whether the left hand should be controlled in task-space.
            * This is compatible with the arm task-space control.
            * Default value is true.
            */
   public void setEnableLeftHandTaskspace(boolean enable_left_hand_taskspace)
   {
      enable_left_hand_taskspace_ = enable_left_hand_taskspace;
   }
   /**
            * Whether the left hand should be controlled in task-space.
            * This is compatible with the arm task-space control.
            * Default value is true.
            */
   public boolean getEnableLeftHandTaskspace()
   {
      return enable_left_hand_taskspace_;
   }

   /**
            * Whether the right hand should be controlled in task-space.
            * This is compatible with the arm task-space control.
            * Default value is true.
            */
   public void setEnableRightHandTaskspace(boolean enable_right_hand_taskspace)
   {
      enable_right_hand_taskspace_ = enable_right_hand_taskspace;
   }
   /**
            * Whether the right hand should be controlled in task-space.
            * This is compatible with the arm task-space control.
            * Default value is true.
            */
   public boolean getEnableRightHandTaskspace()
   {
      return enable_right_hand_taskspace_;
   }

   /**
            * Whether the chest orientation should be controlled.
            * Default value is true.
            */
   public void setEnableChestTaskspace(boolean enable_chest_taskspace)
   {
      enable_chest_taskspace_ = enable_chest_taskspace;
   }
   /**
            * Whether the chest orientation should be controlled.
            * Default value is true.
            */
   public boolean getEnableChestTaskspace()
   {
      return enable_chest_taskspace_;
   }

   /**
            * Whether the pelvis should be controlled in task-space.
            * Default value is true.
            */
   public void setEnablePelvisTaskspace(boolean enable_pelvis_taskspace)
   {
      enable_pelvis_taskspace_ = enable_pelvis_taskspace;
   }
   /**
            * Whether the pelvis should be controlled in task-space.
            * Default value is true.
            */
   public boolean getEnablePelvisTaskspace()
   {
      return enable_pelvis_taskspace_;
   }

   /**
            * Reference frame in which the controller should be tracking the pose for the left hand as computed by the whole-body IK.
            * Default value is WORLD_FRAME, see FrameInformation for useful frame ids.
            */
   public void setLeftHandTrajectoryFrameId(long left_hand_trajectory_frame_id)
   {
      left_hand_trajectory_frame_id_ = left_hand_trajectory_frame_id;
   }
   /**
            * Reference frame in which the controller should be tracking the pose for the left hand as computed by the whole-body IK.
            * Default value is WORLD_FRAME, see FrameInformation for useful frame ids.
            */
   public long getLeftHandTrajectoryFrameId()
   {
      return left_hand_trajectory_frame_id_;
   }

   /**
            * Reference frame in which the controller should be tracking the pose for the right hand as computed by the whole-body IK.
            * Default value is WORLD_FRAME, see FrameInformation for useful frame ids.
            */
   public void setRightHandTrajectoryFrameId(long right_hand_trajectory_frame_id)
   {
      right_hand_trajectory_frame_id_ = right_hand_trajectory_frame_id;
   }
   /**
            * Reference frame in which the controller should be tracking the pose for the right hand as computed by the whole-body IK.
            * Default value is WORLD_FRAME, see FrameInformation for useful frame ids.
            */
   public long getRightHandTrajectoryFrameId()
   {
      return right_hand_trajectory_frame_id_;
   }

   /**
            * Reference frame in which the controller should be tracking the pose for the chest as computed by the whole-body IK.
            * Default value is WORLD_FRAME, see FrameInformation for useful frame ids.
            */
   public void setChestTrajectoryFrameId(long chest_trajectory_frame_id)
   {
      chest_trajectory_frame_id_ = chest_trajectory_frame_id;
   }
   /**
            * Reference frame in which the controller should be tracking the pose for the chest as computed by the whole-body IK.
            * Default value is WORLD_FRAME, see FrameInformation for useful frame ids.
            */
   public long getChestTrajectoryFrameId()
   {
      return chest_trajectory_frame_id_;
   }

   /**
            * Reference frame in which the controller should be tracking the pose for the pelvis as computed by the whole-body IK.
            * Default value is WORLD_FRAME, see FrameInformation for useful frame ids.
            */
   public void setPelvisTrajectoryFrameId(long pelvis_trajectory_frame_id)
   {
      pelvis_trajectory_frame_id_ = pelvis_trajectory_frame_id;
   }
   /**
            * Reference frame in which the controller should be tracking the pose for the pelvis as computed by the whole-body IK.
            * Default value is WORLD_FRAME, see FrameInformation for useful frame ids.
            */
   public long getPelvisTrajectoryFrameId()
   {
      return pelvis_trajectory_frame_id_;
   }


   public static Supplier<KinematicsStreamingToolboxConfigurationMessagePubSubType> getPubSubType()
   {
      return KinematicsStreamingToolboxConfigurationMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return KinematicsStreamingToolboxConfigurationMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(KinematicsStreamingToolboxConfigurationMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.lock_pelvis_, other.lock_pelvis_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.lock_chest_, other.lock_chest_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.enable_left_arm_jointspace_, other.enable_left_arm_jointspace_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.enable_right_arm_jointspace_, other.enable_right_arm_jointspace_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.enable_neck_jointspace_, other.enable_neck_jointspace_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.enable_left_hand_taskspace_, other.enable_left_hand_taskspace_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.enable_right_hand_taskspace_, other.enable_right_hand_taskspace_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.enable_chest_taskspace_, other.enable_chest_taskspace_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.enable_pelvis_taskspace_, other.enable_pelvis_taskspace_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.left_hand_trajectory_frame_id_, other.left_hand_trajectory_frame_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.right_hand_trajectory_frame_id_, other.right_hand_trajectory_frame_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.chest_trajectory_frame_id_, other.chest_trajectory_frame_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.pelvis_trajectory_frame_id_, other.pelvis_trajectory_frame_id_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof KinematicsStreamingToolboxConfigurationMessage)) return false;

      KinematicsStreamingToolboxConfigurationMessage otherMyClass = (KinematicsStreamingToolboxConfigurationMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.lock_pelvis_ != otherMyClass.lock_pelvis_) return false;

      if(this.lock_chest_ != otherMyClass.lock_chest_) return false;

      if(this.enable_left_arm_jointspace_ != otherMyClass.enable_left_arm_jointspace_) return false;

      if(this.enable_right_arm_jointspace_ != otherMyClass.enable_right_arm_jointspace_) return false;

      if(this.enable_neck_jointspace_ != otherMyClass.enable_neck_jointspace_) return false;

      if(this.enable_left_hand_taskspace_ != otherMyClass.enable_left_hand_taskspace_) return false;

      if(this.enable_right_hand_taskspace_ != otherMyClass.enable_right_hand_taskspace_) return false;

      if(this.enable_chest_taskspace_ != otherMyClass.enable_chest_taskspace_) return false;

      if(this.enable_pelvis_taskspace_ != otherMyClass.enable_pelvis_taskspace_) return false;

      if(this.left_hand_trajectory_frame_id_ != otherMyClass.left_hand_trajectory_frame_id_) return false;

      if(this.right_hand_trajectory_frame_id_ != otherMyClass.right_hand_trajectory_frame_id_) return false;

      if(this.chest_trajectory_frame_id_ != otherMyClass.chest_trajectory_frame_id_) return false;

      if(this.pelvis_trajectory_frame_id_ != otherMyClass.pelvis_trajectory_frame_id_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("KinematicsStreamingToolboxConfigurationMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("lock_pelvis=");
      builder.append(this.lock_pelvis_);      builder.append(", ");
      builder.append("lock_chest=");
      builder.append(this.lock_chest_);      builder.append(", ");
      builder.append("enable_left_arm_jointspace=");
      builder.append(this.enable_left_arm_jointspace_);      builder.append(", ");
      builder.append("enable_right_arm_jointspace=");
      builder.append(this.enable_right_arm_jointspace_);      builder.append(", ");
      builder.append("enable_neck_jointspace=");
      builder.append(this.enable_neck_jointspace_);      builder.append(", ");
      builder.append("enable_left_hand_taskspace=");
      builder.append(this.enable_left_hand_taskspace_);      builder.append(", ");
      builder.append("enable_right_hand_taskspace=");
      builder.append(this.enable_right_hand_taskspace_);      builder.append(", ");
      builder.append("enable_chest_taskspace=");
      builder.append(this.enable_chest_taskspace_);      builder.append(", ");
      builder.append("enable_pelvis_taskspace=");
      builder.append(this.enable_pelvis_taskspace_);      builder.append(", ");
      builder.append("left_hand_trajectory_frame_id=");
      builder.append(this.left_hand_trajectory_frame_id_);      builder.append(", ");
      builder.append("right_hand_trajectory_frame_id=");
      builder.append(this.right_hand_trajectory_frame_id_);      builder.append(", ");
      builder.append("chest_trajectory_frame_id=");
      builder.append(this.chest_trajectory_frame_id_);      builder.append(", ");
      builder.append("pelvis_trajectory_frame_id=");
      builder.append(this.pelvis_trajectory_frame_id_);
      builder.append("}");
      return builder.toString();
   }
}
