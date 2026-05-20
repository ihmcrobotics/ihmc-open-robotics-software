package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class AI2RNodeDefinitionMessage extends Packet<AI2RNodeDefinitionMessage> implements Settable<AI2RNodeDefinitionMessage>, EpsilonComparable<AI2RNodeDefinitionMessage>
{
   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage definition_;
   /**
            * Randomized Go-To action controls
            */
   public boolean randomize_go_to_action_;
   public boolean randomize_whole_body_action_;
   public int number_of_randomizations_;
   /**
            * Whole body randomization trigger and parameters
            */
   public int whole_body_randomization_request_id_;
   public controller_msgs.msg.dds.RigidBodyTransformMessage right_hand_transform_to_chest_;
   public controller_msgs.msg.dds.RigidBodyTransformMessage left_hand_transform_to_chest_;
   public double spine_pitch_degrees_;
   public double spine_roll_degrees_;
   public double spine_yaw_degrees_;
   public controller_msgs.msg.dds.RigidBodyTransformMessage pelvis_transform_to_parent_;
   public double right_hand_trajectory_duration_;
   public double left_hand_trajectory_duration_;
   public double spine_trajectory_duration_;
   public double pelvis_trajectory_duration_;
   public int left_hand_mask_;
   public int right_hand_mask_;
   public int spine_mask_;
   public int pelvis_mask_;
   public double probability_one_enabled_;
   public double probability_two_enabled_;
   public double probability_three_enabled_;
   public double probability_four_enabled_;

   public AI2RNodeDefinitionMessage()
   {
      definition_ = new behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage();
      right_hand_transform_to_chest_ = new controller_msgs.msg.dds.RigidBodyTransformMessage();
      left_hand_transform_to_chest_ = new controller_msgs.msg.dds.RigidBodyTransformMessage();
      pelvis_transform_to_parent_ = new controller_msgs.msg.dds.RigidBodyTransformMessage();
   }

   public AI2RNodeDefinitionMessage(AI2RNodeDefinitionMessage other)
   {
      this();
      set(other);
   }

   public void set(AI2RNodeDefinitionMessage other)
   {
      behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      randomize_go_to_action_ = other.randomize_go_to_action_;

      randomize_whole_body_action_ = other.randomize_whole_body_action_;

      number_of_randomizations_ = other.number_of_randomizations_;

      whole_body_randomization_request_id_ = other.whole_body_randomization_request_id_;

      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.staticCopy(other.right_hand_transform_to_chest_, right_hand_transform_to_chest_);
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.staticCopy(other.left_hand_transform_to_chest_, left_hand_transform_to_chest_);
      spine_pitch_degrees_ = other.spine_pitch_degrees_;

      spine_roll_degrees_ = other.spine_roll_degrees_;

      spine_yaw_degrees_ = other.spine_yaw_degrees_;

      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.staticCopy(other.pelvis_transform_to_parent_, pelvis_transform_to_parent_);
      right_hand_trajectory_duration_ = other.right_hand_trajectory_duration_;

      left_hand_trajectory_duration_ = other.left_hand_trajectory_duration_;

      spine_trajectory_duration_ = other.spine_trajectory_duration_;

      pelvis_trajectory_duration_ = other.pelvis_trajectory_duration_;

      left_hand_mask_ = other.left_hand_mask_;

      right_hand_mask_ = other.right_hand_mask_;

      spine_mask_ = other.spine_mask_;

      pelvis_mask_ = other.pelvis_mask_;

      probability_one_enabled_ = other.probability_one_enabled_;

      probability_two_enabled_ = other.probability_two_enabled_;

      probability_three_enabled_ = other.probability_three_enabled_;

      probability_four_enabled_ = other.probability_four_enabled_;

   }


   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage getDefinition()
   {
      return definition_;
   }

   /**
            * Randomized Go-To action controls
            */
   public void setRandomizeGoToAction(boolean randomize_go_to_action)
   {
      randomize_go_to_action_ = randomize_go_to_action;
   }
   /**
            * Randomized Go-To action controls
            */
   public boolean getRandomizeGoToAction()
   {
      return randomize_go_to_action_;
   }

   public void setRandomizeWholeBodyAction(boolean randomize_whole_body_action)
   {
      randomize_whole_body_action_ = randomize_whole_body_action;
   }
   public boolean getRandomizeWholeBodyAction()
   {
      return randomize_whole_body_action_;
   }

   public void setNumberOfRandomizations(int number_of_randomizations)
   {
      number_of_randomizations_ = number_of_randomizations;
   }
   public int getNumberOfRandomizations()
   {
      return number_of_randomizations_;
   }

   /**
            * Whole body randomization trigger and parameters
            */
   public void setWholeBodyRandomizationRequestId(int whole_body_randomization_request_id)
   {
      whole_body_randomization_request_id_ = whole_body_randomization_request_id;
   }
   /**
            * Whole body randomization trigger and parameters
            */
   public int getWholeBodyRandomizationRequestId()
   {
      return whole_body_randomization_request_id_;
   }


   public controller_msgs.msg.dds.RigidBodyTransformMessage getRightHandTransformToChest()
   {
      return right_hand_transform_to_chest_;
   }


   public controller_msgs.msg.dds.RigidBodyTransformMessage getLeftHandTransformToChest()
   {
      return left_hand_transform_to_chest_;
   }

   public void setSpinePitchDegrees(double spine_pitch_degrees)
   {
      spine_pitch_degrees_ = spine_pitch_degrees;
   }
   public double getSpinePitchDegrees()
   {
      return spine_pitch_degrees_;
   }

   public void setSpineRollDegrees(double spine_roll_degrees)
   {
      spine_roll_degrees_ = spine_roll_degrees;
   }
   public double getSpineRollDegrees()
   {
      return spine_roll_degrees_;
   }

   public void setSpineYawDegrees(double spine_yaw_degrees)
   {
      spine_yaw_degrees_ = spine_yaw_degrees;
   }
   public double getSpineYawDegrees()
   {
      return spine_yaw_degrees_;
   }


   public controller_msgs.msg.dds.RigidBodyTransformMessage getPelvisTransformToParent()
   {
      return pelvis_transform_to_parent_;
   }

   public void setRightHandTrajectoryDuration(double right_hand_trajectory_duration)
   {
      right_hand_trajectory_duration_ = right_hand_trajectory_duration;
   }
   public double getRightHandTrajectoryDuration()
   {
      return right_hand_trajectory_duration_;
   }

   public void setLeftHandTrajectoryDuration(double left_hand_trajectory_duration)
   {
      left_hand_trajectory_duration_ = left_hand_trajectory_duration;
   }
   public double getLeftHandTrajectoryDuration()
   {
      return left_hand_trajectory_duration_;
   }

   public void setSpineTrajectoryDuration(double spine_trajectory_duration)
   {
      spine_trajectory_duration_ = spine_trajectory_duration;
   }
   public double getSpineTrajectoryDuration()
   {
      return spine_trajectory_duration_;
   }

   public void setPelvisTrajectoryDuration(double pelvis_trajectory_duration)
   {
      pelvis_trajectory_duration_ = pelvis_trajectory_duration;
   }
   public double getPelvisTrajectoryDuration()
   {
      return pelvis_trajectory_duration_;
   }

   public void setLeftHandMask(int left_hand_mask)
   {
      left_hand_mask_ = left_hand_mask;
   }
   public int getLeftHandMask()
   {
      return left_hand_mask_;
   }

   public void setRightHandMask(int right_hand_mask)
   {
      right_hand_mask_ = right_hand_mask;
   }
   public int getRightHandMask()
   {
      return right_hand_mask_;
   }

   public void setSpineMask(int spine_mask)
   {
      spine_mask_ = spine_mask;
   }
   public int getSpineMask()
   {
      return spine_mask_;
   }

   public void setPelvisMask(int pelvis_mask)
   {
      pelvis_mask_ = pelvis_mask;
   }
   public int getPelvisMask()
   {
      return pelvis_mask_;
   }

   public void setProbabilityOneEnabled(double probability_one_enabled)
   {
      probability_one_enabled_ = probability_one_enabled;
   }
   public double getProbabilityOneEnabled()
   {
      return probability_one_enabled_;
   }

   public void setProbabilityTwoEnabled(double probability_two_enabled)
   {
      probability_two_enabled_ = probability_two_enabled;
   }
   public double getProbabilityTwoEnabled()
   {
      return probability_two_enabled_;
   }

   public void setProbabilityThreeEnabled(double probability_three_enabled)
   {
      probability_three_enabled_ = probability_three_enabled;
   }
   public double getProbabilityThreeEnabled()
   {
      return probability_three_enabled_;
   }

   public void setProbabilityFourEnabled(double probability_four_enabled)
   {
      probability_four_enabled_ = probability_four_enabled;
   }
   public double getProbabilityFourEnabled()
   {
      return probability_four_enabled_;
   }


   public static Supplier<AI2RNodeDefinitionMessagePubSubType> getPubSubType()
   {
      return AI2RNodeDefinitionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return AI2RNodeDefinitionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(AI2RNodeDefinitionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.randomize_go_to_action_, other.randomize_go_to_action_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.randomize_whole_body_action_, other.randomize_whole_body_action_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.number_of_randomizations_, other.number_of_randomizations_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.whole_body_randomization_request_id_, other.whole_body_randomization_request_id_, epsilon)) return false;

      if (!this.right_hand_transform_to_chest_.epsilonEquals(other.right_hand_transform_to_chest_, epsilon)) return false;
      if (!this.left_hand_transform_to_chest_.epsilonEquals(other.left_hand_transform_to_chest_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.spine_pitch_degrees_, other.spine_pitch_degrees_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.spine_roll_degrees_, other.spine_roll_degrees_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.spine_yaw_degrees_, other.spine_yaw_degrees_, epsilon)) return false;

      if (!this.pelvis_transform_to_parent_.epsilonEquals(other.pelvis_transform_to_parent_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.right_hand_trajectory_duration_, other.right_hand_trajectory_duration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.left_hand_trajectory_duration_, other.left_hand_trajectory_duration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.spine_trajectory_duration_, other.spine_trajectory_duration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.pelvis_trajectory_duration_, other.pelvis_trajectory_duration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.left_hand_mask_, other.left_hand_mask_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.right_hand_mask_, other.right_hand_mask_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.spine_mask_, other.spine_mask_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.pelvis_mask_, other.pelvis_mask_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.probability_one_enabled_, other.probability_one_enabled_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.probability_two_enabled_, other.probability_two_enabled_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.probability_three_enabled_, other.probability_three_enabled_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.probability_four_enabled_, other.probability_four_enabled_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof AI2RNodeDefinitionMessage)) return false;

      AI2RNodeDefinitionMessage otherMyClass = (AI2RNodeDefinitionMessage) other;

      if (!this.definition_.equals(otherMyClass.definition_)) return false;
      if(this.randomize_go_to_action_ != otherMyClass.randomize_go_to_action_) return false;

      if(this.randomize_whole_body_action_ != otherMyClass.randomize_whole_body_action_) return false;

      if(this.number_of_randomizations_ != otherMyClass.number_of_randomizations_) return false;

      if(this.whole_body_randomization_request_id_ != otherMyClass.whole_body_randomization_request_id_) return false;

      if (!this.right_hand_transform_to_chest_.equals(otherMyClass.right_hand_transform_to_chest_)) return false;
      if (!this.left_hand_transform_to_chest_.equals(otherMyClass.left_hand_transform_to_chest_)) return false;
      if(this.spine_pitch_degrees_ != otherMyClass.spine_pitch_degrees_) return false;

      if(this.spine_roll_degrees_ != otherMyClass.spine_roll_degrees_) return false;

      if(this.spine_yaw_degrees_ != otherMyClass.spine_yaw_degrees_) return false;

      if (!this.pelvis_transform_to_parent_.equals(otherMyClass.pelvis_transform_to_parent_)) return false;
      if(this.right_hand_trajectory_duration_ != otherMyClass.right_hand_trajectory_duration_) return false;

      if(this.left_hand_trajectory_duration_ != otherMyClass.left_hand_trajectory_duration_) return false;

      if(this.spine_trajectory_duration_ != otherMyClass.spine_trajectory_duration_) return false;

      if(this.pelvis_trajectory_duration_ != otherMyClass.pelvis_trajectory_duration_) return false;

      if(this.left_hand_mask_ != otherMyClass.left_hand_mask_) return false;

      if(this.right_hand_mask_ != otherMyClass.right_hand_mask_) return false;

      if(this.spine_mask_ != otherMyClass.spine_mask_) return false;

      if(this.pelvis_mask_ != otherMyClass.pelvis_mask_) return false;

      if(this.probability_one_enabled_ != otherMyClass.probability_one_enabled_) return false;

      if(this.probability_two_enabled_ != otherMyClass.probability_two_enabled_) return false;

      if(this.probability_three_enabled_ != otherMyClass.probability_three_enabled_) return false;

      if(this.probability_four_enabled_ != otherMyClass.probability_four_enabled_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AI2RNodeDefinitionMessage {");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("randomize_go_to_action=");
      builder.append(this.randomize_go_to_action_);      builder.append(", ");
      builder.append("randomize_whole_body_action=");
      builder.append(this.randomize_whole_body_action_);      builder.append(", ");
      builder.append("number_of_randomizations=");
      builder.append(this.number_of_randomizations_);      builder.append(", ");
      builder.append("whole_body_randomization_request_id=");
      builder.append(this.whole_body_randomization_request_id_);      builder.append(", ");
      builder.append("right_hand_transform_to_chest=");
      builder.append(this.right_hand_transform_to_chest_);      builder.append(", ");
      builder.append("left_hand_transform_to_chest=");
      builder.append(this.left_hand_transform_to_chest_);      builder.append(", ");
      builder.append("spine_pitch_degrees=");
      builder.append(this.spine_pitch_degrees_);      builder.append(", ");
      builder.append("spine_roll_degrees=");
      builder.append(this.spine_roll_degrees_);      builder.append(", ");
      builder.append("spine_yaw_degrees=");
      builder.append(this.spine_yaw_degrees_);      builder.append(", ");
      builder.append("pelvis_transform_to_parent=");
      builder.append(this.pelvis_transform_to_parent_);      builder.append(", ");
      builder.append("right_hand_trajectory_duration=");
      builder.append(this.right_hand_trajectory_duration_);      builder.append(", ");
      builder.append("left_hand_trajectory_duration=");
      builder.append(this.left_hand_trajectory_duration_);      builder.append(", ");
      builder.append("spine_trajectory_duration=");
      builder.append(this.spine_trajectory_duration_);      builder.append(", ");
      builder.append("pelvis_trajectory_duration=");
      builder.append(this.pelvis_trajectory_duration_);      builder.append(", ");
      builder.append("left_hand_mask=");
      builder.append(this.left_hand_mask_);      builder.append(", ");
      builder.append("right_hand_mask=");
      builder.append(this.right_hand_mask_);      builder.append(", ");
      builder.append("spine_mask=");
      builder.append(this.spine_mask_);      builder.append(", ");
      builder.append("pelvis_mask=");
      builder.append(this.pelvis_mask_);      builder.append(", ");
      builder.append("probability_one_enabled=");
      builder.append(this.probability_one_enabled_);      builder.append(", ");
      builder.append("probability_two_enabled=");
      builder.append(this.probability_two_enabled_);      builder.append(", ");
      builder.append("probability_three_enabled=");
      builder.append(this.probability_three_enabled_);      builder.append(", ");
      builder.append("probability_four_enabled=");
      builder.append(this.probability_four_enabled_);
      builder.append("}");
      return builder.toString();
   }
}
