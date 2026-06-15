package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ArmActionStateMessage" defined in "ArmActionStateMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ArmActionStateMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ArmActionStateMessage_.idl instead.
*
*/
public class ArmActionStateMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.ArmActionStateMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::ArmActionStateMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "03536c09dfec4ae17d6b4f18dfd4c7edbf014b0abcd727264c3783861fc1d026";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.ArmActionStateMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.ArmActionStateMessage data) throws java.io.IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      read(data, deserializeCDR);
      deserializeCDR.finishDeserialize();
   }

   public static int getMaxCdrSerializedSize()
   {
      return getMaxCdrSerializedSize(0);
   }

   public static int getMaxCdrSerializedSize(int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += behavior_msgs.msg.dds.ArmActionDefinitionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += ((7) * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 50; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += ((7) * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.ArmActionStateMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.ArmActionStateMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.getCdrSerializedSize(data.getState(), current_alignment);

      current_alignment += behavior_msgs.msg.dds.ArmActionDefinitionMessagePubSubType.getCdrSerializedSize(data.getDefinition(), current_alignment);

      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getCdrSerializedSize(data.getGoalChestTransformToWorld(), current_alignment);

      current_alignment += ((7) * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);
      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getPreviewTrajectory().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getPreviewTrajectory().get(i0), current_alignment);}

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += ((7) * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);
      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getForce(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getTorque(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.ArmActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.write(data.getState(), cdr);
      behavior_msgs.msg.dds.ArmActionDefinitionMessagePubSubType.write(data.getDefinition(), cdr);
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.write(data.getGoalChestTransformToWorld(), cdr);
      for(int i0 = 0; i0 < data.getJointAngles().length; ++i0)
      {
        	cdr.write_type_6(data.getJointAngles()[i0]);	
      }

      cdr.write_type_6(data.getSolutionQuality());

      if(data.getPreviewTrajectory().size() <= 50)
      cdr.write_type_e(data.getPreviewTrajectory());else
          throw new RuntimeException("preview_trajectory field exceeds the maximum length: %d > %d".formatted(data.getPreviewTrajectory().size(), 50));

      cdr.write_type_6(data.getPreviewTrajectoryDuration());

      cdr.write_type_6(data.getPreviewTrajectoryLinearVelocity());

      cdr.write_type_6(data.getPreviewTrajectoryAngularVelocity());

      cdr.write_type_6(data.getPreviewRequestedTime());

      for(int i0 = 0; i0 < data.getPreviewJointAngles().length; ++i0)
      {
        	cdr.write_type_6(data.getPreviewJointAngles()[i0]);	
      }

      cdr.write_type_6(data.getPreviewSolutionQuality());

      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getForce(), cdr);
      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getTorque(), cdr);
   }

   public static void read(behavior_msgs.msg.dds.ArmActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.read(data.getState(), cdr);	
      behavior_msgs.msg.dds.ArmActionDefinitionMessagePubSubType.read(data.getDefinition(), cdr);	
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.read(data.getGoalChestTransformToWorld(), cdr);	
      for(int i0 = 0; i0 < data.getJointAngles().length; ++i0)
      {
        	data.getJointAngles()[i0] = cdr.read_type_6();
        	
      }
      	
      data.setSolutionQuality(cdr.read_type_6());
      	
      cdr.read_type_e(data.getPreviewTrajectory());	
      data.setPreviewTrajectoryDuration(cdr.read_type_6());
      	
      data.setPreviewTrajectoryLinearVelocity(cdr.read_type_6());
      	
      data.setPreviewTrajectoryAngularVelocity(cdr.read_type_6());
      	
      data.setPreviewRequestedTime(cdr.read_type_6());
      	
      for(int i0 = 0; i0 < data.getPreviewJointAngles().length; ++i0)
      {
        	data.getPreviewJointAngles()[i0] = cdr.read_type_6();
        	
      }
      	
      data.setPreviewSolutionQuality(cdr.read_type_6());
      	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getForce(), cdr);	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getTorque(), cdr);	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.ArmActionStateMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("state", new behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType(), data.getState());

      ser.write_type_a("definition", new behavior_msgs.msg.dds.ArmActionDefinitionMessagePubSubType(), data.getDefinition());

      ser.write_type_a("goal_chest_transform_to_world", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getGoalChestTransformToWorld());

      ser.write_type_f("joint_angles", data.getJointAngles());
      ser.write_type_6("solution_quality", data.getSolutionQuality());
      ser.write_type_e("preview_trajectory", data.getPreviewTrajectory());
      ser.write_type_6("preview_trajectory_duration", data.getPreviewTrajectoryDuration());
      ser.write_type_6("preview_trajectory_linear_velocity", data.getPreviewTrajectoryLinearVelocity());
      ser.write_type_6("preview_trajectory_angular_velocity", data.getPreviewTrajectoryAngularVelocity());
      ser.write_type_6("preview_requested_time", data.getPreviewRequestedTime());
      ser.write_type_f("preview_joint_angles", data.getPreviewJointAngles());
      ser.write_type_6("preview_solution_quality", data.getPreviewSolutionQuality());
      ser.write_type_a("force", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getForce());

      ser.write_type_a("torque", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getTorque());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.ArmActionStateMessage data)
   {
      ser.read_type_a("state", new behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType(), data.getState());

      ser.read_type_a("definition", new behavior_msgs.msg.dds.ArmActionDefinitionMessagePubSubType(), data.getDefinition());

      ser.read_type_a("goal_chest_transform_to_world", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getGoalChestTransformToWorld());

      ser.read_type_f("joint_angles", data.getJointAngles());
      data.setSolutionQuality(ser.read_type_6("solution_quality"));
      ser.read_type_e("preview_trajectory", data.getPreviewTrajectory());
      data.setPreviewTrajectoryDuration(ser.read_type_6("preview_trajectory_duration"));
      data.setPreviewTrajectoryLinearVelocity(ser.read_type_6("preview_trajectory_linear_velocity"));
      data.setPreviewTrajectoryAngularVelocity(ser.read_type_6("preview_trajectory_angular_velocity"));
      data.setPreviewRequestedTime(ser.read_type_6("preview_requested_time"));
      ser.read_type_f("preview_joint_angles", data.getPreviewJointAngles());
      data.setPreviewSolutionQuality(ser.read_type_6("preview_solution_quality"));
      ser.read_type_a("force", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getForce());

      ser.read_type_a("torque", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getTorque());

   }

   public static void staticCopy(behavior_msgs.msg.dds.ArmActionStateMessage src, behavior_msgs.msg.dds.ArmActionStateMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.ArmActionStateMessage createData()
   {
      return new behavior_msgs.msg.dds.ArmActionStateMessage();
   }
   @Override
   public int getTypeSize()
   {
      return us.ihmc.idl.CDR.getTypeSize(getMaxCdrSerializedSize());
   }

   @Override
   public java.lang.String getName()
   {
      return name;
   }
   
   public void serialize(behavior_msgs.msg.dds.ArmActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.ArmActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.ArmActionStateMessage src, behavior_msgs.msg.dds.ArmActionStateMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ArmActionStateMessagePubSubType newInstance()
   {
      return new ArmActionStateMessagePubSubType();
   }
}
