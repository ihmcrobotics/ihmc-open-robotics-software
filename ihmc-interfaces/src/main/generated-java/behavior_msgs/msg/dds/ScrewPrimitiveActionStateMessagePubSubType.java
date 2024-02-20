package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ScrewPrimitiveActionStateMessage" defined in "ScrewPrimitiveActionStateMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ScrewPrimitiveActionStateMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ScrewPrimitiveActionStateMessage_.idl instead.
*
*/
public class ScrewPrimitiveActionStateMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.ScrewPrimitiveActionStateMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::ScrewPrimitiveActionStateMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "93ca90e797fd4814231a88e842972c22672ba8a8d7be4b578dec36b53d04e05f";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.ScrewPrimitiveActionStateMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.ScrewPrimitiveActionStateMessage data) throws java.io.IOException
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

      current_alignment += behavior_msgs.msg.dds.ScrewPrimitiveActionDefinitionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 50; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += ((7) * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.ScrewPrimitiveActionStateMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.ScrewPrimitiveActionStateMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.getCdrSerializedSize(data.getState(), current_alignment);

      current_alignment += behavior_msgs.msg.dds.ScrewPrimitiveActionDefinitionMessagePubSubType.getCdrSerializedSize(data.getDefinition(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getPreviewTrajectory().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getPreviewTrajectory().get(i0), current_alignment);}

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getForce(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getTorque(), current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += ((7) * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);
      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.ScrewPrimitiveActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.write(data.getState(), cdr);
      behavior_msgs.msg.dds.ScrewPrimitiveActionDefinitionMessagePubSubType.write(data.getDefinition(), cdr);
      if(data.getPreviewTrajectory().size() <= 50)
      cdr.write_type_e(data.getPreviewTrajectory());else
          throw new RuntimeException("preview_trajectory field exceeds the maximum length");

      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getForce(), cdr);
      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getTorque(), cdr);
      cdr.write_type_6(data.getPreviewTrajectoryDuration());

      cdr.write_type_6(data.getPreviewTrajectoryLinearVelocity());

      cdr.write_type_6(data.getPreviewTrajectoryAngularVelocity());

      cdr.write_type_6(data.getPreviewRequestedTime());

      for(int i0 = 0; i0 < data.getPreviewJointAngles().length; ++i0)
      {
        	cdr.write_type_6(data.getPreviewJointAngles()[i0]);	
      }

      cdr.write_type_6(data.getPreviewSolutionQuality());

   }

   public static void read(behavior_msgs.msg.dds.ScrewPrimitiveActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.read(data.getState(), cdr);	
      behavior_msgs.msg.dds.ScrewPrimitiveActionDefinitionMessagePubSubType.read(data.getDefinition(), cdr);	
      cdr.read_type_e(data.getPreviewTrajectory());	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getForce(), cdr);	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getTorque(), cdr);	
      data.setPreviewTrajectoryDuration(cdr.read_type_6());
      	
      data.setPreviewTrajectoryLinearVelocity(cdr.read_type_6());
      	
      data.setPreviewTrajectoryAngularVelocity(cdr.read_type_6());
      	
      data.setPreviewRequestedTime(cdr.read_type_6());
      	
      for(int i0 = 0; i0 < data.getPreviewJointAngles().length; ++i0)
      {
        	data.getPreviewJointAngles()[i0] = cdr.read_type_6();
        	
      }
      	
      data.setPreviewSolutionQuality(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.ScrewPrimitiveActionStateMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("state", new behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType(), data.getState());

      ser.write_type_a("definition", new behavior_msgs.msg.dds.ScrewPrimitiveActionDefinitionMessagePubSubType(), data.getDefinition());

      ser.write_type_e("preview_trajectory", data.getPreviewTrajectory());
      ser.write_type_a("force", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getForce());

      ser.write_type_a("torque", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getTorque());

      ser.write_type_6("preview_trajectory_duration", data.getPreviewTrajectoryDuration());
      ser.write_type_6("preview_trajectory_linear_velocity", data.getPreviewTrajectoryLinearVelocity());
      ser.write_type_6("preview_trajectory_angular_velocity", data.getPreviewTrajectoryAngularVelocity());
      ser.write_type_6("preview_requested_time", data.getPreviewRequestedTime());
      ser.write_type_f("preview_joint_angles", data.getPreviewJointAngles());
      ser.write_type_6("preview_solution_quality", data.getPreviewSolutionQuality());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.ScrewPrimitiveActionStateMessage data)
   {
      ser.read_type_a("state", new behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType(), data.getState());

      ser.read_type_a("definition", new behavior_msgs.msg.dds.ScrewPrimitiveActionDefinitionMessagePubSubType(), data.getDefinition());

      ser.read_type_e("preview_trajectory", data.getPreviewTrajectory());
      ser.read_type_a("force", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getForce());

      ser.read_type_a("torque", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getTorque());

      data.setPreviewTrajectoryDuration(ser.read_type_6("preview_trajectory_duration"));
      data.setPreviewTrajectoryLinearVelocity(ser.read_type_6("preview_trajectory_linear_velocity"));
      data.setPreviewTrajectoryAngularVelocity(ser.read_type_6("preview_trajectory_angular_velocity"));
      data.setPreviewRequestedTime(ser.read_type_6("preview_requested_time"));
      ser.read_type_f("preview_joint_angles", data.getPreviewJointAngles());
      data.setPreviewSolutionQuality(ser.read_type_6("preview_solution_quality"));
   }

   public static void staticCopy(behavior_msgs.msg.dds.ScrewPrimitiveActionStateMessage src, behavior_msgs.msg.dds.ScrewPrimitiveActionStateMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.ScrewPrimitiveActionStateMessage createData()
   {
      return new behavior_msgs.msg.dds.ScrewPrimitiveActionStateMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.ScrewPrimitiveActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.ScrewPrimitiveActionStateMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.ScrewPrimitiveActionStateMessage src, behavior_msgs.msg.dds.ScrewPrimitiveActionStateMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ScrewPrimitiveActionStateMessagePubSubType newInstance()
   {
      return new ScrewPrimitiveActionStateMessagePubSubType();
   }
}
