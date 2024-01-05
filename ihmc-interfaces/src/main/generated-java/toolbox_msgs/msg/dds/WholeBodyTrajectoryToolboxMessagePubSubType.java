package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "WholeBodyTrajectoryToolboxMessage" defined in "WholeBodyTrajectoryToolboxMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from WholeBodyTrajectoryToolboxMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit WholeBodyTrajectoryToolboxMessage_.idl instead.
*
*/
public class WholeBodyTrajectoryToolboxMessagePubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.WholeBodyTrajectoryToolboxMessage>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::WholeBodyTrajectoryToolboxMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "0c63a0abd12fa5f45b0cc8bd5ff82a1929ce9f91401a9a907630117e4acdb53d";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.WholeBodyTrajectoryToolboxMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.WholeBodyTrajectoryToolboxMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += toolbox_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 10; ++i0)
      {
          current_alignment += toolbox_msgs.msg.dds.WaypointBasedTrajectoryMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 10; ++i0)
      {
          current_alignment += toolbox_msgs.msg.dds.RigidBodyExplorationConfigurationMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 10; ++i0)
      {
          current_alignment += toolbox_msgs.msg.dds.ReachingManifoldMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.WholeBodyTrajectoryToolboxMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.WholeBodyTrajectoryToolboxMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += toolbox_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessagePubSubType.getCdrSerializedSize(data.getConfiguration(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getEndEffectorTrajectories().size(); ++i0)
      {
          current_alignment += toolbox_msgs.msg.dds.WaypointBasedTrajectoryMessagePubSubType.getCdrSerializedSize(data.getEndEffectorTrajectories().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getExplorationConfigurations().size(); ++i0)
      {
          current_alignment += toolbox_msgs.msg.dds.RigidBodyExplorationConfigurationMessagePubSubType.getCdrSerializedSize(data.getExplorationConfigurations().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getReachingManifolds().size(); ++i0)
      {
          current_alignment += toolbox_msgs.msg.dds.ReachingManifoldMessagePubSubType.getCdrSerializedSize(data.getReachingManifolds().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.WholeBodyTrajectoryToolboxMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      toolbox_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessagePubSubType.write(data.getConfiguration(), cdr);
      if(data.getEndEffectorTrajectories().size() <= 10)
      cdr.write_type_e(data.getEndEffectorTrajectories());else
          throw new RuntimeException("end_effector_trajectories field exceeds the maximum length");

      if(data.getExplorationConfigurations().size() <= 10)
      cdr.write_type_e(data.getExplorationConfigurations());else
          throw new RuntimeException("exploration_configurations field exceeds the maximum length");

      if(data.getReachingManifolds().size() <= 10)
      cdr.write_type_e(data.getReachingManifolds());else
          throw new RuntimeException("reaching_manifolds field exceeds the maximum length");

   }

   public static void read(toolbox_msgs.msg.dds.WholeBodyTrajectoryToolboxMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      toolbox_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessagePubSubType.read(data.getConfiguration(), cdr);	
      cdr.read_type_e(data.getEndEffectorTrajectories());	
      cdr.read_type_e(data.getExplorationConfigurations());	
      cdr.read_type_e(data.getReachingManifolds());	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.WholeBodyTrajectoryToolboxMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_a("configuration", new toolbox_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessagePubSubType(), data.getConfiguration());

      ser.write_type_e("end_effector_trajectories", data.getEndEffectorTrajectories());
      ser.write_type_e("exploration_configurations", data.getExplorationConfigurations());
      ser.write_type_e("reaching_manifolds", data.getReachingManifolds());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.WholeBodyTrajectoryToolboxMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_a("configuration", new toolbox_msgs.msg.dds.WholeBodyTrajectoryToolboxConfigurationMessagePubSubType(), data.getConfiguration());

      ser.read_type_e("end_effector_trajectories", data.getEndEffectorTrajectories());
      ser.read_type_e("exploration_configurations", data.getExplorationConfigurations());
      ser.read_type_e("reaching_manifolds", data.getReachingManifolds());
   }

   public static void staticCopy(toolbox_msgs.msg.dds.WholeBodyTrajectoryToolboxMessage src, toolbox_msgs.msg.dds.WholeBodyTrajectoryToolboxMessage dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.WholeBodyTrajectoryToolboxMessage createData()
   {
      return new toolbox_msgs.msg.dds.WholeBodyTrajectoryToolboxMessage();
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
   
   public void serialize(toolbox_msgs.msg.dds.WholeBodyTrajectoryToolboxMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.WholeBodyTrajectoryToolboxMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.WholeBodyTrajectoryToolboxMessage src, toolbox_msgs.msg.dds.WholeBodyTrajectoryToolboxMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public WholeBodyTrajectoryToolboxMessagePubSubType newInstance()
   {
      return new WholeBodyTrajectoryToolboxMessagePubSubType();
   }
}
