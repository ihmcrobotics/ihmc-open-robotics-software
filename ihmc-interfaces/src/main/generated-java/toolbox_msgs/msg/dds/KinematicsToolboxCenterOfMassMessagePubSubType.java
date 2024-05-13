package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "KinematicsToolboxCenterOfMassMessage" defined in "KinematicsToolboxCenterOfMassMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from KinematicsToolboxCenterOfMassMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit KinematicsToolboxCenterOfMassMessage_.idl instead.
*
*/
public class KinematicsToolboxCenterOfMassMessagePubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::KinematicsToolboxCenterOfMassMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "736062f0129adf531ab61e6bb8773f2ff95083022cb01b7cc5b7be8b4e956044";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage data) throws java.io.IOException
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

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += ihmc_common_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += ihmc_common_msgs.msg.dds.WeightMatrix3DMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getDesiredPositionInWorld(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getLinearVelocityInWorld(), current_alignment);

      current_alignment += ihmc_common_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.getCdrSerializedSize(data.getSelectionMatrix(), current_alignment);

      current_alignment += ihmc_common_msgs.msg.dds.WeightMatrix3DMessagePubSubType.getCdrSerializedSize(data.getWeights(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      geometry_msgs.msg.dds.PointPubSubType.write(data.getDesiredPositionInWorld(), cdr);
      cdr.write_type_7(data.getHasLinearVelocity());

      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getLinearVelocityInWorld(), cdr);
      ihmc_common_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.write(data.getSelectionMatrix(), cdr);
      ihmc_common_msgs.msg.dds.WeightMatrix3DMessagePubSubType.write(data.getWeights(), cdr);
   }

   public static void read(toolbox_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getDesiredPositionInWorld(), cdr);	
      data.setHasLinearVelocity(cdr.read_type_7());
      	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getLinearVelocityInWorld(), cdr);	
      ihmc_common_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.read(data.getSelectionMatrix(), cdr);	
      ihmc_common_msgs.msg.dds.WeightMatrix3DMessagePubSubType.read(data.getWeights(), cdr);	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_a("desired_position_in_world", new geometry_msgs.msg.dds.PointPubSubType(), data.getDesiredPositionInWorld());

      ser.write_type_7("has_linear_velocity", data.getHasLinearVelocity());
      ser.write_type_a("linear_velocity_in_world", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getLinearVelocityInWorld());

      ser.write_type_a("selection_matrix", new ihmc_common_msgs.msg.dds.SelectionMatrix3DMessagePubSubType(), data.getSelectionMatrix());

      ser.write_type_a("weights", new ihmc_common_msgs.msg.dds.WeightMatrix3DMessagePubSubType(), data.getWeights());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_a("desired_position_in_world", new geometry_msgs.msg.dds.PointPubSubType(), data.getDesiredPositionInWorld());

      data.setHasLinearVelocity(ser.read_type_7("has_linear_velocity"));
      ser.read_type_a("linear_velocity_in_world", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getLinearVelocityInWorld());

      ser.read_type_a("selection_matrix", new ihmc_common_msgs.msg.dds.SelectionMatrix3DMessagePubSubType(), data.getSelectionMatrix());

      ser.read_type_a("weights", new ihmc_common_msgs.msg.dds.WeightMatrix3DMessagePubSubType(), data.getWeights());

   }

   public static void staticCopy(toolbox_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage src, toolbox_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage createData()
   {
      return new toolbox_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage();
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
   
   public void serialize(toolbox_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage src, toolbox_msgs.msg.dds.KinematicsToolboxCenterOfMassMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public KinematicsToolboxCenterOfMassMessagePubSubType newInstance()
   {
      return new KinematicsToolboxCenterOfMassMessagePubSubType();
   }
}
