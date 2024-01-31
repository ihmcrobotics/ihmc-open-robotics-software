package ihmc_common_msgs.msg.dds;

/**
* 
* Topic data type of the struct "EuclideanTrajectoryPointMessage" defined in "EuclideanTrajectoryPointMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from EuclideanTrajectoryPointMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit EuclideanTrajectoryPointMessage_.idl instead.
*
*/
public class EuclideanTrajectoryPointMessagePubSubType implements us.ihmc.pubsub.TopicDataType<ihmc_common_msgs.msg.dds.EuclideanTrajectoryPointMessage>
{
   public static final java.lang.String name = "ihmc_common_msgs::msg::dds_::EuclideanTrajectoryPointMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "b3585feaf39ab81154b98086cd2909da929f19a2c01a7a243d197689f08c52ff";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(ihmc_common_msgs.msg.dds.EuclideanTrajectoryPointMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, ihmc_common_msgs.msg.dds.EuclideanTrajectoryPointMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.EuclideanTrajectoryPointMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.EuclideanTrajectoryPointMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getPosition(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getLinearVelocity(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(ihmc_common_msgs.msg.dds.EuclideanTrajectoryPointMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_6(data.getTime());

      geometry_msgs.msg.dds.PointPubSubType.write(data.getPosition(), cdr);
      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getLinearVelocity(), cdr);
   }

   public static void read(ihmc_common_msgs.msg.dds.EuclideanTrajectoryPointMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setTime(cdr.read_type_6());
      	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getPosition(), cdr);	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getLinearVelocity(), cdr);	

   }

   @Override
   public final void serialize(ihmc_common_msgs.msg.dds.EuclideanTrajectoryPointMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_6("time", data.getTime());
      ser.write_type_a("position", new geometry_msgs.msg.dds.PointPubSubType(), data.getPosition());

      ser.write_type_a("linear_velocity", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getLinearVelocity());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, ihmc_common_msgs.msg.dds.EuclideanTrajectoryPointMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setTime(ser.read_type_6("time"));
      ser.read_type_a("position", new geometry_msgs.msg.dds.PointPubSubType(), data.getPosition());

      ser.read_type_a("linear_velocity", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getLinearVelocity());

   }

   public static void staticCopy(ihmc_common_msgs.msg.dds.EuclideanTrajectoryPointMessage src, ihmc_common_msgs.msg.dds.EuclideanTrajectoryPointMessage dest)
   {
      dest.set(src);
   }

   @Override
   public ihmc_common_msgs.msg.dds.EuclideanTrajectoryPointMessage createData()
   {
      return new ihmc_common_msgs.msg.dds.EuclideanTrajectoryPointMessage();
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
   
   public void serialize(ihmc_common_msgs.msg.dds.EuclideanTrajectoryPointMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(ihmc_common_msgs.msg.dds.EuclideanTrajectoryPointMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(ihmc_common_msgs.msg.dds.EuclideanTrajectoryPointMessage src, ihmc_common_msgs.msg.dds.EuclideanTrajectoryPointMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public EuclideanTrajectoryPointMessagePubSubType newInstance()
   {
      return new EuclideanTrajectoryPointMessagePubSubType();
   }
}
