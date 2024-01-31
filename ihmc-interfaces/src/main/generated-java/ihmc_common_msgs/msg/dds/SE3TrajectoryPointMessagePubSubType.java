package ihmc_common_msgs.msg.dds;

/**
* 
* Topic data type of the struct "SE3TrajectoryPointMessage" defined in "SE3TrajectoryPointMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from SE3TrajectoryPointMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit SE3TrajectoryPointMessage_.idl instead.
*
*/
public class SE3TrajectoryPointMessagePubSubType implements us.ihmc.pubsub.TopicDataType<ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage>
{
   public static final java.lang.String name = "ihmc_common_msgs::msg::dds_::SE3TrajectoryPointMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "3fa8890a208b211c41656118a300063fc0a924d06e409ed8149fbcbaaac39784";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage data) throws java.io.IOException
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

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getPosition(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getOrientation(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getLinearVelocity(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getAngularVelocity(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_6(data.getTime());

      geometry_msgs.msg.dds.PointPubSubType.write(data.getPosition(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getOrientation(), cdr);
      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getLinearVelocity(), cdr);
      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getAngularVelocity(), cdr);
   }

   public static void read(ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setTime(cdr.read_type_6());
      	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getPosition(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getOrientation(), cdr);	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getLinearVelocity(), cdr);	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getAngularVelocity(), cdr);	

   }

   @Override
   public final void serialize(ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_6("time", data.getTime());
      ser.write_type_a("position", new geometry_msgs.msg.dds.PointPubSubType(), data.getPosition());

      ser.write_type_a("orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getOrientation());

      ser.write_type_a("linear_velocity", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getLinearVelocity());

      ser.write_type_a("angular_velocity", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getAngularVelocity());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setTime(ser.read_type_6("time"));
      ser.read_type_a("position", new geometry_msgs.msg.dds.PointPubSubType(), data.getPosition());

      ser.read_type_a("orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getOrientation());

      ser.read_type_a("linear_velocity", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getLinearVelocity());

      ser.read_type_a("angular_velocity", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getAngularVelocity());

   }

   public static void staticCopy(ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage src, ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage dest)
   {
      dest.set(src);
   }

   @Override
   public ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage createData()
   {
      return new ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage();
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
   
   public void serialize(ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage src, ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public SE3TrajectoryPointMessagePubSubType newInstance()
   {
      return new SE3TrajectoryPointMessagePubSubType();
   }
}
