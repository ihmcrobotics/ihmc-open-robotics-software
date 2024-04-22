package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "HandLoadBearingMessage" defined in "HandLoadBearingMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from HandLoadBearingMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit HandLoadBearingMessage_.idl instead.
*
*/
public class HandLoadBearingMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.HandLoadBearingMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::HandLoadBearingMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "9fc5fdaff0b5bc6cf704491ee99037e8c88c67ed0964fe9bb64a1a78b41be258";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.HandLoadBearingMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.HandLoadBearingMessage data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.HandLoadBearingMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.HandLoadBearingMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getContactPointInBodyFrame(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getContactNormalInWorld(), current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.HandLoadBearingMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getRobotSide());

      cdr.write_type_7(data.getLoad());

      cdr.write_type_6(data.getCoefficientOfFriction());

      geometry_msgs.msg.dds.PointPubSubType.write(data.getContactPointInBodyFrame(), cdr);
      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getContactNormalInWorld(), cdr);
      cdr.write_type_6(data.getExecutionDelayTime());

   }

   public static void read(controller_msgs.msg.dds.HandLoadBearingMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setRobotSide(cdr.read_type_9());
      	
      data.setLoad(cdr.read_type_7());
      	
      data.setCoefficientOfFriction(cdr.read_type_6());
      	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getContactPointInBodyFrame(), cdr);	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getContactNormalInWorld(), cdr);	
      data.setExecutionDelayTime(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.HandLoadBearingMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_7("load", data.getLoad());
      ser.write_type_6("coefficient_of_friction", data.getCoefficientOfFriction());
      ser.write_type_a("contact_point_in_body_frame", new geometry_msgs.msg.dds.PointPubSubType(), data.getContactPointInBodyFrame());

      ser.write_type_a("contact_normal_in_world", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getContactNormalInWorld());

      ser.write_type_6("execution_delay_time", data.getExecutionDelayTime());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.HandLoadBearingMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setRobotSide(ser.read_type_9("robot_side"));
      data.setLoad(ser.read_type_7("load"));
      data.setCoefficientOfFriction(ser.read_type_6("coefficient_of_friction"));
      ser.read_type_a("contact_point_in_body_frame", new geometry_msgs.msg.dds.PointPubSubType(), data.getContactPointInBodyFrame());

      ser.read_type_a("contact_normal_in_world", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getContactNormalInWorld());

      data.setExecutionDelayTime(ser.read_type_6("execution_delay_time"));
   }

   public static void staticCopy(controller_msgs.msg.dds.HandLoadBearingMessage src, controller_msgs.msg.dds.HandLoadBearingMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.HandLoadBearingMessage createData()
   {
      return new controller_msgs.msg.dds.HandLoadBearingMessage();
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
   
   public void serialize(controller_msgs.msg.dds.HandLoadBearingMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.HandLoadBearingMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.HandLoadBearingMessage src, controller_msgs.msg.dds.HandLoadBearingMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public HandLoadBearingMessagePubSubType newInstance()
   {
      return new HandLoadBearingMessagePubSubType();
   }
}
