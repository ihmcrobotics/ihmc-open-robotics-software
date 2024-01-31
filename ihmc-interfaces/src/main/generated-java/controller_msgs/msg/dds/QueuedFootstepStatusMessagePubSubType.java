package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "QueuedFootstepStatusMessage" defined in "QueuedFootstepStatusMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from QueuedFootstepStatusMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit QueuedFootstepStatusMessage_.idl instead.
*
*/
public class QueuedFootstepStatusMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.QueuedFootstepStatusMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::QueuedFootstepStatusMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "ab75e46819eae18287b738b8b36fdc8cd483fb1f430092376294f39193468d7e";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.QueuedFootstepStatusMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.QueuedFootstepStatusMessage data) throws java.io.IOException
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

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.QueuedFootstepStatusMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.QueuedFootstepStatusMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getLocation(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getOrientation(), current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.QueuedFootstepStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getRobotSide());

      geometry_msgs.msg.dds.PointPubSubType.write(data.getLocation(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getOrientation(), cdr);
      cdr.write_type_6(data.getSwingDuration());

      cdr.write_type_6(data.getTransferDuration());

   }

   public static void read(controller_msgs.msg.dds.QueuedFootstepStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setRobotSide(cdr.read_type_9());
      	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getLocation(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getOrientation(), cdr);	
      data.setSwingDuration(cdr.read_type_6());
      	
      data.setTransferDuration(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.QueuedFootstepStatusMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_a("location", new geometry_msgs.msg.dds.PointPubSubType(), data.getLocation());

      ser.write_type_a("orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getOrientation());

      ser.write_type_6("swing_duration", data.getSwingDuration());
      ser.write_type_6("transfer_duration", data.getTransferDuration());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.QueuedFootstepStatusMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setRobotSide(ser.read_type_9("robot_side"));
      ser.read_type_a("location", new geometry_msgs.msg.dds.PointPubSubType(), data.getLocation());

      ser.read_type_a("orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getOrientation());

      data.setSwingDuration(ser.read_type_6("swing_duration"));
      data.setTransferDuration(ser.read_type_6("transfer_duration"));
   }

   public static void staticCopy(controller_msgs.msg.dds.QueuedFootstepStatusMessage src, controller_msgs.msg.dds.QueuedFootstepStatusMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.QueuedFootstepStatusMessage createData()
   {
      return new controller_msgs.msg.dds.QueuedFootstepStatusMessage();
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
   
   public void serialize(controller_msgs.msg.dds.QueuedFootstepStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.QueuedFootstepStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.QueuedFootstepStatusMessage src, controller_msgs.msg.dds.QueuedFootstepStatusMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public QueuedFootstepStatusMessagePubSubType newInstance()
   {
      return new QueuedFootstepStatusMessagePubSubType();
   }
}
