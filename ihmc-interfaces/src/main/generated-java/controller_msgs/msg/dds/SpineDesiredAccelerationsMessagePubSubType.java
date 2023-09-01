package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "SpineDesiredAccelerationsMessage" defined in "SpineDesiredAccelerationsMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from SpineDesiredAccelerationsMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit SpineDesiredAccelerationsMessage_.idl instead.
*
*/
public class SpineDesiredAccelerationsMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.SpineDesiredAccelerationsMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::SpineDesiredAccelerationsMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "450763f5ebef929b85ddc1655c903ef6ece4b1abd4a1ebdf3f4039bf73021ebc";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.SpineDesiredAccelerationsMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.SpineDesiredAccelerationsMessage data) throws java.io.IOException
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

      current_alignment += controller_msgs.msg.dds.DesiredAccelerationsMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.SpineDesiredAccelerationsMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.SpineDesiredAccelerationsMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += controller_msgs.msg.dds.DesiredAccelerationsMessagePubSubType.getCdrSerializedSize(data.getDesiredAccelerations(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.SpineDesiredAccelerationsMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      controller_msgs.msg.dds.DesiredAccelerationsMessagePubSubType.write(data.getDesiredAccelerations(), cdr);
   }

   public static void read(controller_msgs.msg.dds.SpineDesiredAccelerationsMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      controller_msgs.msg.dds.DesiredAccelerationsMessagePubSubType.read(data.getDesiredAccelerations(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.SpineDesiredAccelerationsMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_a("desired_accelerations", new controller_msgs.msg.dds.DesiredAccelerationsMessagePubSubType(), data.getDesiredAccelerations());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.SpineDesiredAccelerationsMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_a("desired_accelerations", new controller_msgs.msg.dds.DesiredAccelerationsMessagePubSubType(), data.getDesiredAccelerations());

   }

   public static void staticCopy(controller_msgs.msg.dds.SpineDesiredAccelerationsMessage src, controller_msgs.msg.dds.SpineDesiredAccelerationsMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.SpineDesiredAccelerationsMessage createData()
   {
      return new controller_msgs.msg.dds.SpineDesiredAccelerationsMessage();
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
   
   public void serialize(controller_msgs.msg.dds.SpineDesiredAccelerationsMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.SpineDesiredAccelerationsMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.SpineDesiredAccelerationsMessage src, controller_msgs.msg.dds.SpineDesiredAccelerationsMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public SpineDesiredAccelerationsMessagePubSubType newInstance()
   {
      return new SpineDesiredAccelerationsMessagePubSubType();
   }
}
