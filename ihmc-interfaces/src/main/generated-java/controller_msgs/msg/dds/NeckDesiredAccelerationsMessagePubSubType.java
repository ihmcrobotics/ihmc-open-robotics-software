package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "NeckDesiredAccelerationsMessage" defined in "NeckDesiredAccelerationsMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from NeckDesiredAccelerationsMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit NeckDesiredAccelerationsMessage_.idl instead.
*
*/
public class NeckDesiredAccelerationsMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.NeckDesiredAccelerationsMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::NeckDesiredAccelerationsMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "86fb32abd5c055d67dc468caea60ef4f6e67a74166217cb335d90fcb2f1a3379";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.NeckDesiredAccelerationsMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.NeckDesiredAccelerationsMessage data) throws java.io.IOException
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

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.NeckDesiredAccelerationsMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.NeckDesiredAccelerationsMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += controller_msgs.msg.dds.DesiredAccelerationsMessagePubSubType.getCdrSerializedSize(data.getDesiredAccelerations(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.NeckDesiredAccelerationsMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      controller_msgs.msg.dds.DesiredAccelerationsMessagePubSubType.write(data.getDesiredAccelerations(), cdr);
   }

   public static void read(controller_msgs.msg.dds.NeckDesiredAccelerationsMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      controller_msgs.msg.dds.DesiredAccelerationsMessagePubSubType.read(data.getDesiredAccelerations(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.NeckDesiredAccelerationsMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_a("desired_accelerations", new controller_msgs.msg.dds.DesiredAccelerationsMessagePubSubType(), data.getDesiredAccelerations());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.NeckDesiredAccelerationsMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_a("desired_accelerations", new controller_msgs.msg.dds.DesiredAccelerationsMessagePubSubType(), data.getDesiredAccelerations());

   }

   public static void staticCopy(controller_msgs.msg.dds.NeckDesiredAccelerationsMessage src, controller_msgs.msg.dds.NeckDesiredAccelerationsMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.NeckDesiredAccelerationsMessage createData()
   {
      return new controller_msgs.msg.dds.NeckDesiredAccelerationsMessage();
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
   
   public void serialize(controller_msgs.msg.dds.NeckDesiredAccelerationsMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.NeckDesiredAccelerationsMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.NeckDesiredAccelerationsMessage src, controller_msgs.msg.dds.NeckDesiredAccelerationsMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public NeckDesiredAccelerationsMessagePubSubType newInstance()
   {
      return new NeckDesiredAccelerationsMessagePubSubType();
   }
}
