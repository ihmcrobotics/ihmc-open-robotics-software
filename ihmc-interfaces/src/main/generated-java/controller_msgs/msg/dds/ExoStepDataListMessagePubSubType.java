package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ExoStepDataListMessage" defined in "ExoStepDataListMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ExoStepDataListMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ExoStepDataListMessage_.idl instead.
*
*/
public class ExoStepDataListMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.ExoStepDataListMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::ExoStepDataListMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.ExoStepDataListMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.ExoStepDataListMessage data) throws java.io.IOException
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


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 50; ++i0)
      {
          current_alignment += controller_msgs.msg.dds.ExoStepDataMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += controller_msgs.msg.dds.QueueableMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ExoStepDataListMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ExoStepDataListMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getStepDataList().size(); ++i0)
      {
          current_alignment += controller_msgs.msg.dds.ExoStepDataMessagePubSubType.getCdrSerializedSize(data.getStepDataList().get(i0), current_alignment);}


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      current_alignment += controller_msgs.msg.dds.QueueableMessagePubSubType.getCdrSerializedSize(data.getQueueingProperties(), current_alignment);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.ExoStepDataListMessage data, us.ihmc.idl.CDR cdr)
   {

      cdr.write_type_4(data.getSequenceId());


      if(data.getStepDataList().size() <= 50)
      cdr.write_type_e(data.getStepDataList());else
          throw new RuntimeException("step_data_list field exceeds the maximum length");


      cdr.write_type_6(data.getDefaultSwingDuration());


      cdr.write_type_6(data.getDefaultTransferDuration());


      controller_msgs.msg.dds.QueueableMessagePubSubType.write(data.getQueueingProperties(), cdr);

      cdr.write_type_9(data.getStepType());

   }

   public static void read(controller_msgs.msg.dds.ExoStepDataListMessage data, us.ihmc.idl.CDR cdr)
   {

      data.setSequenceId(cdr.read_type_4());
      	

      cdr.read_type_e(data.getStepDataList());	

      data.setDefaultSwingDuration(cdr.read_type_6());
      	

      data.setDefaultTransferDuration(cdr.read_type_6());
      	

      controller_msgs.msg.dds.QueueableMessagePubSubType.read(data.getQueueingProperties(), cdr);	

      data.setStepType(cdr.read_type_9());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.ExoStepDataListMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {

      ser.write_type_4("sequence_id", data.getSequenceId());

      ser.write_type_e("step_data_list", data.getStepDataList());

      ser.write_type_6("default_swing_duration", data.getDefaultSwingDuration());

      ser.write_type_6("default_transfer_duration", data.getDefaultTransferDuration());

      ser.write_type_a("queueing_properties", new controller_msgs.msg.dds.QueueableMessagePubSubType(), data.getQueueingProperties());


      ser.write_type_9("step_type", data.getStepType());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.ExoStepDataListMessage data)
   {

      data.setSequenceId(ser.read_type_4("sequence_id"));

      ser.read_type_e("step_data_list", data.getStepDataList());

      data.setDefaultSwingDuration(ser.read_type_6("default_swing_duration"));

      data.setDefaultTransferDuration(ser.read_type_6("default_transfer_duration"));

      ser.read_type_a("queueing_properties", new controller_msgs.msg.dds.QueueableMessagePubSubType(), data.getQueueingProperties());


      data.setStepType(ser.read_type_9("step_type"));
   }

   public static void staticCopy(controller_msgs.msg.dds.ExoStepDataListMessage src, controller_msgs.msg.dds.ExoStepDataListMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.ExoStepDataListMessage createData()
   {
      return new controller_msgs.msg.dds.ExoStepDataListMessage();
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
   
   public void serialize(controller_msgs.msg.dds.ExoStepDataListMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.ExoStepDataListMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.ExoStepDataListMessage src, controller_msgs.msg.dds.ExoStepDataListMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ExoStepDataListMessagePubSubType newInstance()
   {
      return new ExoStepDataListMessagePubSubType();
   }
}
