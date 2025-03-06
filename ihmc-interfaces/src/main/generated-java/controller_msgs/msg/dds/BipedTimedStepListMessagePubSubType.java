package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "BipedTimedStepListMessage" defined in "BipedTimedStepListMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from BipedTimedStepListMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit BipedTimedStepListMessage_.idl instead.
*
*/
public class BipedTimedStepListMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.BipedTimedStepListMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::BipedTimedStepListMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "a6027e4a2a214f3d5cc739890d603ceb8b63e72230f6b5caeb766b714cd3a5c5";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.BipedTimedStepListMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.BipedTimedStepListMessage data) throws java.io.IOException
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
          current_alignment += controller_msgs.msg.dds.BipedTimedStepMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.BipedTimedStepListMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.BipedTimedStepListMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getTimedStepList().size(); ++i0)
      {
          current_alignment += controller_msgs.msg.dds.BipedTimedStepMessagePubSubType.getCdrSerializedSize(data.getTimedStepList().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.BipedTimedStepListMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      if(data.getTimedStepList().size() <= 50)
      cdr.write_type_e(data.getTimedStepList());else
          throw new RuntimeException("timed_step_list field exceeds the maximum length: %d > %d".formatted(data.getTimedStepList().size(), 50));

   }

   public static void read(controller_msgs.msg.dds.BipedTimedStepListMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      cdr.read_type_e(data.getTimedStepList());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.BipedTimedStepListMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_e("timed_step_list", data.getTimedStepList());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.BipedTimedStepListMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_e("timed_step_list", data.getTimedStepList());
   }

   public static void staticCopy(controller_msgs.msg.dds.BipedTimedStepListMessage src, controller_msgs.msg.dds.BipedTimedStepListMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.BipedTimedStepListMessage createData()
   {
      return new controller_msgs.msg.dds.BipedTimedStepListMessage();
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
   
   public void serialize(controller_msgs.msg.dds.BipedTimedStepListMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.BipedTimedStepListMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.BipedTimedStepListMessage src, controller_msgs.msg.dds.BipedTimedStepListMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public BipedTimedStepListMessagePubSubType newInstance()
   {
      return new BipedTimedStepListMessagePubSubType();
   }
}
