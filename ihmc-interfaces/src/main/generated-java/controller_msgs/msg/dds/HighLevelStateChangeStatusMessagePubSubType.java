package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "HighLevelStateChangeStatusMessage" defined in "HighLevelStateChangeStatusMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from HighLevelStateChangeStatusMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit HighLevelStateChangeStatusMessage_.idl instead.
*
*/
public class HighLevelStateChangeStatusMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.HighLevelStateChangeStatusMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::HighLevelStateChangeStatusMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "5d64b0f92d0a14fcf5e303fe0c99a324e7bfc4508e6e7477a88c8370ca93606c";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.HighLevelStateChangeStatusMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.HighLevelStateChangeStatusMessage data) throws java.io.IOException
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


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.HighLevelStateChangeStatusMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.HighLevelStateChangeStatusMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.HighLevelStateChangeStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getInitialHighLevelControllerName());

      cdr.write_type_9(data.getEndHighLevelControllerName());

   }

   public static void read(controller_msgs.msg.dds.HighLevelStateChangeStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setInitialHighLevelControllerName(cdr.read_type_9());
      	
      data.setEndHighLevelControllerName(cdr.read_type_9());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.HighLevelStateChangeStatusMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("initial_high_level_controller_name", data.getInitialHighLevelControllerName());
      ser.write_type_9("end_high_level_controller_name", data.getEndHighLevelControllerName());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.HighLevelStateChangeStatusMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setInitialHighLevelControllerName(ser.read_type_9("initial_high_level_controller_name"));
      data.setEndHighLevelControllerName(ser.read_type_9("end_high_level_controller_name"));
   }

   public static void staticCopy(controller_msgs.msg.dds.HighLevelStateChangeStatusMessage src, controller_msgs.msg.dds.HighLevelStateChangeStatusMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.HighLevelStateChangeStatusMessage createData()
   {
      return new controller_msgs.msg.dds.HighLevelStateChangeStatusMessage();
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
   
   public void serialize(controller_msgs.msg.dds.HighLevelStateChangeStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.HighLevelStateChangeStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.HighLevelStateChangeStatusMessage src, controller_msgs.msg.dds.HighLevelStateChangeStatusMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public HighLevelStateChangeStatusMessagePubSubType newInstance()
   {
      return new HighLevelStateChangeStatusMessagePubSubType();
   }
}
