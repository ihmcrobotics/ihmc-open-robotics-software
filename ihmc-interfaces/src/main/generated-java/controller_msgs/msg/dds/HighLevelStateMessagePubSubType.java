package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "HighLevelStateMessage" defined in "HighLevelStateMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from HighLevelStateMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit HighLevelStateMessage_.idl instead.
*
*/
public class HighLevelStateMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.HighLevelStateMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::HighLevelStateMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "0ebd8b9922c7fe452c5c2d30ea52160eb6d2e608230f8305cf04ee12238fdb12";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.HighLevelStateMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.HighLevelStateMessage data) throws java.io.IOException
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


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.HighLevelStateMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.HighLevelStateMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.HighLevelStateMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getHighLevelControllerName());

   }

   public static void read(controller_msgs.msg.dds.HighLevelStateMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setHighLevelControllerName(cdr.read_type_9());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.HighLevelStateMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("high_level_controller_name", data.getHighLevelControllerName());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.HighLevelStateMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setHighLevelControllerName(ser.read_type_9("high_level_controller_name"));
   }

   public static void staticCopy(controller_msgs.msg.dds.HighLevelStateMessage src, controller_msgs.msg.dds.HighLevelStateMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.HighLevelStateMessage createData()
   {
      return new controller_msgs.msg.dds.HighLevelStateMessage();
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
   
   public void serialize(controller_msgs.msg.dds.HighLevelStateMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.HighLevelStateMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.HighLevelStateMessage src, controller_msgs.msg.dds.HighLevelStateMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public HighLevelStateMessagePubSubType newInstance()
   {
      return new HighLevelStateMessagePubSubType();
   }
}
