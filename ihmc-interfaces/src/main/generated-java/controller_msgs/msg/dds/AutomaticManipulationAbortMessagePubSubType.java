package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "AutomaticManipulationAbortMessage" defined in "AutomaticManipulationAbortMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from AutomaticManipulationAbortMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit AutomaticManipulationAbortMessage_.idl instead.
*
*/
public class AutomaticManipulationAbortMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.AutomaticManipulationAbortMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::AutomaticManipulationAbortMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "719ae080a28c22f4da2fdcf7a669f31334e333a17b6c6502798dd33b117abe65";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.AutomaticManipulationAbortMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.AutomaticManipulationAbortMessage data) throws java.io.IOException
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

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.AutomaticManipulationAbortMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.AutomaticManipulationAbortMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.AutomaticManipulationAbortMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_7(data.getEnable());

   }

   public static void read(controller_msgs.msg.dds.AutomaticManipulationAbortMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setEnable(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.AutomaticManipulationAbortMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_7("enable", data.getEnable());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.AutomaticManipulationAbortMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setEnable(ser.read_type_7("enable"));
   }

   public static void staticCopy(controller_msgs.msg.dds.AutomaticManipulationAbortMessage src, controller_msgs.msg.dds.AutomaticManipulationAbortMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.AutomaticManipulationAbortMessage createData()
   {
      return new controller_msgs.msg.dds.AutomaticManipulationAbortMessage();
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
   
   public void serialize(controller_msgs.msg.dds.AutomaticManipulationAbortMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.AutomaticManipulationAbortMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.AutomaticManipulationAbortMessage src, controller_msgs.msg.dds.AutomaticManipulationAbortMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public AutomaticManipulationAbortMessagePubSubType newInstance()
   {
      return new AutomaticManipulationAbortMessagePubSubType();
   }
}
