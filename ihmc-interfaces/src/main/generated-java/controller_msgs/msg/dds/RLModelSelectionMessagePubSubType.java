package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "RLModelSelectionMessage" defined in "RLModelSelectionMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from RLModelSelectionMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit RLModelSelectionMessage_.idl instead.
*
*/
public class RLModelSelectionMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.RLModelSelectionMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::RLModelSelectionMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "ae45cdcf08b814fdf9b15830dd1e17aa3cf27fb2f9266ff0e99e4685cd23ff7d";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.RLModelSelectionMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.RLModelSelectionMessage data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.RLModelSelectionMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.RLModelSelectionMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.RLModelSelectionMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_9(data.getDesiredModel());

      cdr.write_type_7(data.getExecuteDesiredModel());

   }

   public static void read(controller_msgs.msg.dds.RLModelSelectionMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setDesiredModel(cdr.read_type_9());
      	
      data.setExecuteDesiredModel(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.RLModelSelectionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_9("desired_model", data.getDesiredModel());
      ser.write_type_7("execute_desired_model", data.getExecuteDesiredModel());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.RLModelSelectionMessage data)
   {
      data.setDesiredModel(ser.read_type_9("desired_model"));
      data.setExecuteDesiredModel(ser.read_type_7("execute_desired_model"));
   }

   public static void staticCopy(controller_msgs.msg.dds.RLModelSelectionMessage src, controller_msgs.msg.dds.RLModelSelectionMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.RLModelSelectionMessage createData()
   {
      return new controller_msgs.msg.dds.RLModelSelectionMessage();
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
   
   public void serialize(controller_msgs.msg.dds.RLModelSelectionMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.RLModelSelectionMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.RLModelSelectionMessage src, controller_msgs.msg.dds.RLModelSelectionMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public RLModelSelectionMessagePubSubType newInstance()
   {
      return new RLModelSelectionMessagePubSubType();
   }
}
