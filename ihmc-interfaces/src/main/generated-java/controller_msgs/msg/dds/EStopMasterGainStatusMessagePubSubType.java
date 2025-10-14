package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "EStopMasterGainStatusMessage" defined in "EStopMasterGainStatusMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from EStopMasterGainStatusMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit EStopMasterGainStatusMessage_.idl instead.
*
*/
public class EStopMasterGainStatusMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.EStopMasterGainStatusMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::EStopMasterGainStatusMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "4ea3b33a261e3747c0ce86a506b02ddd29be3658945dadfcd67af1e492b6e744";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.EStopMasterGainStatusMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.EStopMasterGainStatusMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.EStopMasterGainStatusMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.EStopMasterGainStatusMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.EStopMasterGainStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_7(data.getEstop());

      cdr.write_type_6(data.getMasterGain());

   }

   public static void read(controller_msgs.msg.dds.EStopMasterGainStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setEstop(cdr.read_type_7());
      	
      data.setMasterGain(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.EStopMasterGainStatusMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_7("estop", data.getEstop());
      ser.write_type_6("master_gain", data.getMasterGain());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.EStopMasterGainStatusMessage data)
   {
      data.setEstop(ser.read_type_7("estop"));
      data.setMasterGain(ser.read_type_6("master_gain"));
   }

   public static void staticCopy(controller_msgs.msg.dds.EStopMasterGainStatusMessage src, controller_msgs.msg.dds.EStopMasterGainStatusMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.EStopMasterGainStatusMessage createData()
   {
      return new controller_msgs.msg.dds.EStopMasterGainStatusMessage();
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
   
   public void serialize(controller_msgs.msg.dds.EStopMasterGainStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.EStopMasterGainStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.EStopMasterGainStatusMessage src, controller_msgs.msg.dds.EStopMasterGainStatusMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public EStopMasterGainStatusMessagePubSubType newInstance()
   {
      return new EStopMasterGainStatusMessagePubSubType();
   }
}
