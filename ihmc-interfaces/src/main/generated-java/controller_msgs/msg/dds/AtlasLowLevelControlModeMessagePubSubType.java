package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "AtlasLowLevelControlModeMessage" defined in "AtlasLowLevelControlModeMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from AtlasLowLevelControlModeMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit AtlasLowLevelControlModeMessage_.idl instead.
*
*/
public class AtlasLowLevelControlModeMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.AtlasLowLevelControlModeMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::AtlasLowLevelControlModeMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.AtlasLowLevelControlModeMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.AtlasLowLevelControlModeMessage data) throws java.io.IOException
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

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.AtlasLowLevelControlModeMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.AtlasLowLevelControlModeMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.AtlasLowLevelControlModeMessage data, us.ihmc.idl.CDR cdr)
   {

      cdr.write_type_4(data.getSequenceId());


      cdr.write_type_9(data.getRequestedAtlasLowLevelControlMode());

   }

   public static void read(controller_msgs.msg.dds.AtlasLowLevelControlModeMessage data, us.ihmc.idl.CDR cdr)
   {

      data.setSequenceId(cdr.read_type_4());
      	

      data.setRequestedAtlasLowLevelControlMode(cdr.read_type_9());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.AtlasLowLevelControlModeMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {

      ser.write_type_4("sequence_id", data.getSequenceId());

      ser.write_type_9("requested_atlas_low_level_control_mode", data.getRequestedAtlasLowLevelControlMode());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.AtlasLowLevelControlModeMessage data)
   {

      data.setSequenceId(ser.read_type_4("sequence_id"));

      data.setRequestedAtlasLowLevelControlMode(ser.read_type_9("requested_atlas_low_level_control_mode"));
   }

   public static void staticCopy(controller_msgs.msg.dds.AtlasLowLevelControlModeMessage src, controller_msgs.msg.dds.AtlasLowLevelControlModeMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.AtlasLowLevelControlModeMessage createData()
   {
      return new controller_msgs.msg.dds.AtlasLowLevelControlModeMessage();
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
   
   public void serialize(controller_msgs.msg.dds.AtlasLowLevelControlModeMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.AtlasLowLevelControlModeMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.AtlasLowLevelControlModeMessage src, controller_msgs.msg.dds.AtlasLowLevelControlModeMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public AtlasLowLevelControlModeMessagePubSubType newInstance()
   {
      return new AtlasLowLevelControlModeMessagePubSubType();
   }
}
