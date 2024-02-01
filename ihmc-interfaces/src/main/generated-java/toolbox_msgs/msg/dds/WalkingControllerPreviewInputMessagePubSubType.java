package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "WalkingControllerPreviewInputMessage" defined in "WalkingControllerPreviewInputMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from WalkingControllerPreviewInputMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit WalkingControllerPreviewInputMessage_.idl instead.
*
*/
public class WalkingControllerPreviewInputMessagePubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.WalkingControllerPreviewInputMessage>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::WalkingControllerPreviewInputMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "f415c52b1c0828fc0e3ec069036da2b26b4c4eb2e188fbf16691d38850e5756c";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.WalkingControllerPreviewInputMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.WalkingControllerPreviewInputMessage data) throws java.io.IOException
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

      current_alignment += controller_msgs.msg.dds.FootstepDataListMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.WalkingControllerPreviewInputMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.WalkingControllerPreviewInputMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += controller_msgs.msg.dds.FootstepDataListMessagePubSubType.getCdrSerializedSize(data.getFootsteps(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.WalkingControllerPreviewInputMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      controller_msgs.msg.dds.FootstepDataListMessagePubSubType.write(data.getFootsteps(), cdr);
   }

   public static void read(toolbox_msgs.msg.dds.WalkingControllerPreviewInputMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      controller_msgs.msg.dds.FootstepDataListMessagePubSubType.read(data.getFootsteps(), cdr);	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.WalkingControllerPreviewInputMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_a("footsteps", new controller_msgs.msg.dds.FootstepDataListMessagePubSubType(), data.getFootsteps());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.WalkingControllerPreviewInputMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_a("footsteps", new controller_msgs.msg.dds.FootstepDataListMessagePubSubType(), data.getFootsteps());

   }

   public static void staticCopy(toolbox_msgs.msg.dds.WalkingControllerPreviewInputMessage src, toolbox_msgs.msg.dds.WalkingControllerPreviewInputMessage dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.WalkingControllerPreviewInputMessage createData()
   {
      return new toolbox_msgs.msg.dds.WalkingControllerPreviewInputMessage();
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
   
   public void serialize(toolbox_msgs.msg.dds.WalkingControllerPreviewInputMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.WalkingControllerPreviewInputMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.WalkingControllerPreviewInputMessage src, toolbox_msgs.msg.dds.WalkingControllerPreviewInputMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public WalkingControllerPreviewInputMessagePubSubType newInstance()
   {
      return new WalkingControllerPreviewInputMessagePubSubType();
   }
}
