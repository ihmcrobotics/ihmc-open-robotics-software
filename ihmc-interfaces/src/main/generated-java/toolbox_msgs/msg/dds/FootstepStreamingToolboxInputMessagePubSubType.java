package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FootstepStreamingToolboxInputMessage" defined in "FootstepStreamingToolboxInputMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FootstepStreamingToolboxInputMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FootstepStreamingToolboxInputMessage_.idl instead.
*
*/
public class FootstepStreamingToolboxInputMessagePubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.FootstepStreamingToolboxInputMessage>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::FootstepStreamingToolboxInputMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "f708e69c0e473b66556bc5c39327c96d875260c61c70b1008447a7bea0f931e4";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.FootstepStreamingToolboxInputMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.FootstepStreamingToolboxInputMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 10; ++i0)
      {
          current_alignment += toolbox_msgs.msg.dds.FootstepStreamingToolboxTrackerMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.FootstepStreamingToolboxInputMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.FootstepStreamingToolboxInputMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getTrackers().size(); ++i0)
      {
          current_alignment += toolbox_msgs.msg.dds.FootstepStreamingToolboxTrackerMessagePubSubType.getCdrSerializedSize(data.getTrackers().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.FootstepStreamingToolboxInputMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_12(data.getSequenceId());

      if(data.getTrackers().size() <= 10)
      cdr.write_type_e(data.getTrackers());else
          throw new RuntimeException("trackers field exceeds the maximum length");

   }

   public static void read(toolbox_msgs.msg.dds.FootstepStreamingToolboxInputMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_12());
      	
      cdr.read_type_e(data.getTrackers());	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.FootstepStreamingToolboxInputMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_12("sequence_id", data.getSequenceId());
      ser.write_type_e("trackers", data.getTrackers());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.FootstepStreamingToolboxInputMessage data)
   {
      data.setSequenceId(ser.read_type_12("sequence_id"));
      ser.read_type_e("trackers", data.getTrackers());
   }

   public static void staticCopy(toolbox_msgs.msg.dds.FootstepStreamingToolboxInputMessage src, toolbox_msgs.msg.dds.FootstepStreamingToolboxInputMessage dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.FootstepStreamingToolboxInputMessage createData()
   {
      return new toolbox_msgs.msg.dds.FootstepStreamingToolboxInputMessage();
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
   
   public void serialize(toolbox_msgs.msg.dds.FootstepStreamingToolboxInputMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.FootstepStreamingToolboxInputMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.FootstepStreamingToolboxInputMessage src, toolbox_msgs.msg.dds.FootstepStreamingToolboxInputMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FootstepStreamingToolboxInputMessagePubSubType newInstance()
   {
      return new FootstepStreamingToolboxInputMessagePubSubType();
   }
}
