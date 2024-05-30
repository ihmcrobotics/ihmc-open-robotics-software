package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "KinematicsStreamingToolboxInputMessage" defined in "KinematicsStreamingToolboxInputMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from KinematicsStreamingToolboxInputMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit KinematicsStreamingToolboxInputMessage_.idl instead.
*
*/
public class KinematicsStreamingToolboxInputMessagePubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.KinematicsStreamingToolboxInputMessage>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::KinematicsStreamingToolboxInputMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "3aa12b68ff0ca8b8fb903de4c748a1684dd6fef0a114b419761ea66dedc7004c";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.KinematicsStreamingToolboxInputMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.KinematicsStreamingToolboxInputMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 10; ++i0)
      {
          current_alignment += toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.KinematicsStreamingToolboxInputMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.KinematicsStreamingToolboxInputMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getInputs().size(); ++i0)
      {
          current_alignment += toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessagePubSubType.getCdrSerializedSize(data.getInputs().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.KinematicsStreamingToolboxInputMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_11(data.getTimestamp());

      cdr.write_type_7(data.getStreamToController());

      cdr.write_type_6(data.getStreamInitialBlendDuration());

      cdr.write_type_6(data.getAngularRateLimitation());

      cdr.write_type_6(data.getLinearRateLimitation());

      if(data.getInputs().size() <= 10)
      cdr.write_type_e(data.getInputs());else
          throw new RuntimeException("inputs field exceeds the maximum length");

   }

   public static void read(toolbox_msgs.msg.dds.KinematicsStreamingToolboxInputMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setTimestamp(cdr.read_type_11());
      	
      data.setStreamToController(cdr.read_type_7());
      	
      data.setStreamInitialBlendDuration(cdr.read_type_6());
      	
      data.setAngularRateLimitation(cdr.read_type_6());
      	
      data.setLinearRateLimitation(cdr.read_type_6());
      	
      cdr.read_type_e(data.getInputs());	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.KinematicsStreamingToolboxInputMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_11("timestamp", data.getTimestamp());
      ser.write_type_7("stream_to_controller", data.getStreamToController());
      ser.write_type_6("stream_initial_blend_duration", data.getStreamInitialBlendDuration());
      ser.write_type_6("angular_rate_limitation", data.getAngularRateLimitation());
      ser.write_type_6("linear_rate_limitation", data.getLinearRateLimitation());
      ser.write_type_e("inputs", data.getInputs());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.KinematicsStreamingToolboxInputMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setTimestamp(ser.read_type_11("timestamp"));
      data.setStreamToController(ser.read_type_7("stream_to_controller"));
      data.setStreamInitialBlendDuration(ser.read_type_6("stream_initial_blend_duration"));
      data.setAngularRateLimitation(ser.read_type_6("angular_rate_limitation"));
      data.setLinearRateLimitation(ser.read_type_6("linear_rate_limitation"));
      ser.read_type_e("inputs", data.getInputs());
   }

   public static void staticCopy(toolbox_msgs.msg.dds.KinematicsStreamingToolboxInputMessage src, toolbox_msgs.msg.dds.KinematicsStreamingToolboxInputMessage dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.KinematicsStreamingToolboxInputMessage createData()
   {
      return new toolbox_msgs.msg.dds.KinematicsStreamingToolboxInputMessage();
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
   
   public void serialize(toolbox_msgs.msg.dds.KinematicsStreamingToolboxInputMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.KinematicsStreamingToolboxInputMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.KinematicsStreamingToolboxInputMessage src, toolbox_msgs.msg.dds.KinematicsStreamingToolboxInputMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public KinematicsStreamingToolboxInputMessagePubSubType newInstance()
   {
      return new KinematicsStreamingToolboxInputMessagePubSubType();
   }
}
