package quadruped_msgs.msg.dds;

/**
* 
* Topic data type of the struct "QuadrupedRequestedSteppingStateMessage" defined in "QuadrupedRequestedSteppingStateMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from QuadrupedRequestedSteppingStateMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit QuadrupedRequestedSteppingStateMessage_.idl instead.
*
*/
public class QuadrupedRequestedSteppingStateMessagePubSubType implements us.ihmc.pubsub.TopicDataType<quadruped_msgs.msg.dds.QuadrupedRequestedSteppingStateMessage>
{
   public static final java.lang.String name = "quadruped_msgs::msg::dds_::QuadrupedRequestedSteppingStateMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "f5d00ffe3925146a9e08d26938030e7c3b27df4c53b3eaddfb96976cd73ea87c";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(quadruped_msgs.msg.dds.QuadrupedRequestedSteppingStateMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, quadruped_msgs.msg.dds.QuadrupedRequestedSteppingStateMessage data) throws java.io.IOException
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

   public final static int getCdrSerializedSize(quadruped_msgs.msg.dds.QuadrupedRequestedSteppingStateMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(quadruped_msgs.msg.dds.QuadrupedRequestedSteppingStateMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(quadruped_msgs.msg.dds.QuadrupedRequestedSteppingStateMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getQuadrupedSteppingRequestedEvent());

   }

   public static void read(quadruped_msgs.msg.dds.QuadrupedRequestedSteppingStateMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setQuadrupedSteppingRequestedEvent(cdr.read_type_9());
      	

   }

   @Override
   public final void serialize(quadruped_msgs.msg.dds.QuadrupedRequestedSteppingStateMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("quadruped_stepping_requested_event", data.getQuadrupedSteppingRequestedEvent());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, quadruped_msgs.msg.dds.QuadrupedRequestedSteppingStateMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setQuadrupedSteppingRequestedEvent(ser.read_type_9("quadruped_stepping_requested_event"));
   }

   public static void staticCopy(quadruped_msgs.msg.dds.QuadrupedRequestedSteppingStateMessage src, quadruped_msgs.msg.dds.QuadrupedRequestedSteppingStateMessage dest)
   {
      dest.set(src);
   }

   @Override
   public quadruped_msgs.msg.dds.QuadrupedRequestedSteppingStateMessage createData()
   {
      return new quadruped_msgs.msg.dds.QuadrupedRequestedSteppingStateMessage();
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
   
   public void serialize(quadruped_msgs.msg.dds.QuadrupedRequestedSteppingStateMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(quadruped_msgs.msg.dds.QuadrupedRequestedSteppingStateMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(quadruped_msgs.msg.dds.QuadrupedRequestedSteppingStateMessage src, quadruped_msgs.msg.dds.QuadrupedRequestedSteppingStateMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public QuadrupedRequestedSteppingStateMessagePubSubType newInstance()
   {
      return new QuadrupedRequestedSteppingStateMessagePubSubType();
   }
}
