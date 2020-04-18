package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "PauseWalkingMessage" defined in "PauseWalkingMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from PauseWalkingMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit PauseWalkingMessage_.idl instead.
*
*/
public class PauseWalkingMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.PauseWalkingMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::PauseWalkingMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.PauseWalkingMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.PauseWalkingMessage data) throws java.io.IOException
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

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.PauseWalkingMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.PauseWalkingMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.PauseWalkingMessage data, us.ihmc.idl.CDR cdr)
   {

      cdr.write_type_4(data.getSequenceId());


      cdr.write_type_7(data.getPause());

   }

   public static void read(controller_msgs.msg.dds.PauseWalkingMessage data, us.ihmc.idl.CDR cdr)
   {

      data.setSequenceId(cdr.read_type_4());
      	

      data.setPause(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.PauseWalkingMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {

      ser.write_type_4("sequence_id", data.getSequenceId());

      ser.write_type_7("pause", data.getPause());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.PauseWalkingMessage data)
   {

      data.setSequenceId(ser.read_type_4("sequence_id"));

      data.setPause(ser.read_type_7("pause"));
   }

   public static void staticCopy(controller_msgs.msg.dds.PauseWalkingMessage src, controller_msgs.msg.dds.PauseWalkingMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.PauseWalkingMessage createData()
   {
      return new controller_msgs.msg.dds.PauseWalkingMessage();
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
   
   public void serialize(controller_msgs.msg.dds.PauseWalkingMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.PauseWalkingMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.PauseWalkingMessage src, controller_msgs.msg.dds.PauseWalkingMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public PauseWalkingMessagePubSubType newInstance()
   {
      return new PauseWalkingMessagePubSubType();
   }
}
