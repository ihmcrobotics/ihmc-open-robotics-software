package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "DirectionalControlInputMessage" defined in "DirectionalControlInputMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from DirectionalControlInputMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit DirectionalControlInputMessage_.idl instead.
*
*/
public class DirectionalControlInputMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.DirectionalControlInputMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::DirectionalControlInputMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "d4214e040772434f3f45ae142120672b9cbbb0c55270ad03cc9ad8104ae4eca5";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.DirectionalControlInputMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.DirectionalControlInputMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.DirectionalControlInputMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.DirectionalControlInputMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.DirectionalControlInputMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_6(data.getForward());

      cdr.write_type_6(data.getRight());

      cdr.write_type_6(data.getClockwise());

   }

   public static void read(controller_msgs.msg.dds.DirectionalControlInputMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setForward(cdr.read_type_6());
      	
      data.setRight(cdr.read_type_6());
      	
      data.setClockwise(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.DirectionalControlInputMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_6("forward", data.getForward());
      ser.write_type_6("right", data.getRight());
      ser.write_type_6("clockwise", data.getClockwise());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.DirectionalControlInputMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setForward(ser.read_type_6("forward"));
      data.setRight(ser.read_type_6("right"));
      data.setClockwise(ser.read_type_6("clockwise"));
   }

   public static void staticCopy(controller_msgs.msg.dds.DirectionalControlInputMessage src, controller_msgs.msg.dds.DirectionalControlInputMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.DirectionalControlInputMessage createData()
   {
      return new controller_msgs.msg.dds.DirectionalControlInputMessage();
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
   
   public void serialize(controller_msgs.msg.dds.DirectionalControlInputMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.DirectionalControlInputMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.DirectionalControlInputMessage src, controller_msgs.msg.dds.DirectionalControlInputMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public DirectionalControlInputMessagePubSubType newInstance()
   {
      return new DirectionalControlInputMessagePubSubType();
   }
}
