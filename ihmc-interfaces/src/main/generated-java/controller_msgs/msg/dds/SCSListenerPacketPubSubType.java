package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "SCSListenerPacket" defined in "SCSListenerPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from SCSListenerPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit SCSListenerPacket_.idl instead.
*
*/
public class SCSListenerPacketPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.SCSListenerPacket>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::SCSListenerPacket_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "01b80c91854fa3d31a11becfd0b0499e16631e413b5501e0e21a41ed339b037b";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.SCSListenerPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.SCSListenerPacket data) throws java.io.IOException
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

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.SCSListenerPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.SCSListenerPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.SCSListenerPacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_7(data.getIsStopped());

   }

   public static void read(controller_msgs.msg.dds.SCSListenerPacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setIsStopped(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.SCSListenerPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_7("is_stopped", data.getIsStopped());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.SCSListenerPacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setIsStopped(ser.read_type_7("is_stopped"));
   }

   public static void staticCopy(controller_msgs.msg.dds.SCSListenerPacket src, controller_msgs.msg.dds.SCSListenerPacket dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.SCSListenerPacket createData()
   {
      return new controller_msgs.msg.dds.SCSListenerPacket();
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
   
   public void serialize(controller_msgs.msg.dds.SCSListenerPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.SCSListenerPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.SCSListenerPacket src, controller_msgs.msg.dds.SCSListenerPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public SCSListenerPacketPubSubType newInstance()
   {
      return new SCSListenerPacketPubSubType();
   }
}
