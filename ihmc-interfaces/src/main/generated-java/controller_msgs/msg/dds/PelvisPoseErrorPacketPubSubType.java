package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "PelvisPoseErrorPacket" defined in "PelvisPoseErrorPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from PelvisPoseErrorPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit PelvisPoseErrorPacket_.idl instead.
*
*/
public class PelvisPoseErrorPacketPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.PelvisPoseErrorPacket>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::PelvisPoseErrorPacket_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "8a1e2507219677c650ee7505b2f2621f1ea18fd8f1dea16465d5d2ed25096f3e";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.PelvisPoseErrorPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.PelvisPoseErrorPacket data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.PelvisPoseErrorPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.PelvisPoseErrorPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.PelvisPoseErrorPacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_5(data.getResidualError());

      cdr.write_type_5(data.getTotalError());

      cdr.write_type_7(data.getHasMapBeenReset());

   }

   public static void read(controller_msgs.msg.dds.PelvisPoseErrorPacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setResidualError(cdr.read_type_5());
      	
      data.setTotalError(cdr.read_type_5());
      	
      data.setHasMapBeenReset(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.PelvisPoseErrorPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_5("residual_error", data.getResidualError());
      ser.write_type_5("total_error", data.getTotalError());
      ser.write_type_7("has_map_been_reset", data.getHasMapBeenReset());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.PelvisPoseErrorPacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setResidualError(ser.read_type_5("residual_error"));
      data.setTotalError(ser.read_type_5("total_error"));
      data.setHasMapBeenReset(ser.read_type_7("has_map_been_reset"));
   }

   public static void staticCopy(controller_msgs.msg.dds.PelvisPoseErrorPacket src, controller_msgs.msg.dds.PelvisPoseErrorPacket dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.PelvisPoseErrorPacket createData()
   {
      return new controller_msgs.msg.dds.PelvisPoseErrorPacket();
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
   
   public void serialize(controller_msgs.msg.dds.PelvisPoseErrorPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.PelvisPoseErrorPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.PelvisPoseErrorPacket src, controller_msgs.msg.dds.PelvisPoseErrorPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public PelvisPoseErrorPacketPubSubType newInstance()
   {
      return new PelvisPoseErrorPacketPubSubType();
   }
}
