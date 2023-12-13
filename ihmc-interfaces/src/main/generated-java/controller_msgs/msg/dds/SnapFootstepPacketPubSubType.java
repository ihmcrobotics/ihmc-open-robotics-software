package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "SnapFootstepPacket" defined in "SnapFootstepPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from SnapFootstepPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit SnapFootstepPacket_.idl instead.
*
*/
public class SnapFootstepPacketPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.SnapFootstepPacket>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::SnapFootstepPacket_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "a6b46e3cc735cacc9327182cdf14c12d5b59a1b9af937c67ba7b9a8e75e3ddab";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.SnapFootstepPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.SnapFootstepPacket data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += controller_msgs.msg.dds.FootstepDataMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.SnapFootstepPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.SnapFootstepPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getFootstepData().size(); ++i0)
      {
          current_alignment += controller_msgs.msg.dds.FootstepDataMessagePubSubType.getCdrSerializedSize(data.getFootstepData().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getFootstepOrder().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getFlag().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.SnapFootstepPacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      if(data.getFootstepData().size() <= 100)
      cdr.write_type_e(data.getFootstepData());else
          throw new RuntimeException("footstep_data field exceeds the maximum length");

      if(data.getFootstepOrder().size() <= 100)
      cdr.write_type_e(data.getFootstepOrder());else
          throw new RuntimeException("footstep_order field exceeds the maximum length");

      if(data.getFlag().size() <= 100)
      cdr.write_type_e(data.getFlag());else
          throw new RuntimeException("flag field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.SnapFootstepPacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      cdr.read_type_e(data.getFootstepData());	
      cdr.read_type_e(data.getFootstepOrder());	
      cdr.read_type_e(data.getFlag());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.SnapFootstepPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_e("footstep_data", data.getFootstepData());
      ser.write_type_e("footstep_order", data.getFootstepOrder());
      ser.write_type_e("flag", data.getFlag());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.SnapFootstepPacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_e("footstep_data", data.getFootstepData());
      ser.read_type_e("footstep_order", data.getFootstepOrder());
      ser.read_type_e("flag", data.getFlag());
   }

   public static void staticCopy(controller_msgs.msg.dds.SnapFootstepPacket src, controller_msgs.msg.dds.SnapFootstepPacket dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.SnapFootstepPacket createData()
   {
      return new controller_msgs.msg.dds.SnapFootstepPacket();
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
   
   public void serialize(controller_msgs.msg.dds.SnapFootstepPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.SnapFootstepPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.SnapFootstepPacket src, controller_msgs.msg.dds.SnapFootstepPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public SnapFootstepPacketPubSubType newInstance()
   {
      return new SnapFootstepPacketPubSubType();
   }
}
