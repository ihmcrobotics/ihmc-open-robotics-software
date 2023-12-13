package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "DrillDetectionPacket" defined in "DrillDetectionPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from DrillDetectionPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit DrillDetectionPacket_.idl instead.
*
*/
public class DrillDetectionPacketPubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.DrillDetectionPacket>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::DrillDetectionPacket_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "989bf816fca52ac22ff98f5d1e2fe99d0efc7db1efbcdfbc7410b0ae98526415";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.DrillDetectionPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.DrillDetectionPacket data) throws java.io.IOException
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

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.DrillDetectionPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.DrillDetectionPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.DrillDetectionPacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_7(data.getIsDrillOn());

   }

   public static void read(perception_msgs.msg.dds.DrillDetectionPacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setIsDrillOn(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.DrillDetectionPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_7("is_drill_on", data.getIsDrillOn());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.DrillDetectionPacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setIsDrillOn(ser.read_type_7("is_drill_on"));
   }

   public static void staticCopy(perception_msgs.msg.dds.DrillDetectionPacket src, perception_msgs.msg.dds.DrillDetectionPacket dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.DrillDetectionPacket createData()
   {
      return new perception_msgs.msg.dds.DrillDetectionPacket();
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
   
   public void serialize(perception_msgs.msg.dds.DrillDetectionPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.DrillDetectionPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.DrillDetectionPacket src, perception_msgs.msg.dds.DrillDetectionPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public DrillDetectionPacketPubSubType newInstance()
   {
      return new DrillDetectionPacketPubSubType();
   }
}
